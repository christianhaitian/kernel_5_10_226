// SPDX-License-Identifier: GPL-2.0-only
/*
 * RG56 Pro built-in gamepad driver
 *
 * Input driver for the RG56 Pro handheld's integrated gamepad.
 * Reads analog sticks and triggers via IIO (SARADC), ADC-threshold buttons
 * via IIO, and GPIO-connected buttons via IRQ with debounce.
 *
 * Features:
 * - IRQ-driven GPIO buttons with configurable debounce
 * - Force feedback (rumble) via GPIO-connected motors
 * - Right stick axis inversion (DT properties)
 * - Left/right stick X/Y axis swap (DT properties)
 * - Axis-to-dpad mode via sysfs
 *
 * Compatible with the "play_joystick" device tree node shipped in the
 * stock RK3562 firmware.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/iio/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/pm.h>

#define DRIVER_NAME	"rg56pro-joystick"

/* Number of stick axes (LX, LY, RX, RY) */
#define NUM_STICK_CHANS	4
/* Number of trigger axes (L2, R2) */
#define NUM_TRIG_CHANS	2
/* Number of ADC-threshold buttons (dpad L/R/D, B, X, Y) */
#define NUM_ADC_BTNS	6
/* Number of GPIO buttons */
#define NUM_GPIO_BTNS	10

/* Reported axis range to userspace */
#define AXIS_MIN	-32767
#define AXIS_MAX	32767
#define TRIG_MIN	0
#define TRIG_MAX	32767

/* Axis-to-dpad threshold: ~55% of 32767 ≈ 18000 */
#define DPAD_THRESHOLD	18000

/* Default debounce interval in ms (manufacturer hardcodes 10 despite DT 100) */
#define DEFAULT_DEBOUNCE_MS	10

/* DTS key codes that need remapping to BTN_* range for joydev visibility */
#define KEY_HOME_DTS	102
#define KEY_FN_DTS	464

struct rg56pro_joystick;

struct rg56pro_gpio_btn {
	struct gpio_desc *gpiod;
	unsigned int code;
	int irq;
	struct delayed_work work;
	struct rg56pro_joystick *joy;
};

struct rg56pro_joystick {
	struct device *dev;
	struct input_dev *input;

	/* IIO channels */
	struct iio_channel *stick_chans[NUM_STICK_CHANS];
	struct iio_channel *trig_chans[NUM_TRIG_CHANS];
	struct iio_channel *adc_btn_chans[NUM_ADC_BTNS];

	/* GPIO buttons (IRQ-driven) */
	struct rg56pro_gpio_btn gpio_btns[NUM_GPIO_BTNS];
	unsigned int debounce_ms;

	/* ADC button codes and thresholds */
	unsigned int adc_btn_codes[NUM_ADC_BTNS];
	int adc_btn_thresh[NUM_ADC_BTNS]; /* in microvolts */

	/* Stick calibration (millivolts from DTS, converted to microvolts) */
	int axis_min_uv;
	int axis_max_uv;
	int axis_dz_lo_uv;  /* dead zone low */
	int axis_dz_hi_uv;  /* dead zone high */

	/* Trigger calibration (millivolts from DTS, converted to microvolts) */
	int trig_min_uv;
	int trig_max_uv;

	/* Axis inversion flags */
	bool lx_swap;
	bool ly_swap;
	bool rx_swap;
	bool ry_swap;

	/* Axis X/Y swap flags */
	bool l_xy_swap;
	bool r_xy_swap;

	/* Rumble motors (optional) */
	struct gpio_desc *moto_gpio;
	struct gpio_desc *moto_r_gpio;
	bool rumble_on;

	/* Axis-to-dpad mode */
	bool axis_to_dpad;
	bool last_dpad_up;
	bool last_dpad_down;
	bool last_dpad_left;
	bool last_dpad_right;
};

static const char * const stick_chan_names[NUM_STICK_CHANS] = {
	"button0", "button1", "button2", "button3",
};

static const char * const trig_chan_names[NUM_TRIG_CHANS] = {
	"l2-abs", "r2-abs",
};

/* ADC button channel names (same order as io-channel-names in DTS) */
static const char * const adc_btn_chan_names[NUM_ADC_BTNS] = {
	"left-key", "right-key", "down-key", "b-key", "x-key", "y-key",
};

/* Base axis ABS codes for sticks (before XY swap) */
static const unsigned int stick_abs_codes[NUM_STICK_CHANS] = {
	ABS_X, ABS_Y, ABS_RX, ABS_RY,
};

/* Axis ABS codes for triggers */
static const unsigned int trig_abs_codes[NUM_TRIG_CHANS] = {
	ABS_Z, ABS_RZ,
};

/*
 * Return the ABS code for stick channel @i, accounting for XY swap.
 */
static unsigned int rg56pro_stick_code(struct rg56pro_joystick *joy, int i)
{
	switch (i) {
	case 0: /* LX */
		return joy->l_xy_swap ? ABS_Y : ABS_X;
	case 1: /* LY */
		return joy->l_xy_swap ? ABS_X : ABS_Y;
	case 2: /* RX */
		return joy->r_xy_swap ? ABS_RY : ABS_RX;
	case 3: /* RY */
		return joy->r_xy_swap ? ABS_RX : ABS_RY;
	default:
		return stick_abs_codes[i];
	}
}

/*
 * Map a raw microvolt ADC reading to the [-32767, 32767] axis range.
 * Values within the dead zone map to 0.
 */
static int rg56pro_map_stick(struct rg56pro_joystick *joy, int uv, bool swap)
{
	int val;

	/* Clamp to calibration range */
	if (uv < joy->axis_min_uv)
		uv = joy->axis_min_uv;
	if (uv > joy->axis_max_uv)
		uv = joy->axis_max_uv;

	/* Dead zone → 0 */
	if (uv >= joy->axis_dz_lo_uv && uv <= joy->axis_dz_hi_uv)
		return 0;

	if (uv < joy->axis_dz_lo_uv) {
		/* Below dead zone: map [min, dz_lo] → [-32767, 0] */
		val = (int)((long long)(uv - joy->axis_dz_lo_uv) * 32767 /
			    (joy->axis_dz_lo_uv - joy->axis_min_uv));
	} else {
		/* Above dead zone: map [dz_hi, max] → [0, 32767] */
		val = (int)((long long)(uv - joy->axis_dz_hi_uv) * 32767 /
			    (joy->axis_max_uv - joy->axis_dz_hi_uv));
	}

	if (swap)
		val = -val;

	if (val < AXIS_MIN)
		val = AXIS_MIN;
	if (val > AXIS_MAX)
		val = AXIS_MAX;

	return val;
}

/*
 * Map a raw microvolt reading to the [0, 32767] trigger range.
 */
static int rg56pro_map_trigger(struct rg56pro_joystick *joy, int uv)
{
	int val;

	if (uv < joy->trig_min_uv)
		uv = joy->trig_min_uv;
	if (uv > joy->trig_max_uv)
		uv = joy->trig_max_uv;

	val = (int)((long long)(uv - joy->trig_min_uv) * 32767 /
		    (joy->trig_max_uv - joy->trig_min_uv));

	if (val < TRIG_MIN)
		val = TRIG_MIN;
	if (val > TRIG_MAX)
		val = TRIG_MAX;

	return val;
}

/* --- IRQ-driven GPIO buttons --- */

static void rg56pro_gpio_work_func(struct work_struct *work)
{
	struct rg56pro_gpio_btn *btn =
		container_of(work, struct rg56pro_gpio_btn, work.work);
	struct rg56pro_joystick *joy = btn->joy;
	int val;

	val = gpiod_get_value_cansleep(btn->gpiod);
	if (val < 0) {
		dev_err_ratelimited(joy->dev,
				    "GPIO read failed for btn %u: %d\n",
				    btn->code, val);
		return;
	}

	input_report_key(joy->input, btn->code, val);
	input_sync(joy->input);
}

static irqreturn_t rg56pro_gpio_isr(int irq, void *data)
{
	struct rg56pro_gpio_btn *btn = data;
	struct rg56pro_joystick *joy = btn->joy;

	mod_delayed_work(system_wq, &btn->work,
			 msecs_to_jiffies(joy->debounce_ms));

	return IRQ_HANDLED;
}

static void rg56pro_cancel_gpio_work(void *data)
{
	struct rg56pro_joystick *joy = data;
	int i;

	for (i = 0; i < NUM_GPIO_BTNS; i++)
		cancel_delayed_work_sync(&joy->gpio_btns[i].work);
}

/* --- Force feedback (rumble) --- */

static int rg56pro_play_effect(struct input_dev *dev, void *data,
			       struct ff_effect *effect)
{
	struct rg56pro_joystick *joy = input_get_drvdata(dev);
	bool on;

	on = effect->u.rumble.strong_magnitude ||
	     effect->u.rumble.weak_magnitude;

	joy->rumble_on = on;

	if (joy->moto_gpio)
		gpiod_set_value_cansleep(joy->moto_gpio, on);
	if (joy->moto_r_gpio)
		gpiod_set_value_cansleep(joy->moto_r_gpio, on);

	return 0;
}

/* --- Axis-to-dpad sysfs --- */

static ssize_t axis_to_dpad_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct rg56pro_joystick *joy = platform_get_drvdata(to_platform_device(dev));

	return sysfs_emit(buf, "%d\n", joy->axis_to_dpad);
}

static ssize_t axis_to_dpad_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct rg56pro_joystick *joy = platform_get_drvdata(to_platform_device(dev));
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (joy->axis_to_dpad && !val) {
		/* Releasing dpad buttons when disabling mode */
		if (joy->last_dpad_up) {
			input_report_key(joy->input, BTN_DPAD_UP, 0);
			joy->last_dpad_up = false;
		}
		if (joy->last_dpad_down) {
			input_report_key(joy->input, BTN_DPAD_DOWN, 0);
			joy->last_dpad_down = false;
		}
		if (joy->last_dpad_left) {
			input_report_key(joy->input, BTN_DPAD_LEFT, 0);
			joy->last_dpad_left = false;
		}
		if (joy->last_dpad_right) {
			input_report_key(joy->input, BTN_DPAD_RIGHT, 0);
			joy->last_dpad_right = false;
		}
		input_sync(joy->input);
	}

	joy->axis_to_dpad = val;

	return count;
}

static DEVICE_ATTR_RW(axis_to_dpad);

static struct attribute *rg56pro_attrs[] = {
	&dev_attr_axis_to_dpad.attr,
	NULL,
};
ATTRIBUTE_GROUPS(rg56pro);

/* --- Poll function --- */

static void rg56pro_poll(struct input_dev *input)
{
	struct rg56pro_joystick *joy = input_get_drvdata(input);
	int stick_vals[NUM_STICK_CHANS];
	int i, ret, raw;
	bool swap;

	/* Step 1: Read all stick channels and apply inversion */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		ret = iio_read_channel_processed(joy->stick_chans[i], &raw);
		if (ret < 0) {
			stick_vals[i] = 0;
			continue;
		}

		/* iio_read_channel_processed returns millivolts; convert to uV */
		raw *= 1000;

		swap = false;
		if (i == 0)
			swap = joy->lx_swap;
		else if (i == 1)
			swap = joy->ly_swap;
		else if (i == 2)
			swap = joy->rx_swap;
		else if (i == 3)
			swap = joy->ry_swap;

		stick_vals[i] = rg56pro_map_stick(joy, raw, swap);
	}

	/* Step 2: Read and report triggers */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		ret = iio_read_channel_processed(joy->trig_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000;
		input_report_abs(input, trig_abs_codes[i],
				 rg56pro_map_trigger(joy, raw));
	}

	/* Step 3: Read and report ADC-threshold buttons */
	for (i = 0; i < NUM_ADC_BTNS; i++) {
		ret = iio_read_channel_processed(joy->adc_btn_chans[i], &raw);
		if (ret < 0)
			continue;

		raw *= 1000; /* mV → uV */
		input_report_key(input, joy->adc_btn_codes[i],
				 raw < joy->adc_btn_thresh[i] ? 1 : 0);
	}

	/* Step 4: GPIO buttons handled by IRQs, not polled */

	/* Step 5/6: Report stick axes or dpad */
	if (joy->axis_to_dpad) {
		/* Left stick → dpad keys */
		bool up    = stick_vals[1] < -DPAD_THRESHOLD;
		bool down  = stick_vals[1] >  DPAD_THRESHOLD;
		bool left  = stick_vals[0] < -DPAD_THRESHOLD;
		bool right = stick_vals[0] >  DPAD_THRESHOLD;

		if (up != joy->last_dpad_up) {
			input_report_key(input, BTN_DPAD_UP, up);
			joy->last_dpad_up = up;
		}
		if (down != joy->last_dpad_down) {
			input_report_key(input, BTN_DPAD_DOWN, down);
			joy->last_dpad_down = down;
		}
		if (left != joy->last_dpad_left) {
			input_report_key(input, BTN_DPAD_LEFT, left);
			joy->last_dpad_left = left;
		}
		if (right != joy->last_dpad_right) {
			input_report_key(input, BTN_DPAD_RIGHT, right);
			joy->last_dpad_right = right;
		}

		/* Still report left stick as zeroed ABS so apps don't see stale values */
		input_report_abs(input, rg56pro_stick_code(joy, 0), 0);
		input_report_abs(input, rg56pro_stick_code(joy, 1), 0);

		/* Right stick still reports normally */
		input_report_abs(input, rg56pro_stick_code(joy, 2), stick_vals[2]);
		input_report_abs(input, rg56pro_stick_code(joy, 3), stick_vals[3]);
	} else {
		/* Normal mode: report all 4 axes with XY swap */
		for (i = 0; i < NUM_STICK_CHANS; i++)
			input_report_abs(input, rg56pro_stick_code(joy, i),
					 stick_vals[i]);
	}

	/* Step 7: Single sync */
	input_sync(input);
}

/* --- DT parsing --- */

static int rg56pro_parse_adc_buttons(struct rg56pro_joystick *joy,
				     struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *child;
	int i = 0;

	for_each_child_of_node(node, child) {
		u32 code, thresh;

		if (i >= NUM_ADC_BTNS) {
			of_node_put(child);
			break;
		}

		if (of_property_read_u32(child, "linux,code", &code)) {
			dev_warn(dev, "ADC button %s missing linux,code\n",
				 child->name);
			continue;
		}

		if (of_property_read_u32(child, "press-threshold-microvolt",
					 &thresh))
			thresh = 80000; /* default 80 mV */

		joy->adc_btn_codes[i] = code;
		joy->adc_btn_thresh[i] = thresh;
		i++;
	}

	if (i != NUM_ADC_BTNS) {
		dev_err(dev, "Expected %d ADC button children, found %d\n",
			NUM_ADC_BTNS, i);
		return -EINVAL;
	}

	return 0;
}

/*
 * Remap DTS key codes that fall outside the BTN_* range (and would be
 * invisible to joydev) into BTN_TL2/BTN_TR2.
 */
static unsigned int rg56pro_remap_code(unsigned int dts_code)
{
	switch (dts_code) {
	case KEY_HOME_DTS:
		return BTN_TL2;   /* 0x138 — Home → distinct function button */
	case KEY_FN_DTS:
		return BTN_MODE;  /* 0x13c — FN/Menu → hotkey enable in RetroArch */
	default:
		return dts_code;
	}
}

/* --- PM ops --- */

static int rg56pro_suspend(struct device *dev)
{
	struct rg56pro_joystick *joy = dev_get_drvdata(dev);

	/* Turn off motors on suspend */
	if (joy->moto_gpio)
		gpiod_set_value_cansleep(joy->moto_gpio, 0);
	if (joy->moto_r_gpio)
		gpiod_set_value_cansleep(joy->moto_r_gpio, 0);

	return 0;
}

static int rg56pro_resume(struct device *dev)
{
	struct rg56pro_joystick *joy = dev_get_drvdata(dev);

	/* Restore rumble state */
	if (joy->rumble_on) {
		if (joy->moto_gpio)
			gpiod_set_value_cansleep(joy->moto_gpio, 1);
		if (joy->moto_r_gpio)
			gpiod_set_value_cansleep(joy->moto_r_gpio, 1);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(rg56pro_pm_ops, rg56pro_suspend, rg56pro_resume);

/* --- Probe --- */

static int rg56pro_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rg56pro_joystick *joy;
	struct input_dev *input;
	u32 gpio_codes[NUM_GPIO_BTNS];
	u32 poll_interval = 16;
	u32 debounce_ms = DEFAULT_DEBOUNCE_MS;
	int axis_min_mv, axis_max_mv, dz_lo_mv, dz_hi_mv;
	int trig_min_mv, trig_max_mv;
	int i, ret, count;

	joy = devm_kzalloc(dev, sizeof(*joy), GFP_KERNEL);
	if (!joy)
		return -ENOMEM;

	joy->dev = dev;
	platform_set_drvdata(pdev, joy);

	/* --- Parse DT properties --- */

	of_property_read_u32(dev->of_node, "poll-interval", &poll_interval);
	of_property_read_u32(dev->of_node, "debounce-interval", &debounce_ms);
	joy->debounce_ms = debounce_ms;

	/* Axis inversion */
	joy->lx_swap = of_property_read_bool(dev->of_node, "l_x_swap");
	joy->ly_swap = of_property_read_bool(dev->of_node, "l_y_swap");
	joy->rx_swap = of_property_read_bool(dev->of_node, "r_x_swap");
	joy->ry_swap = of_property_read_bool(dev->of_node, "r_y_swap");

	/* Axis X/Y swap */
	joy->l_xy_swap = of_property_read_bool(dev->of_node, "l_xy_swap");
	joy->r_xy_swap = of_property_read_bool(dev->of_node, "r_xy_swap");

	/* Stick calibration (millivolts → microvolts) */
	if (of_property_read_u32(dev->of_node, "axis-min-value-mv",
				 &axis_min_mv))
		axis_min_mv = 250;
	if (of_property_read_u32(dev->of_node, "axis-max-value-mv",
				 &axis_max_mv))
		axis_max_mv = 1550;
	if (of_property_read_u32(dev->of_node, "axis-dead-zone-l", &dz_lo_mv))
		dz_lo_mv = 790;
	if (of_property_read_u32(dev->of_node, "axis-dead-zone-h", &dz_hi_mv))
		dz_hi_mv = 1010;

	joy->axis_min_uv = axis_min_mv * 1000;
	joy->axis_max_uv = axis_max_mv * 1000;
	joy->axis_dz_lo_uv = dz_lo_mv * 1000;
	joy->axis_dz_hi_uv = dz_hi_mv * 1000;

	/* Trigger calibration */
	if (of_property_read_u32(dev->of_node, "l2-r2-min-value-mv",
				 &trig_min_mv))
		trig_min_mv = 50;
	if (of_property_read_u32(dev->of_node, "l2-r2-max-value-mv",
				 &trig_max_mv))
		trig_max_mv = 800;

	joy->trig_min_uv = trig_min_mv * 1000;
	joy->trig_max_uv = trig_max_mv * 1000;

	/* --- Acquire IIO channels --- */

	for (i = 0; i < NUM_STICK_CHANS; i++) {
		joy->stick_chans[i] = devm_iio_channel_get(dev,
							    stick_chan_names[i]);
		if (IS_ERR(joy->stick_chans[i])) {
			ret = PTR_ERR(joy->stick_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					stick_chan_names[i], ret);
			return ret;
		}
	}

	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		joy->trig_chans[i] = devm_iio_channel_get(dev,
							   trig_chan_names[i]);
		if (IS_ERR(joy->trig_chans[i])) {
			ret = PTR_ERR(joy->trig_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					trig_chan_names[i], ret);
			return ret;
		}
	}

	for (i = 0; i < NUM_ADC_BTNS; i++) {
		joy->adc_btn_chans[i] = devm_iio_channel_get(dev,
							      adc_btn_chan_names[i]);
		if (IS_ERR(joy->adc_btn_chans[i])) {
			ret = PTR_ERR(joy->adc_btn_chans[i]);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get IIO channel %s: %d\n",
					adc_btn_chan_names[i], ret);
			return ret;
		}
	}

	/* --- Parse ADC button child nodes --- */

	ret = rg56pro_parse_adc_buttons(joy, dev);
	if (ret)
		return ret;

	/* --- Acquire GPIO buttons (IRQ-driven) --- */

	count = of_property_count_u32_elems(dev->of_node, "key-gpios-map");
	if (count != NUM_GPIO_BTNS) {
		dev_err(dev, "Expected %d GPIO key codes, got %d\n",
			NUM_GPIO_BTNS, count);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(dev->of_node, "key-gpios-map",
					 gpio_codes, NUM_GPIO_BTNS);
	if (ret) {
		dev_err(dev, "Failed to read key-gpios-map: %d\n", ret);
		return ret;
	}

	for (i = 0; i < NUM_GPIO_BTNS; i++) {
		struct rg56pro_gpio_btn *btn = &joy->gpio_btns[i];

		btn->joy = joy;
		btn->code = rg56pro_remap_code(gpio_codes[i]);
		INIT_DELAYED_WORK(&btn->work, rg56pro_gpio_work_func);

		btn->gpiod = devm_gpiod_get_index(dev, "key", i, GPIOD_IN);
		if (IS_ERR(btn->gpiod)) {
			ret = PTR_ERR(btn->gpiod);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get GPIO %d: %d\n",
					i, ret);
			return ret;
		}

		btn->irq = gpiod_to_irq(btn->gpiod);
		if (btn->irq < 0) {
			dev_err(dev, "Failed to get IRQ for GPIO %d: %d\n",
				i, btn->irq);
			return btn->irq;
		}

		ret = devm_request_any_context_irq(dev, btn->irq,
						   rg56pro_gpio_isr,
						   IRQF_TRIGGER_RISING |
						   IRQF_TRIGGER_FALLING,
						   DRIVER_NAME, btn);
		if (ret < 0) {
			dev_err(dev, "Failed to request IRQ %d for GPIO %d: %d\n",
				btn->irq, i, ret);
			return ret;
		}
	}

	ret = devm_add_action_or_reset(dev, rg56pro_cancel_gpio_work, joy);
	if (ret)
		return ret;

	/* --- Acquire rumble motor GPIOs (optional) --- */

	joy->moto_gpio = devm_gpiod_get_optional(dev, "moto", GPIOD_OUT_LOW);
	if (IS_ERR(joy->moto_gpio))
		return dev_err_probe(dev, PTR_ERR(joy->moto_gpio),
				     "Failed to get moto GPIO\n");

	joy->moto_r_gpio = devm_gpiod_get_optional(dev, "moto-r",
						    GPIOD_OUT_LOW);
	if (IS_ERR(joy->moto_r_gpio))
		return dev_err_probe(dev, PTR_ERR(joy->moto_r_gpio),
				     "Failed to get moto-r GPIO\n");

	/* --- Setup input device --- */

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	joy->input = input;
	input->name = DRIVER_NAME;
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0;
	input->id.product = 0;
	input->id.version = 0x0100;

	input_set_drvdata(input, joy);

	/* Register stick axes */
	for (i = 0; i < NUM_STICK_CHANS; i++) {
		input_set_abs_params(input, stick_abs_codes[i],
				     AXIS_MIN, AXIS_MAX, 16, 128);
	}

	/* Register trigger axes */
	for (i = 0; i < NUM_TRIG_CHANS; i++) {
		input_set_abs_params(input, trig_abs_codes[i],
				     TRIG_MIN, TRIG_MAX, 16, 128);
	}

	/* Register GPIO buttons */
	for (i = 0; i < NUM_GPIO_BTNS; i++)
		input_set_capability(input, EV_KEY, joy->gpio_btns[i].code);

	/* Register ADC buttons */
	for (i = 0; i < NUM_ADC_BTNS; i++)
		input_set_capability(input, EV_KEY, joy->adc_btn_codes[i]);

	/* Register dpad button capabilities (for axis-to-dpad mode) */
	input_set_capability(input, EV_KEY, BTN_DPAD_UP);
	input_set_capability(input, EV_KEY, BTN_DPAD_DOWN);
	input_set_capability(input, EV_KEY, BTN_DPAD_LEFT);
	input_set_capability(input, EV_KEY, BTN_DPAD_RIGHT);

	/* Setup force feedback if motors are present */
	if (joy->moto_gpio || joy->moto_r_gpio) {
		input_set_capability(input, EV_FF, FF_RUMBLE);
		ret = input_ff_create_memless(input, NULL,
					      rg56pro_play_effect);
		if (ret) {
			dev_err(dev, "Failed to create FF device: %d\n", ret);
			return ret;
		}
	}

	/* Setup polling (for analog axes and ADC buttons) */
	ret = input_setup_polling(input, rg56pro_poll);
	if (ret) {
		dev_err(dev, "Failed to setup polling: %d\n", ret);
		return ret;
	}

	input_set_poll_interval(input, poll_interval);
	input_set_min_poll_interval(input, 8);
	input_set_max_poll_interval(input, 100);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "Failed to register input device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "RG56 Pro joystick registered (poll %u ms, debounce %u ms, rumble %s)\n",
		 poll_interval, joy->debounce_ms,
		 (joy->moto_gpio || joy->moto_r_gpio) ? "yes" : "no");

	return 0;
}

static const struct of_device_id rg56pro_of_match[] = {
	{ .compatible = "play_joystick" },
	{ },
};
MODULE_DEVICE_TABLE(of, rg56pro_of_match);

static struct platform_driver rg56pro_driver = {
	.probe = rg56pro_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rg56pro_of_match,
		.pm = &rg56pro_pm_ops,
		.dev_groups = rg56pro_groups,
	},
};
module_platform_driver(rg56pro_driver);

MODULE_DESCRIPTION("RG56 Pro built-in gamepad driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dArkOS contributors");
