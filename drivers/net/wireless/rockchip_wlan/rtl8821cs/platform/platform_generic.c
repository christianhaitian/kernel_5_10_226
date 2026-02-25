#include <linux/errno.h>
#include <linux/etherdevice.h>

int rockchip_wifi_power(int on)
{
    /* Assume power is handled by regulator in DT */
    return 0;
}

int rockchip_wifi_set_carddetect(int val)
{
    /* Let MMC core handle card detection */
    return 0;
}

int rockchip_wifi_get_oob_irq(void)
{
    /* No out-of-band wake IRQ */
    return -1;
}

int rockchip_wifi_mac_addr(u8 *buf)
{
    /* Return -ENOENT so driver falls back to random MAC */
    return -ENOENT;
}