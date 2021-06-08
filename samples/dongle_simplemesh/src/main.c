#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
	int ret;
	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
	LOG_INF("Hello Simple Mesh");

	int loop = 0;
	while (1) {
		LOG_INF("sm> loop %d",loop);
		k_sleep(K_MSEC(3000));
		loop++;
	}
}
