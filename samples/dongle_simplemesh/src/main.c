#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

#include "simplemesh.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

void main(void)
{
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
	#endif
	LOG_INF("Hello Simple Mesh");

	#ifdef CONFIG_ESB
		sm_start_rx();//still manual change from TX, RX
	#endif

	int loop = 0;
	while (1) {
		LOG_INF("sm> loop %d",loop);
		k_sleep(K_MSEC(3000));
		loop++;
	}
}
