
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

#include "hal/nrf_radio.h"

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define SLEEP_TIME_MS   10000

void print_ot_info(otInstance *instance)
{
	otDeviceRole role = otThreadGetDeviceRole(instance);
        switch (role)
        {
        case OT_DEVICE_ROLE_DISABLED:
            LOG_INF("state : disabled");
            break;
        case OT_DEVICE_ROLE_DETACHED:
            LOG_INF("state : detached");
            break;
        case OT_DEVICE_ROLE_CHILD:
            LOG_INF("state : child");
            break;
        case OT_DEVICE_ROLE_ROUTER:
            LOG_INF("state : router");
            break;
        case OT_DEVICE_ROLE_LEADER:
            LOG_INF("state : leader");
            break;
        default:
            LOG_INF("state : invalid");
            break;
        }

	otLinkModeConfig linkMode = otThreadGetLinkMode(instance);
	LOG_INF(" * Rx On WhenIdle: %d , * Full Thread Device %d, * Full Network Data %d",
						linkMode.mRxOnWhenIdle,linkMode.mDeviceType,linkMode.mNetworkData);
}

void main(void)
{
	k_sleep(K_MSEC(3000));

	LOG_INF("Hello openthread udp");
	k_sleep(K_MSEC(10));

	otInstance *openthread = openthread_get_default_instance();
    bool rxOnWhenIdle = false;
    bool deviceType = false;//Not FTD just MTD
    bool networkData = false;//No full Network Data
	otLinkModeConfig linkMode = {rxOnWhenIdle, deviceType, networkData};
	otThreadSetLinkMode(openthread,linkMode);

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOG_INF("starting loop (%d)",count);
		print_ot_info(openthread);
		
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d}",id0,id1,count);

		send_udp(message, size);
		k_sleep(K_MSEC(100));

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		k_sleep(K_MSEC(3000));

	}
}
