
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <net/net_if.h>
#include <stdio.h>
#include <drivers/gpio.h>

#include <net/openthread.h>
#include <openthread/thread.h>

#include "hal/nrf_radio.h"

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define SLEEP_TIME_MS   10000

#define DEBUG_PIN_APP 	 2
#define DEBUG_PIN_OS	29

#define APP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)	
#define APP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
#define LOOP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_OS, 1)
#define LOOP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_OS, 0)

const struct device *gpio_dev;
void gpio_pin_init()
{
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	int ret = gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
	ret = gpio_pin_configure(gpio_dev, DEBUG_PIN_OS, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
}

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
	gpio_pin_init();
	APP_CLEAR;
	LOOP_CLEAR;
	k_sleep(K_MSEC(10));

	APP_SET;
	k_sleep(K_MSEC(10));
	APP_CLEAR;

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
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);
		print_ot_info(openthread);
		
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d}",id0,id1,count);

		APP_SET;
		send_udp(message, size);
		APP_CLEAR;
		k_sleep(K_MSEC(100));

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		LOOP_CLEAR;
		k_sleep(K_MSEC(3000));

	}
}
