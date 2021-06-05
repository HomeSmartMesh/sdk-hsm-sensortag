
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

LOG_MODULE_REGISTER(main, LOG_LEVEL_NONE);
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

	struct otInstance *openthread = openthread_get_default_instance();
	struct net_if * net = net_if_get_default();

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);
		
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d}",id0,id1,count);

		APP_SET;
		if(!net_if_is_up(net))
		{
			nrf_radio_power_set(NRF_RADIO,true);
			net_if_up(net);
			otThreadSetEnabled(openthread,true);
		}
		APP_CLEAR;
		send_udp(message, size);
		k_sleep(K_MSEC(100));
		APP_SET;		//loop pulse 2 send_udp
		otThreadSetEnabled(openthread,false);
		net_if_down(net);
		nrf_radio_power_set(NRF_RADIO,false);
		APP_CLEAR;

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		LOOP_CLEAR;
		k_sleep(K_MSEC(3000));

	}
}
