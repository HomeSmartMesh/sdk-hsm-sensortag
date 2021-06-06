
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <net/net_if.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <sensor/veml6030.h>
#include <sensor/ms8607.h>
#include <battery.h>

#include <net/openthread.h>
#include <openthread/thread.h>

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_NONE);
//#define CONFIG_GPIO_DEBUG
#ifdef CONFIG_GPIO_DEBUG
	#include <drivers/gpio.h>
	#define DEBUG_PIN_APP 	 2
	#define DEBUG_PIN_LOOP	29

	#define APP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)
	#define APP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
	#define LOOP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 1)
	#define LOOP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 0)

	const struct device *gpio_dev;
	void gpio_pin_init()
	{
		gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
		gpio_pin_configure(gpio_dev, DEBUG_PIN_LOOP, GPIO_OUTPUT_ACTIVE);
	}
#else
	#define APP_SET
	#define APP_CLEAR
	#define LOOP_SET
	#define LOOP_CLEAR
	#define gpio_pin_init()
#endif

void openthread_set_mtd()
{
	otInstance *openthread = openthread_get_default_instance();
    bool rxOnWhenIdle = false;
    bool deviceType   = false;//Not FTD just MTD
    bool networkData  = false;//No full Network Data
	otLinkModeConfig linkMode = {rxOnWhenIdle, deviceType, networkData};
	otThreadSetLinkMode(openthread,linkMode);
}

void main(void)
{
	gpio_pin_init();
	APP_CLEAR;
	LOOP_CLEAR;
	k_sleep(K_MSEC(10));

	LOG_INF("Hello Sensors Broadcast");
	openthread_set_mtd();

	battery_init();
	const struct device *light_dev = device_get_binding(DT_LABEL(DT_INST(0, vishay_veml6030)));
	//getting the ms8607 is not needed due to the hardcoding of i2c adresses, multi instance is not possible
	//const struct device *env_dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));
	if(ms8607_is_connected()){
		LOG_INF("ms8607> connected");
	}else{
		LOG_ERR("ms8607> not connected");
	}

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);
		APP_SET;
		battery_start();
		k_sleep(K_MSEC(10));
		int32_t voltage_mv = battery_get_mv();
		float voltage = voltage_mv;
		voltage /= 1000;
		float light = veml6030_auto_measure(light_dev);
		float t, p, h;
		enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
		if(status != ms8607_status_ok){
			LOG_ERR("ms8607> status = %d",status);
		}
		APP_CLEAR;
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
									id0,id1,count, voltage, light, t, h, p);
		
		APP_SET;
		send_udp(message, size);
		APP_CLEAR;

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		LOOP_CLEAR;
		k_sleep(K_MSEC(3000));

	}
}
