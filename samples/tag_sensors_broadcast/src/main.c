
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

	LOG_INF("Hello Sensors Broadcast");
	k_sleep(K_MSEC(10));

	#ifdef USE_SENSORS
	battery_init();
	const struct device *light_dev = device_get_binding(DT_LABEL(DT_INST(0, vishay_veml6030)));
	//getting the ms8607 is not needed due to the hardcoding of i2c adresses, multi instance is not possible
	//const struct device *env_dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));
	if(ms8607_is_connected()){
		LOG_INF("ms8607> connected");
	}else{
		LOG_ERR("ms8607> not connected");
	}
	#endif

	struct otInstance *openthread = openthread_get_default_instance();
	struct net_if * net = net_if_get_default();

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);
		#ifdef USE_SENSORS
		battery_start();//loop pulse battery 100 us
		#endif

		#ifdef USE_SENSORS
		int32_t voltage_mv = battery_get_mv();
		float voltage = voltage_mv;
		voltage /= 1000;

		float light = veml6030_auto_measure(light_dev);

		float t, p, h;
		enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
		if(status != ms8607_status_ok){
			LOG_ERR("ms8607> status = %d",status);
		}
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
									id0,id1,count, voltage, light, t, h, p);
		#else
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d}",id0,id1,count);
		#endif

		APP_SET;
		if(!net_if_is_up(net))
		{
			net_if_up(net);
			otThreadSetEnabled(openthread,true);
		}
		APP_CLEAR;


		send_udp(message, size);

		APP_SET;		//loop pulse 2 send_udp
		otThreadSetEnabled(openthread,false);
		net_if_down(net);
		APP_CLEAR;

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		LOOP_CLEAR;
		k_sleep(K_MSEC(3000));

	}
}
