
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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define SLEEP_TIME_MS   10000

#define DEBUG_PIN_APP 	 2
#define DEBUG_PIN_OS	29

#define DEBUG_PIN_2_SET 	(*(int * const)0x50000508) = 0x00000004
#define DEBUG_PIN_2_CLEAR 	(*(int *) 0x5000050C) = 0x00000004

#define DEBUG_PIN_29_SET 	(*(int * const)0x50000508) = 0x20000000
#define DEBUG_PIN_29_CLEAR 	(*(int *) 0x5000050C) = 0x20000000

//SEGGER_SYSVIEW_MODULE IPModule = {
//	"M=test_mainMD, " \
//	"0 SendPacket IFace=%u NumBytes=%u, " \
//	"1 ReceivePacket Iface=%d NumBytes=%u", // sModule
//	2, // NumEvents
//	0,// EventOffset, Set by SEGGER_SYSVIEW_RegisterModule()
//	NULL,// pfSendModuleDesc, NULL: No additional module description
//	NULL,// pNext, Set by SEGGER_SYSVIEW_RegisterModule()
//	};
//
//static void _IPTraceConfig(void) {
//	SEGGER_SYSVIEW_RegisterModule(&IPModule);
//}

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

	DEBUG_PIN_2_SET;
	k_sleep(K_MSEC(1));
	DEBUG_PIN_2_CLEAR;
	LOG_INF("Hello Sensors Broadcast");

	battery_init();
	const struct device *light_dev = device_get_binding(DT_LABEL(DT_INST(0, vishay_veml6030)));
	//getting the ms8607 is not needed due to the hardcoding of i2c adresses, multi instance is not possible
	//const struct device *env_dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));
	if(ms8607_is_connected()){
		LOG_INF("ms8607> connected");
	}else{
		LOG_ERR("ms8607> not connected");
	}

	struct otInstance *openthread = openthread_get_default_instance();
	struct net_if * net = net_if_get_default();

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		DEBUG_PIN_29_SET;
		LOG_INF("starting loop (%d)",count);
		battery_start();//loop pulse battery 100 us
		DEBUG_PIN_2_SET;
		k_sleep(K_MSEC(10));
		DEBUG_PIN_2_CLEAR;	//(6)

		int32_t voltage_mv = battery_get_mv();
		float voltage = voltage_mv;
		voltage /= 1000;

		float light = veml6030_auto_measure(light_dev);

		float t, p, h;
		enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
		if(status != ms8607_status_ok){
			LOG_ERR("ms8607> status = %d",status);
		}

		DEBUG_PIN_2_SET;
		if(!net_if_is_up(net))
		{
			net_if_up(net);
			otThreadSetEnabled(openthread,true);
		}
		DEBUG_PIN_2_CLEAR;

		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
									id0,id1,count, voltage, light, t, h, p);
		send_udp(message, size);

		DEBUG_PIN_2_SET;		//loop pulse 2 send_udp
		otThreadSetEnabled(openthread,false);
		net_if_down(net);
		DEBUG_PIN_2_CLEAR;

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		DEBUG_PIN_29_CLEAR;
		k_sleep(K_MSEC(3000));

	}
}
