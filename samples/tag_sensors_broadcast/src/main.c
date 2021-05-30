
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <sensor/veml6030.h>
#include <sensor/ms8607.h>
#include <battery.h>

#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>
#include <drivers/timer/system_timer.h>

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define SLEEP_TIME_MS   10000

#define DEBUG_PIN 29
#define debug_up()		gpio_pin_set(gpio_dev, DEBUG_PIN, 1)
#define debug_down()	gpio_pin_set(gpio_dev, DEBUG_PIN, 0)

void lp_sleep_ms(int sleep_ms)
{
	sys_clock_set_timeout(k_ms_to_ticks_ceil32(sleep_ms),false);//sleep @2.9 uA
	//----------sleep prepare--------------
	nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTOP);
	nrf_power_task_trigger(NRF_POWER,NRF_POWER_TASK_LOWPWR);
	__WFE();
	__SEV();// Clear the internal event register.
	__WFE();
	//----------sleep wakeup--------------
    nrf_clock_event_clear(NRF_CLOCK,NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTART);
	while(!nrf_clock_hf_is_running(NRF_CLOCK,NRF_CLOCK_HFCLK_HIGH_ACCURACY));
}

const struct device *gpio_dev;
void gpio_pin_init()
{
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	int ret = gpio_pin_configure(gpio_dev, DEBUG_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
}


void main(void)
{
	gpio_pin_init();

	debug_up();
	k_sleep(K_MSEC(1));
	debug_down();
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

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOG_INF("starting loop (%d)",count);

		debug_up();		//(1)
		battery_start();
		k_sleep(K_USEC(100));//max acquisition time
		int32_t voltage_mv = battery_get_mv();
		float voltage = voltage_mv;
		voltage /= 1000;

		debug_down();	//(2)
		float light = veml6030_auto_measure(light_dev);

		debug_up();		//(3)
		float t, p, h;
		enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
		if(status != ms8607_status_ok){
			LOG_ERR("ms8607> status = %d",status);
		}

		debug_down();	//(4)
		char message[250];
		int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
									id0,id1,count, voltage, light, t, h, p);

		debug_up();		//(5)
		send_udp(message, size);

		debug_down();	//(6)
		printf("%s\n",message);
		LOG_INF("sleeping 10 sec");
		lp_sleep_ms(SLEEP_TIME_MS);

		count++;
	}
}
