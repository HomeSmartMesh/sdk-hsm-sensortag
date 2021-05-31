
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

#include <pm/pm.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define SLEEP_TIME_MS   10000

#define DEBUG_PIN_APP 	 2
#define DEBUG_PIN_OS	29

#define DEBUG_PIN_2_SET 	(*(int * const)0x50000508) = 0x00000004
#define DEBUG_PIN_2_CLEAR 	(*(int *) 0x5000050C) = 0x00000004

#define DEBUG_PIN_29_SET 	(*(int * const)0x50000508) = 0x20000000
#define DEBUG_PIN_29_CLEAR 	(*(int *) 0x5000050C) = 0x20000000

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
	int ret = gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
	ret = gpio_pin_configure(gpio_dev, DEBUG_PIN_OS, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
}


#ifdef CONFIG_PM_POLICY_APP
static const struct pm_state_info pm_min_residency[] ={
	{PM_STATE_RUNTIME_IDLE		,1 ,1000 , 0},		//  1 ms
	{PM_STATE_STANDBY			,2 ,20000 , 0},	// 20 ms
	{PM_STATE_SUSPEND_TO_RAM	,3 ,100000 , 0},	// 100 ms
	{PM_STATE_SUSPEND_TO_DISK	,4 ,200000 , 0}	// 200 ms
};

struct pm_state_info pm_policy_next_state(int32_t ticks)
{
	int i;
	static struct pm_state_info current = {PM_STATE_ACTIVE,0, 1000};

	for (i = ARRAY_SIZE(pm_min_residency) - 1; i >= 0; i--) {
		if (!pm_constraint_get(pm_min_residency[i].state)) {
			continue;
		}

		if ((ticks == K_TICKS_FOREVER) || (ticks >= k_us_to_ticks_ceil32(pm_min_residency[i].min_residency_us))) 
		{
			if(current.state != pm_min_residency[i].state)
			{
				LOG_INF("Selected power state %d (ticks: %d, min_residency: %u)",pm_min_residency[i].state, ticks,pm_min_residency[i].min_residency_us);
				current = pm_min_residency[i];
			}
			return pm_min_residency[i];
		}
	}

	//LOG_DBG("No suitable power state found!");
	return (struct pm_state_info){PM_STATE_ACTIVE, 0, 0};
}
#endif


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

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	while (1) {
		LOG_INF("starting loop (%d)",count);
		battery_start();//loop pulse battery 100 us
		DEBUG_PIN_2_SET;
		k_sleep(K_MSEC(10));
		DEBUG_PIN_2_CLEAR;	//(6)
		int32_t voltage_mv = battery_get_mv();
		float voltage = voltage_mv;
		voltage /= 1000;

		#define MEASURE_SENSORS

		#ifdef MEASURE_SENSORS
		float light = veml6030_auto_measure(light_dev);
		//float light = 0;

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
		send_udp(message, size);

		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		DEBUG_PIN_2_SET;		//loop pulse 2 send_udp
		//k_busy_wait(3*1000*1000);//3 sec
		//lp_sleep_ms(1);
		//DEBUG_PIN_2_CLEAR;
		//lp_sleep_ms(1);
		//DEBUG_PIN_2_SET;		//loop pulse 2 send_udp
		//lp_sleep_ms(1000);
		k_sleep(K_MSEC(3000));
		DEBUG_PIN_2_CLEAR;

		count++;
	}
}
