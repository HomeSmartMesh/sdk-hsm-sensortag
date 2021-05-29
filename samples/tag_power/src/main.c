//reference sample : zephyr\samples\boards\nrf\system_off 2.4.99

#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>
#include <drivers/timer/system_timer.h>

#include <pm/pm.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DEBUG_PIN_APP 	 2
#define DEBUG_PIN_OS	29

#define DEBUG_PIN_29_SET 	(*(int * const)0x50000508) = 0x20000000
#define DEBUG_PIN_29_CLEAR 	(*(int *) 0x5000050C) = 0x20000000

#define debug_up()		gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)
#define debug_down()	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
#define debug_os_down()	gpio_pin_set(gpio_dev, DEBUG_PIN_OS, 0)

void app_sleep_prepare()
{
	nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTOP);

	nrf_power_task_trigger(NRF_POWER,NRF_POWER_TASK_LOWPWR);

	DEBUG_PIN_29_SET;

	__WFE();
	// Clear the internal event register.
	__SEV();
	__WFE();
}

void app_sleep_wakeup()
{
    nrf_clock_event_clear(NRF_CLOCK,NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK,NRF_CLOCK_TASK_HFCLKSTART);
	while(!nrf_clock_hf_is_running(NRF_CLOCK,NRF_CLOCK_HFCLK_HIGH_ACCURACY));
	DEBUG_PIN_29_CLEAR;
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

void test_sleep(uint32_t sleep_ms)
{
	LOG_INF("going to sleep %u ms",sleep_ms);
	debug_up();
	k_sleep(K_MSEC(sleep_ms));
	debug_down();
}

#ifdef CONFIG_PM
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
	debug_down();
	debug_os_down();
	LOG_INF("Hello Power management");

	test_sleep(1);
	test_sleep(1);

	//in this next 250 ms sleep, the os idle task keeps ticking through RTC @ 297 us period
	//therefore using 'k_sleep' it's not possible to reach a long average low power mode
	test_sleep(250);

	#if STOP_TEST_OFF
		pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});//PM_STATE_SOFT_OFF
	#endif

	while (1) {
		//first cycle @268 us, then second @5 sec (washing out the last os tick left)
		debug_up();
		sys_clock_set_timeout(k_ms_to_ticks_ceil32(5000),false);//5 sec @2.9 uA
		app_sleep_prepare();
		app_sleep_wakeup();
		debug_down();
		#if LOOP_WITH_SLEEP
			test_sleep(100);//if this line is enabled it prevents the os from stoping the idle task ticks
		#endif
		k_busy_wait(1000*1000);//1 sec @3 mA
	}
}
