//reference sample : zephyr\samples\boards\nrf\system_off 2.4.99

#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <power/power.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DEBUG_PIN_APP 	 2
#define DEBUG_PIN_OS	29

#define debug_up()		gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)
#define debug_down()	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
#define debug_os_down()	gpio_pin_set(gpio_dev, DEBUG_PIN_OS, 0)

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

static const struct pm_state_info pm_min_residency[] ={
	{PM_STATE_RUNTIME_IDLE,0, 1000},		//  1 ms
	{PM_STATE_STANDBY, 0, 20000},			// 20 ms
	{PM_STATE_SUSPEND_TO_RAM, 0, 100000},	// 100 ms
	{PM_STATE_SUSPEND_TO_DISK, 0, 200000}	// 200 ms
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

//pm_constraint_set(PM_STATE_SOFT_OFF);
//pm_constraint_release(PM_STATE_SOFT_OFF);
void main(void)
{
	gpio_pin_init();
	debug_down();
	debug_os_down();
	LOG_INF("Hello Power management");

	//int volatile * const p0_out = (int * const)0x50000504;
	//(*p0_out) = 0x20000000;//pio set

	test_sleep(1);
	test_sleep(1);

	//debug_up();
	//pm_system_suspend(100);
	//debug_down();

	//test_sleep(21);
	//test_sleep(101);
	test_sleep(250);
	//test_sleep(10000);//10 sec

	pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});//PM_STATE_SOFT_OFF

	while (1) {
	}
}
