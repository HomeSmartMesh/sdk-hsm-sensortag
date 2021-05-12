//reference sample : zephyr\samples\boards\nrf\system_off 2.4.99

#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <power/power.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DEBUG_PIN 29
#define debug_up()		gpio_pin_set(gpio_dev, DEBUG_PIN, 1)
#define debug_down()	gpio_pin_set(gpio_dev, DEBUG_PIN, 0)

const struct device *gpio_dev;
void gpio_pin_init()
{
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	int ret = gpio_pin_configure(gpio_dev, DEBUG_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio_pin_configure() failed");
	}
}

void test_state(enum pm_state state)
{
	LOG_INF("Power state %d",state);
	pm_power_state_force((struct pm_state_info){state, 0, 0});//PM_STATE_SOFT_OFF
	debug_up();
	k_sleep(K_MSEC(10));
	debug_down();
}

//pm_constraint_set(PM_STATE_SOFT_OFF);
//pm_constraint_release(PM_STATE_SOFT_OFF);
void main(void)
{
	gpio_pin_init();

	debug_up();
	k_sleep(K_MSEC(1));
	debug_down();
	LOG_INF("Hello Power management");
	k_sleep(K_MSEC(1000));

	test_state(PM_STATE_ACTIVE);			//104 us
	test_state(PM_STATE_RUNTIME_IDLE);		//110 us
	test_state(PM_STATE_SUSPEND_TO_IDLE);	//110 us
	test_state(PM_STATE_STANDBY);			//110 us
	test_state(PM_STATE_SUSPEND_TO_RAM);	//110 us
	test_state(PM_STATE_SUSPEND_TO_DISK);	//110 us
	test_state(PM_STATE_SOFT_OFF);

	k_sleep(K_MSEC(1000));

	while (1) {
		debug_up();
		k_sleep(K_MSEC(1));
		debug_down();
		LOG_INF("loop");
		k_sleep(K_MSEC(999));
	}
}
