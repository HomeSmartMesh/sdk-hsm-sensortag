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


static int disable_ds_1(const struct device *dev)
{
	ARG_UNUSED(dev);
	pm_constraint_set(PM_STATE_SOFT_OFF);
	return 0;
}

SYS_INIT(disable_ds_1, PRE_KERNEL_2, 0);

void main(void)
{
	gpio_pin_init();
	debug_up();
	LOG_INF("Hello Power management");
	debug_down();
	k_sleep(K_MSEC(100));

	while (1) {
		LOG_INF("start of while");
		debug_up();
		k_sleep(K_MSEC(1));
		debug_down();
		LOG_INF("loop");
		k_sleep(K_MSEC(100));

		pm_power_state_force((struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
		LOG_INF("after pm");
		k_sleep(K_MSEC(1));
		LOG_INF("after pm k_sleep");
		k_sleep(K_MSEC(999));
	}
}
