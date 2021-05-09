
#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <battery.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>


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

void main(void)
{
	gpio_pin_init();
	debug_up();
	battery_init();
	debug_down();
	LOG_INF("Hello Power management");

	while (1) {
		debug_up();
		battery_start();
		debug_down();
		k_sleep(K_MSEC(1));//max acquisition time
		int32_t voltage = battery_get_mv();
		LOG_INF("battery> Voltage = %d mV",voltage);
		k_sleep(K_MSEC(1999));
	}
}
