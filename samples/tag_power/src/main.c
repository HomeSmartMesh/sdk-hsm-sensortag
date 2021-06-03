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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define WITH_GPIO

#ifdef WITH_GPIO
	#define DEBUG_PIN_APP 	 2
	#define DEBUG_PIN_OS	29

	#define DEBUG_PIN_29_SET 	(*(int * const)0x50000508) = 0x20000000
	#define DEBUG_PIN_29_CLEAR 	(*(int *) 0x5000050C) = 0x20000000

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
#else
	#define debug_up()		
	#define debug_down()	
	#define debug_os_down()	
	#define gpio_pin_init()
#endif

void main(void)
{
	gpio_pin_init();
	debug_down();
	debug_os_down();
	LOG_INF("Hello Power management");

	debug_up();
	k_sleep(K_MSEC(1));
	debug_down();
	k_sleep(K_MSEC(1));
	debug_up();

	k_sleep(K_MSEC(250));
	debug_down();

	while (1) {
		debug_up();
		k_sleep(K_MSEC(5000));//@3 uA
		debug_down();
		k_busy_wait(2000*1000);//@3 mA
	}
}

