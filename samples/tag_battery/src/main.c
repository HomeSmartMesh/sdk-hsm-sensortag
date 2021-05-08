
#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>

#include "battery.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
	LOG_INF("Test App for Battery Voltage Meadure through ADC");

	battery_init();

	while (1) {
		battery_start();
		k_sleep(K_MSEC(100));
		uint16_t voltage = battery_get_mv();
		LOG_INF("battery> Voltage = %u mV",voltage);
		k_sleep(K_MSEC(4900));
	}
}
