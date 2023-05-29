
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include <battery.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
	LOG_INF("Battery Voltage Measure ADC VDD channel");

	battery_init();

	while (1) {
		battery_start();
		k_sleep(K_MSEC(1));//max acquisition time
		int32_t voltage = battery_get_mv();
		LOG_INF("battery> Voltage = %d mV",voltage);
		k_sleep(K_MSEC(4999));
	}
}
