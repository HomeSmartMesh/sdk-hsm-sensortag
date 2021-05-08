
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/adc.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define SENSOR_LOG_LEVEL_OFF      0
#define SENSOR_LOG_LEVEL_ERROR    1
#define SENSOR_LOG_LEVEL_WARNING  2
#define SENSOR_LOG_LEVEL_INFO     3
#define SENSOR_LOG_LEVEL_DEBUG    4

void battery_init()
{
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));

	LOG_INF("battery_init()");
}

void battery_start()
{
	LOG_INF("battery_start()");
}

uint16_t battery_get_mv()
{
	LOG_INF("battery_get_mv()");
	return 0;
}
