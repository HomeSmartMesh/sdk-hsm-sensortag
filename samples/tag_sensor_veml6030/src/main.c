
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <sensor/ms8607.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


void main(void)
{
	LOG_INF("MS8607 Temperature Humidity pressure sensor application");
	const struct device *dev = get_ms8607_device();
	if (!dev) {
		LOG_ERR("sensor: device not found.");
		return;
	}

	while (1) {
		k_sleep(K_MSEC(5000));
	}
}
