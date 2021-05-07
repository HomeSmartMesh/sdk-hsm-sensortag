
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <sensor/ms8607.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *get_ms8607_device(void)
{
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));
	//const struct device *dev = device_get_binding("VEML6030");

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	return dev;
}

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
