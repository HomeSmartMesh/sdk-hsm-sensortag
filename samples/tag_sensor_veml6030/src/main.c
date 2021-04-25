
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <sensor/veml6030.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *get_veml6030_device(void)
{
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, vishay_veml6030)));
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

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

void read_device(const struct device *dev)
{
	struct sensor_value val;

	if (veml6030_power_on(dev) != 0) {
		LOG_ERR("sensor: power on fail.");
		return;
	}

	k_sleep(K_MSEC(2000));//wait the integration time

	if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_LIGHT) != 0) {
		LOG_ERR("sensor: sample fetch fail.");
		return;
	}
	if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &val) != 0) {
		LOG_ERR("sensor: channel get fail.");
		return;
	}

	LOG_INF("sensor: ambient light reading: %d.%06d lux", val.val1,val.val2);

	if (veml6030_power_off(dev) != 0) {
		LOG_ERR("sensor: power off fail");
		return;
	}

}

void read_auto(const struct device *dev)
{
	struct sensor_value val;
	veml6030_auto_measure(dev);//unused return
	if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &val) != 0) {
		LOG_ERR("sensor: channel get fail.");
		return;
	}
	LOG_INF("=====> sensor: ambient light reading: %d.%06d lux", val.val1,val.val2);
}

void main(void)
{
	LOG_INF("VEML6030 light sensor application");
	const struct device *dev = get_veml6030_device();
	if (!dev) {
		LOG_ERR("sensor: device not found.");
		return;
	}

	while (1) {
		read_auto(dev);
		k_sleep(K_MSEC(5000));
	}
}
