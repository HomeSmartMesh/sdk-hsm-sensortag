
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <sensor/veml6030.h>
#include <sensor/ms8607.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

#include "battery.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

char device_id[20];

const struct device *light_dev = DEVICE_DT_GET_ONE(vishay_veml6030);

void report_sensors(int count,bool send){
	battery_start();
	k_sleep(K_MSEC(10));
	int32_t voltage_mv = battery_get_mv();
	float voltage = voltage_mv;
	voltage /= 1000;
	float light = veml6030_auto_measure(light_dev);
	float t, p, h;
	enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t, &p, &h);
	if (status == ms8607_status_ok) {
        LOG_DBG("ms8607> t=%.2f h=.%2f p=%.2f", t, p, h);
    } else {
		LOG_ERR("ms8607> status = %d", status);
	}

	char message[250];
	int size = sprintf(message,"thread_tags/%s{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
								device_id, count, voltage, light, t, h, p);
	LOG_INF("%s",message);
}

void main(void)
{
	battery_init();
	if (ms8607_is_connected()) {
		LOG_INF("ms8607> connected");
	} else {
		LOG_ERR("ms8607> not connected");
	}

    otInstance* ot = openthread_get_default_instance();

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	sprintf(device_id, "%04lX%04lX", id0, id1);
	LOG_INF("Device ID: %s", device_id);
}
