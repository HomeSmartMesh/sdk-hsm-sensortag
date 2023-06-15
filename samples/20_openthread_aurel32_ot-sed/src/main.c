/*
 * Copyright (c) 2020 Aurelien Jarno
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <net/socket.h>
#include <net/openthread.h>
#include <openthread/platform/radio.h>

#define SERVER_ADDR		"ff02::1"
#define SERVER_PORT		4242

#ifdef CONFIG_GPIO
	#include <drivers/gpio.h>
	#define DEBUG_PIN_APP 	 2
	#define DEBUG_PIN_LOOP	29

	#define APP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)
	#define APP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
	#define LOOP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 1)
	#define LOOP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 0)

	const struct device *gpio_dev;
	void gpio_pin_init()
	{
		gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
		gpio_pin_configure(gpio_dev, DEBUG_PIN_LOOP, GPIO_OUTPUT_ACTIVE);
	}
#else
	#define APP_SET
	#define APP_CLEAR
	#define LOOP_SET
	#define LOOP_CLEAR
	#define gpio_pin_init()
#endif

const struct device *temp_sensor = DEVICE_DT_GET(DT_INST(0, nordic_nrf_temp));

void main(void)
{
	struct sockaddr_in6 addr6;
	struct sockaddr *addr;
	int sock;
	int ret;

	gpio_pin_init();
	APP_CLEAR;
	LOOP_CLEAR;

	k_sleep(K_SECONDS(2));

	int count_main = 0;
	while (1) {
		//init pulse
		APP_SET;
		count_main++;
		addr6.sin6_family = AF_INET6;
		addr6.sin6_port = htons(SERVER_PORT);
		inet_pton(AF_INET6, SERVER_ADDR, &addr6.sin6_addr);
		addr = (struct sockaddr *)&addr6;
		APP_CLEAR;

		sock = socket(addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
		if (sock < 0) {
			continue;
		}
		//connect pulse
		APP_SET;
		ret = connect(sock, addr, sizeof(addr6));
		if (ret < 0) {
			continue;
		}
		APP_CLEAR;
		int count_sub = 0;
		while (1) {
			LOOP_SET;
			count_sub++;
			char payload[64];
			struct sensor_value val;
			sensor_sample_fetch(temp_sensor);
			sensor_channel_get(temp_sensor, SENSOR_CHAN_DIE_TEMP, &val);
			int i = val.val1;
			int f = val.val2 / 10000;
			snprintk(payload, sizeof(payload), "{\"main\":%d,\"sub\":%d,\"die_temp\":%d.%02d}",
												count_main,count_sub, i, f);
			//send pulse
			APP_SET;
			ret = send(sock, payload, strlen(payload), 0);
			if (ret < 0) {
				break;
			}
			APP_CLEAR;
			LOOP_CLEAR;
			k_sleep(K_SECONDS(3));
		}
		
		close(sock);
		k_sleep(K_SECONDS(1));
	}
}
