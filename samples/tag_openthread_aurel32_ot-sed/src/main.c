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

const struct device *temp_sensor = DEVICE_DT_GET(DT_INST(0, nordic_nrf_temp));

void main(void)
{
	struct sockaddr_in6 addr6;
	struct sockaddr *addr;
	int sock;
	int ret;

        otInstance *otdev = openthread_get_default_instance();
        otPlatRadioSetTransmitPower(otdev, 8);

	k_sleep(K_SECONDS(2));

	while (1) {
		addr6.sin6_family = AF_INET6;
		addr6.sin6_port = htons(SERVER_PORT);
		inet_pton(AF_INET6, SERVER_ADDR, &addr6.sin6_addr);
		addr = (struct sockaddr *)&addr6;

		sock = socket(addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
		if (sock < 0) {
			continue;
		}
		ret = connect(sock, addr, sizeof(addr6));
		if (ret < 0) {
			continue;
		}
		while (1) {
			char payload[30];
			struct sensor_value val;
			sensor_sample_fetch(temp_sensor);
			sensor_channel_get(temp_sensor, SENSOR_CHAN_DIE_TEMP, &val);
			int i = val.val1;
			int f = val.val2 / 10000;

			snprintk(payload, sizeof(payload), "%d.%02d\n", i, f);

			ret = send(sock, payload, strlen(payload), 0);
			if (ret < 0) {
				break;
			}
			k_sleep(K_SECONDS(15));
		}
		
		close(sock);
		k_sleep(K_SECONDS(2));
	}
}
