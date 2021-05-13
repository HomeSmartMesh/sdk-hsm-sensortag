
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <stdio.h>

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SLEEP_TIME_MS   10000

void main(void)
{
	LOG_INF("Hello from Sensor Tag");

	int count = 0;
	while (1) {
		LOG_INF("sleeping 10 sec");
		k_msleep(SLEEP_TIME_MS);
		char message[20];
		int size = sprintf(message,"Tag loop (%d)\n",count);
		send_udp(message, size);
		LOG_INF("end of loop: %d",count++);
	}
}
