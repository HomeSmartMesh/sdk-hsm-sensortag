
#include <zephyr.h>
#include <logging/log.h>
#include <net/socket.h>
#include <net/net_if.h>
#include <stdio.h>
#include <drivers/gpio.h>

#include <net/openthread.h>
#include <openthread/thread.h>

#include "hal/nrf_radio.h"

#include "udp_client.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
#define SLEEP_TIME_MS   10000

void main(void)
{
	LOG_INF("Hello openthread dongle");
	k_sleep(K_MSEC(10));

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	int count = 0;
	//todo udp bind, listen and print
	while (1) {
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);
		
		char message[250];
		int size = sprintf(message,"thread_dongles/%04lX%04lX{\"alive\":%d}",id0,id1,count);
		printf("%s\n",message);
		LOG_INF("sleeping 1 sec");
		count++;
		LOOP_CLEAR;
		k_sleep(K_MSEC(3000));
	}
}
