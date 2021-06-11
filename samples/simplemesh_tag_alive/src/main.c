#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

#include "simplemesh.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

void main(void)
{
	LOG_INF("Hello Simple Mesh from Sensor Tag");

	sm_start();

	int loop = 0;
	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	while (1) {
		char message[50];
		sprintf(message,"simplemesh/%04lX%04lX{\"alive\":%d}",id0,id1,loop);
		mesh_bcast_text(message);
		printk("%s\n",message);
		k_sleep(K_SECONDS(4));
		loop++;
	}
}
