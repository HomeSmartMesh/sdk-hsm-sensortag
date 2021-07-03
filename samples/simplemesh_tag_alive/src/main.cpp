#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>

#include <json.hpp>
#include <simplemesh.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
json j;

void main(void)
{
	LOG_INF("Hello Simple Mesh from Sensor Tag");

	sm_start();

	int loop = 0;
	while (1) {
		j["alive"] = loop;
		mesh_bcast_json(j);
		printk("broadcasted : %s\n",j.dump().c_str());
		k_sleep(K_SECONDS(4));
		loop++;
	}
}
