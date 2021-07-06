#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <console/console.h>

#include <json.hpp>
#include <simplemesh.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
json j;


void main(void)
{
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
		//TODO wait for USB to be connected
	#endif

	k_sleep(K_SECONDS(5));

	sm_start();
	std::string uid = sm_get_uid();
	printf("main>Simple Mesh UID [%s]\n",uid.c_str());

	std::string topic = sm_get_topic();
	j["sniffer"] = "started";
	mesh_bcast_json(j);
	printf("%s\n",j.dump().c_str());

	console_getline_init();
	while (1) {
		printf(">");fflush(NULL);
		char *text = console_getline();
		mesh_bcast_text(text);
	}
}
