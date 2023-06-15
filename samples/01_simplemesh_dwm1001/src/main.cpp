#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <simplemesh.h>
#include <json.hpp>
#include <string>

std::string uid;
std::string full_topic;
json j;

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

void rx_topic_json_handler(std::string &topic, json &data)
{
	printk("topic = %s ; json :\n",topic.c_str());
	printk("%s\n",data.dump(2).c_str());
}

void main(void)
{
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
	#endif

	uid = sm_get_uid();
	full_topic = sm_get_topic();
	printk("Hello Simple Mesh from DWM1001 UID (%s)\n",uid.c_str());
	printk("broadcasting on self topic (%s)\n",full_topic.c_str());

	sm_set_callback_rx_json(rx_topic_json_handler);
	sm_start();

	int loop = 0;
	while (1) {
		j["alive"] = loop;
		mesh_bcast_json(j);
		printk("broadcasted : %s\n",j.dump().c_str());
		k_sleep(K_SECONDS(5));
		loop++;
	}
}
