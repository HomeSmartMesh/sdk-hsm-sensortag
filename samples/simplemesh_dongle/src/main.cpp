#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <console/console.h>

#include <json.hpp>
#include <simplemesh.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
json j;

#define STACKSIZE 1025
#define PRIORITY 10
void console_thread();

K_SEM_DEFINE(sem_print, 1, 1);
K_THREAD_DEFINE(sm_console, STACKSIZE, console_thread, 
                NULL, NULL, NULL, PRIORITY, 0, 0);

void console_thread()
{
	console_getline_init();
	while (1) {
		printf(">");
		char *text = console_getline();
		mesh_bcast_text(text);
	}
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
		//TODO wait for USB to be connected
	#endif

	k_sleep(K_SECONDS(5));
	sm_start();
	std::string uid = sm_get_uid();
	printf("main>Hello Simple Mesh Started from UID [%s]",uid.c_str());

	std::string topic = sm_get_topic();
	int loop = 0;
	while (1) {
		j["alive"] = loop;
		mesh_bcast_json(j);
		printf("%s:%s\n",topic.c_str(),j.dump().c_str());
		k_sleep(K_SECONDS(60));
		loop++;
	}
}
