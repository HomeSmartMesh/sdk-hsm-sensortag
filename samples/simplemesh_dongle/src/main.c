#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <console/console.h>

#include "simplemesh.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

char uid[20];

#define STACKSIZE 1025
#define PRIORITY 10
void console_thread();

K_SEM_DEFINE(sem_print, 1, 1);
K_THREAD_DEFINE(sm_console, STACKSIZE, console_thread, 
                NULL, NULL, NULL, PRIORITY, 0, 0);

void safe_print(const char * text)
{
	k_sem_take(&sem_print, K_FOREVER);
	printk("%s\n", text);
	k_sem_give(&sem_print);
}

void console_thread()
{
	console_getline_init();
	while (1) {
		printk(">");
		char *text = console_getline();
		mesh_bcast_text(text);
	}
}

void rx_handler(message_t* msg)
{
	if(msg->pid == Mesh_Pid_Text){
		msg->payload[msg->payload_length] = '\0';
		safe_print((char*)msg->payload);
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
	sm_start(rx_handler);
	sm_get_uid(uid);
	printk("Hello Simple Mesh Started from UID (%s)",uid);

	int loop = 0;
	while (1) {
		char message[50];
		sprintf(message,"sm/%s{\"alive\":%d}",uid,loop);
		mesh_bcast_text(message);
		k_sleep(K_SECONDS(20));
		loop++;
	}
}
