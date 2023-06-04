#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <stdio.h>
#include <openthread/thread.h>
#include <openthread/link.h>
#include <openthread/instance.h>

//#include "app_button.h"
#include "app_ot.h"

otInstance* instance;
#if defined(CONFIG_OPENTHREAD_JOINER_PSKD)
#define OT_JOINER_PSKD CONFIG_OPENTHREAD_JOINER_PSKD
#else
#define OT_JOINER_PSKD ""
#endif

void click(void){
	printk("button - click - rebooting warm\n");
	sys_reboot(SYS_REBOOT_WARM);
}

void long_press(){
	printk("button - long press - OpenThread Factory Reset\n");
	otInstanceFactoryReset(instance);
	sys_reboot(SYS_REBOOT_COLD);
}

void get_eui64(char* eui64){
	otExtAddress aEui64;
	otLinkGetFactoryAssignedIeeeEui64(instance,&aEui64);
	for(int i=0;i<8;i++){
		sprintf(eui64+=2,"%02X",aEui64.m8[i]);
	}
	eui64[16] = '\0';
}

int app_ot_init(void){
	instance = otInstanceInitSingle();

	//app_button_init();
	//app_button_set_short_callback(click);
	//app_button_set_long_callback(long_press);
	//app_button_set_short_timeout(1000);
	//app_button_set_long_timeout(4000);

	char eui64[17];
	get_eui64(eui64);
	printk("Joiner eui64: %s ; pskd: %s\n",eui64,OT_JOINER_PSKD);
	printk("qrcode: v=1&&eui=%s&&cc=%s\n",eui64,OT_JOINER_PSKD);
	printk("https://dhrishi.github.io/connectedhomeip/qrcode.html?data=v%%3D1%%26%%26eui%%3D%s%%26%%26cc%%3D%s\n",eui64,OT_JOINER_PSKD);

	return 0;
}
