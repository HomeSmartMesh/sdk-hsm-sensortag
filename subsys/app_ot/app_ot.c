#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <stdio.h>
#include <openthread/thread.h>
#include <openthread/link.h>
#include <openthread/instance.h>

#include <zephyr/net/openthread.h>

#include "app_button.h"
#include "app_ot.h"

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
	otInstance* instance = openthread_get_default_instance();
	printk("button - long press - OpenThread Factory Reset\n");
	otInstanceFactoryReset(instance);
	printk("Factory Reset complete - rebooting now\n");
	sys_reboot(SYS_REBOOT_COLD);
}

void quick_reboot_factoryreset(){
	long_press();
}

void get_eui64(char* eui64){
	otInstance* instance = openthread_get_default_instance();
	otExtAddress aEui64;
	otLinkGetFactoryAssignedIeeeEui64(instance,&aEui64);
	for(int i=0;i<8;i++){
		sprintf(eui64,"%02X",aEui64.m8[i]);
		eui64+=2;
	}
	eui64[16] = '\0';
}

void get_ext_address(char* ext_addr){
	otInstance* instance = openthread_get_default_instance();
	const otExtAddress* add = otLinkGetExtendedAddress(instance);
	for(int i=0;i<8;i++){
		sprintf(ext_addr,"%02X",add->m8[i]);
		ext_addr+=2;
	}
	ext_addr[16] = '\0';
}

int app_ot_init(void){

	printk("app_ot_init() start\n");
	otInstance* instance = openthread_get_default_instance();

	char ext_addr[17];
	get_ext_address(ext_addr);
	printk("Extended address: '%s'\n",ext_addr);

	otLinkSetMaxFrameRetriesDirect(instance,2);
	//app_button_init();
	//app_button_set_short_callback(click);
	//app_button_set_long_callback(long_press);
	//app_button_set_short_timeout(1000);
	//app_button_set_long_timeout(4000);

	char eui64[17];
	get_eui64(eui64);
	printk("Joiner eui64:'%s' ; pskd: '%s'\n",eui64,OT_JOINER_PSKD);
	printk("qrcode: v=1&&eui=%s&&cc=%s\n",eui64,OT_JOINER_PSKD);
	printk("https://dhrishi.github.io/connectedhomeip/qrcode.html?data=v%%3D1%%26%%26eui%%3D%s%%26%%26cc%%3D%s\n",eui64,OT_JOINER_PSKD);

	printk("app_ot_init() done\n");
	return 0;
}

otDeviceRole ot_app_role(){
	otInstance* instance = openthread_get_default_instance();	
	otDeviceRole role = otThreadGetDeviceRole(instance);
	printk("role: %s\n",otThreadDeviceRoleToString(role));

	otLinkModeConfig linkMode = otThreadGetLinkMode(instance);
	printk("LinkMode ; %s , %s, %s \n",
		linkMode.mRxOnWhenIdle?"RxIdle":"no RxIdle",
		linkMode.mDeviceType?"FTD":"not FTD",
		linkMode.mNetworkData?"NetData":"No NetData");

	uint8_t channel = otLinkGetChannel(instance);
	printk("channel : %u\n",channel);
	return role;

}
