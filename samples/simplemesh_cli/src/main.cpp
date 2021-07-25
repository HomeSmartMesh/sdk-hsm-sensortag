#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <usb/usb_device.h>
#include <console/console.h>
#include <sys/reboot.h>

#include <json.hpp>
#include <simplemesh.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
json j;
extern bool critical_parse;

#if (CONFIG_SM_GPIO_DEBUG)
	#include <drivers/gpio.h>
	const struct device *gpio_dev;

	#if CONFIG_SM_GPIO_DEBUG
		//0.625 us per toggle
		#define PIN_SM_SET 		gpio_pin_set(gpio_dev, CONFIG_SM_PIN_APP, 1)
		#define PIN_SM_CLEAR 	gpio_pin_set(gpio_dev, CONFIG_SM_PIN_APP, 0)
	#else
		#define PIN_SM_SET
		#define PIN_SM_CLEAR
	#endif
#endif

void app_gpio_init()
{
	#if (CONFIG_SM_GPIO_DEBUG)
		gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		#if CONFIG_SM_GPIO_DEBUG
			gpio_pin_configure(gpio_dev, CONFIG_SM_PIN_APP, GPIO_OUTPUT_ACTIVE);
			sm_gpio_init(gpio_dev);
			PIN_SM_CLEAR;
			PIN_SM_SET;
			PIN_SM_CLEAR;
		#endif
	#endif
}


void main(void)
{
	app_gpio_init();
	#ifdef CONFIG_USB
		int ret;
		ret = usb_enable(NULL);
		if (ret != 0) {
			LOG_ERR("Failed to enable USB");
			return;
		}
		//TODO wait for USB to be connected
		k_sleep(K_SECONDS(5));
	#endif

	printf("main> simplemesh_cli start ------------- \n");
	sm_start();
	std::string uid = sm_get_uid();
	printf("main> UID (%s)\n",uid.c_str());

	console_getline_init();
	while (1) {
		printf(">");fflush(NULL);
		char *text = console_getline();
	    uint8_t size = strlen(text);
		std::string payload(text,size);
		if(payload.find("{") != std::string::npos){
			mesh_bcast_packet(sm::pid::json,(uint8_t*)text,size);
		}else{
			mesh_bcast_packet(sm::pid::text,(uint8_t*)text,size);
		}
		if(critical_parse){//non functional in simplemesh_cli due to blocking 
			sys_reboot(SYS_REBOOT_WARM);//param unused on ARM-M
		}
	}
}
