
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <sensor/veml6030.h>
#include <sensor/ms8607.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

//#include "app_battery.h"
#include "udp_client.h"
#include "app_ot.h"


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
//#define CONFIG_GPIO_DEBUG
#ifdef CONFIG_GPIO_DEBUG
	#include <zephyr/drivers/gpio.h>
	#define DEBUG_PIN_APP 	 2
	#define DEBUG_PIN_LOOP	29

	#define APP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 1)
	#define APP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_APP, 0)
	#define LOOP_SET 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 1)
	#define LOOP_CLEAR 	gpio_pin_set(gpio_dev, DEBUG_PIN_LOOP, 0)

	const struct device *gpio_dev;
	void gpio_pin_init()
	{
		gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
		gpio_pin_configure(gpio_dev, DEBUG_PIN_APP, GPIO_OUTPUT_ACTIVE);
		gpio_pin_configure(gpio_dev, DEBUG_PIN_LOOP, GPIO_OUTPUT_ACTIVE);
	}
#else
	#define APP_SET
	#define APP_CLEAR
	#define LOOP_SET
	#define LOOP_CLEAR
	#define gpio_pin_init()
#endif

#define RUN_CYCLE_SEC 2U
#define SLEEP_DISABLED_SEC 300U
#define SLEEP_CYCLE_SEC 30U

//reboot every ~ 30 min
#define REBOOT_CYCLES_COUNT 60

#define WDT_MAX_WINDOW_MS  60000U
#define WDT_MIN_WINDOW_MS  0U
int wdt_channel_id;
char uid_text[20];

const struct device *light_dev = DEVICE_DT_GET_ONE(vishay_veml6030);

void start_watchdog(const struct device *const wdt){
	struct wdt_timeout_cfg wdt_config = {
		.flags = WDT_FLAG_RESET_SOC,
		.window.min = WDT_MIN_WINDOW_MS,
		.window.max = WDT_MAX_WINDOW_MS,
	};

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		printk("Watchdog install error\n");
		return;
	}
	wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

void report_sensors(int count,bool send){
	APP_SET;
	//battery_start();
	k_sleep(K_MSEC(10));
	//int32_t voltage_mv = battery_get_mv();
	//float voltage = voltage_mv;
	//voltage /= 1000;
	float light = veml6030_auto_measure(light_dev);
	float t, p, h;
	enum ms8607_status status = ms8607_read_temperature_pressure_humidity(&t,&p,&h);
	if(status != ms8607_status_ok){
		LOG_ERR("ms8607> status = %d",status);
	}
	APP_CLEAR;

	char message[250];
	//int size = sprintf(message,"thread_tags/%04lX%04lX{\"alive\":%d,\"voltage\":%.3f,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
	//							id0,id1,count, voltage, light, t, h, p);
	int size = sprintf(message,"thread_tags/%s{\"alive\":%d,\"light\":%0.3f,\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f}",
								uid_text,count, light, t, h, p);
	if(send){
		APP_SET;
		send_udp(message, size);
		APP_CLEAR;
	}
	LOG_INF("%s",message);
}

void main(void)
{
	gpio_pin_init();
	APP_CLEAR;
	LOOP_CLEAR;
	k_sleep(K_MSEC(10));

	LOG_INF("Hello Sensors Broadcast");

	//battery_init();
	//const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
	//start_watchdog(wdt);
	app_ot_init();//logs joiner info and initializes reset buttons
	//getting the ms8607 is not needed due to the hardcoding of i2c adresses, multi instance is not possible
	//const struct device *env_dev = device_get_binding(DT_LABEL(DT_INST(0, teconnectivity_ms8607)));
	if(ms8607_is_connected()){
		LOG_INF("ms8607> connected");
	}else{
		LOG_ERR("ms8607> not connected");
	}

	long unsigned int id0 = NRF_FICR->DEVICEID[0];//just for type casting and readable printing
	long unsigned int id1 = NRF_FICR->DEVICEID[1];
	sprintf(uid_text,"%04lX%04lX",id0,id1);
	int count = 0;
	while (1) {
		//wdt_feed(wdt, wdt_channel_id);
		LOOP_SET;
		LOG_INF("starting loop (%d)",count);

		otDeviceRole role = ot_app_role();
		//bool send = (role >= OT_DEVICE_ROLE_CHILD);
		report_sensors(count,true);

		if(role == OT_DEVICE_ROLE_DISABLED){
			LOG_INF("role:disabled ; sleeping %d sec cout = %d",SLEEP_DISABLED_SEC,count);
			k_sleep(K_MSEC(SLEEP_DISABLED_SEC*1000));
			role = ot_app_role();
			if(role == OT_DEVICE_ROLE_DISABLED){
				sys_reboot(SYS_REBOOT_WARM);
			}
		}else{
			LOG_INF("sleeping %d sec count = %d",SLEEP_CYCLE_SEC,count);
			k_sleep(K_MSEC(SLEEP_CYCLE_SEC*1000));
		}
		count++;
		LOOP_CLEAR;
	}
}
