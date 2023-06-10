
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>
#include "app_button.h"

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,{0});
static struct gpio_callback button_cb_data;
static struct k_work_delayable long_press_work;

#define LONG_PRESS_TIMEOUT K_SECONDS(4)
#define SHORT_PRESS_TIMEOUT_MSEC 1000
#define LONG_PRESS_TIMEOUT_MSEC 4000

static button_callback_handler_t button_short_press = NULL;
static button_callback_handler_t button_long_press = NULL;
int64_t short_press_timeout_ms = SHORT_PRESS_TIMEOUT_MSEC;
int64_t long_press_timeout_ms = LONG_PRESS_TIMEOUT_MSEC;


static bool pressed;
static bool booting;
static bool triggered;
int64_t press_time;

static void button_hold_timeout(struct k_work *work)
{
	triggered = true;//used to cancel
	printk("Button hold timeout\n");
	if(button_long_press != NULL){
		button_long_press();
	}
}

static bool button_is_pressed(void)
{
	return gpio_pin_get_dt(&button) > 0;
}

void button_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	//this debounces cases of very fast signal jitter that loses the release state
	//while executing the press call
	if (button_is_pressed() == pressed) {
		return;
	}
	pressed = !pressed;
	if (pressed) {
		press_time = k_uptime_get();
		triggered = false;
		booting = false;
		k_work_reschedule(&long_press_work, LONG_PRESS_TIMEOUT);
		printk("Button pressed at %" PRIi64 " \n", press_time);
	}
	else{//released
		if(booting){
			printk("Button released after boot");
			booting = false;
		}
		else{
			if(!triggered){//that's because it was released too early - cancel hold timeout
				int64_t time = k_uptime_delta(&press_time);
				if(time < short_press_timeout_ms){
					if(button_short_press != NULL){
						button_short_press();
					}
				}else{
					printk("Button released after %" PRIi64 " ms - not short, not long\n", time);
				}
				k_work_cancel_delayable(&long_press_work);
			}//else triggered, nothing to do, release after trigger
		}
	}
}

int app_button_init(void){
	int ret;

	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",button.port->name);
		return ENODEV;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",ret, button.port->name, button.pin);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",ret, button.port->name, button.pin);
		return ret;
	}

	booting = true;
	pressed = button_is_pressed();
	triggered = false;
	press_time = k_uptime_get();//in case of boot with btn pressed
	k_work_init_delayable(&long_press_work, button_hold_timeout);

	gpio_init_callback(&button_cb_data, button_interrupt, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	return ret;
}

void app_button_set_short_callback(button_callback_handler_t cb){
	button_short_press = cb;
}

void app_button_set_long_callback(button_callback_handler_t cb){
	button_long_press = cb;
}

void app_button_set_long_timeout(int64_t timeout_ms){
	long_press_timeout_ms = timeout_ms;
}

void app_button_set_short_timeout(int64_t timeout_ms){
	short_press_timeout_ms = timeout_ms;
}
