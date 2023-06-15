
#ifndef __APP_LED_H__
#define __APP_LED_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int app_led_init(void);
int app_led_set_red(float brightness);
int app_led_set_green(float brightness);
int app_led_set_blue(float brightness);
int app_led_set_color(float r,float g, float b);
int app_led_on();
int app_led_off();

int app_led_blink_red(float brightness,int32_t up, int32_t down);
int app_led_blink_green(float brightness,int32_t up, int32_t down);
int app_led_blink_blue(float brightness,int32_t up, int32_t down);
int app_led_blink_color(float r,float g, float b,int32_t up, int32_t down);

#ifdef __cplusplus
}
#endif

#endif /*__APP_LED_H__*/
