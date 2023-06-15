#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec red_pwm_led =	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct pwm_dt_spec green_pwm_led =	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
static const struct pwm_dt_spec blue_pwm_led =	PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));

//not calibrated, fixed for specific board
#define PWM_PERIOD PWM_MSEC(20)

int init_led(const struct pwm_dt_spec pwm_led){
	int ret = 0;
	if (!device_is_ready(pwm_led.dev)) {
		printk("PWM LED Init error '%s' device_is_ready()\n",pwm_led.dev->name);
		return ret;
	}

    ret = pwm_set_dt(&pwm_led, PWM_PERIOD, 0);
    if (ret) {
        printk("Error %d: failed to set pulse width for %s\n", ret,pwm_led.dev->name);
        return ret;
    }
    return ret;
}

int set_led(const struct pwm_dt_spec pwm_led,float brightness){
    int ret = 0;
    uint32_t pulse = brightness * PWM_PERIOD;
	ret = pwm_set_pulse_dt(&pwm_led, pulse);
	if (ret < 0) {
		printk("LED Init error pwm_set_pulse_dt()\n");
		return ret;
	}
    return ret;
}

int app_led_set_red(float brightness){
	pwm_set_pulse_dt(&green_pwm_led, 0);
	pwm_set_pulse_dt(&blue_pwm_led, 0);
	return set_led(red_pwm_led, brightness);
}
int app_led_set_green(float brightness){
	pwm_set_pulse_dt(&red_pwm_led, 0);
	pwm_set_pulse_dt(&blue_pwm_led, 0);
	return set_led(green_pwm_led, brightness);
}
int app_led_set_blue(float brightness){
	pwm_set_pulse_dt(&red_pwm_led, 0);
	pwm_set_pulse_dt(&green_pwm_led, 0);
	return set_led(blue_pwm_led, brightness);
}

int app_led_set_color(float r,float g, float b){
    uint32_t red = r * PWM_PERIOD;
    uint32_t green = g * PWM_PERIOD;
    uint32_t blue = b * PWM_PERIOD;
	pwm_set_pulse_dt(&red_pwm_led, red);
	pwm_set_pulse_dt(&green_pwm_led, green);
	pwm_set_pulse_dt(&blue_pwm_led, blue);
	return 0;
}

int app_led_on(){
	pwm_set_pulse_dt(&red_pwm_led, PWM_PERIOD);
	pwm_set_pulse_dt(&green_pwm_led, PWM_PERIOD);
	pwm_set_pulse_dt(&blue_pwm_led, PWM_PERIOD);
	return 0;
}

int app_led_off(){
	pwm_set_pulse_dt(&red_pwm_led, 0);
	pwm_set_pulse_dt(&green_pwm_led, 0);
	pwm_set_pulse_dt(&blue_pwm_led, 0);
	return 0;
}

int app_led_blink_red(float brightness,int32_t up,int32_t down){
	app_led_set_red(brightness);
	k_msleep(up);
	app_led_off();
	k_msleep(down);
	return 0;
}
int app_led_blink_green(float brightness,int32_t up,int32_t down){
	app_led_set_green(brightness);
	k_msleep(up);
	app_led_off();
	k_msleep(down);
	return 0;
}
int app_led_blink_blue(float brightness,int32_t up,int32_t down){
	app_led_set_blue(brightness);
	k_msleep(up);
	app_led_off();
	k_msleep(down);
	return 0;
}

int app_led_blink_color(float r,float g, float b,int32_t up,int32_t down){
	app_led_set_color(r,g,b);
	k_msleep(up);
	app_led_off();
	k_msleep(down);
	return 0;
}


int app_led_init(void){
    init_led(red_pwm_led);
    init_led(green_pwm_led);
    init_led(blue_pwm_led);
    return 0;
}
