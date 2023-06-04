/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * Edited for app_battery optimizations
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <hal/nrf_saadc.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#define VBATT DT_PATH(vbatt)
#define ZEPHYR_USER DT_PATH(zephyr_user)

static const uint32_t sFullOhms = DT_PROP(VBATT, full_ohms);
static const uint32_t sOutputOhms = DT_PROP(VBATT, output_ohms);
static const struct adc_dt_spec sAdc = ADC_DT_SPEC_GET(VBATT);

int16_t sAdcBuffer = 0;

struct adc_sequence sAdcSeq = {
	.buffer = &sAdcBuffer,
	.buffer_size = sizeof(sAdcBuffer),
	.calibrate = true,
};

int app_battery_init()
{
	int err = 0;

	if (!device_is_ready(sAdc.dev)) {
		printk("ADC controller not ready");
		return -ENODEV;
	}

	err += adc_channel_setup_dt(&sAdc);
	if (err) {
		printk("Setting up the ADC channel failed");
		return err;
	}

	(void)adc_sequence_init_dt(&sAdc, &sAdcSeq);

	return err;
}

int32_t app_battery_voltage_mv()
{
	int32_t result = 0;
    int ret = adc_read(sAdc.dev, &sAdcSeq);
    if (ret == 0) {
        int32_t val = sAdcBuffer;
        adc_raw_to_millivolts_dt(&sAdc, &val);
        result = (int32_t)((int64_t)(val) * sFullOhms / sOutputOhms);
    }else{
        printk("ADC Fail = %d\n",ret);
    }
	return result;
}
