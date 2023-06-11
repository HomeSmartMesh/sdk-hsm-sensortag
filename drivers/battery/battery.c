
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static int16_t sample;
int32_t adc_vref;

static const struct adc_channel_cfg ch0_cfg_dt =
    ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc), channel_0));

struct adc_sequence sequence = {
	.channels    = BIT(ADC_CHANNEL),
	.buffer      = &sample,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample),
	.resolution  = ADC_RESOLUTION,
};

void battery_init()
{
	if (!device_is_ready(dev_adc)) {
		LOG_ERR("ADC device not found\n");
		return;
	}

	adc_channel_setup(dev_adc, &ch0_cfg_dt);
	adc_vref = adc_ref_internal(dev_adc);

	LOG_INF("battery_init() vref = %d",adc_vref);
}

void battery_start()
{
	int err = adc_read(dev_adc, &sequence);
	if (err != 0) {
		LOG_ERR("ADC reading failed with error %d.\n", err);
		return;
	}else{
		LOG_DBG("battery_start> adc_read()");
	}
}

int32_t battery_get_mv()
{
	int32_t raw_value = sample;
	LOG_INF("battery_get_mv() raw = %d",raw_value);
	int32_t mv_value = raw_value;
	adc_raw_to_millivolts(adc_vref, ADC_GAIN,ADC_RESOLUTION, &mv_value);
	return mv_value;
}
