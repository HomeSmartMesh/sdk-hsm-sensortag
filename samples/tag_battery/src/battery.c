
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/adc.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define ADC_CHANNEL				0
#define ADC_RESOLUTION			12
#define ADC_GAIN				ADC_GAIN_1_6
#define ADC_REFERENCE			ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS,40)

const struct device *dev_adc;
static int16_t sample;
int32_t adc_vref;
struct adc_channel_cfg channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL,
	#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
	.differential = 0,
	.input_positive = SAADC_CH_PSELN_PSELN_VDD
	#else
	.differential = 0
	#endif
};

struct adc_sequence sequence = {
	.channels    = BIT(ADC_CHANNEL),
	.buffer      = &sample,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(sample),
	.resolution  = ADC_RESOLUTION,
};

void battery_init()
{
	dev_adc = device_get_binding(DT_LABEL(DT_NODELABEL(adc)));
	if (!device_is_ready(dev_adc)) {
		LOG_ERR("ADC device not found\n");
		return;
	}

	adc_channel_setup(dev_adc, &channel_cfg);
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
		LOG_INF("battery_start> adc_read()");
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
