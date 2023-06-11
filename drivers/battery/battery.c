
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif


#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static int16_t sample;
int32_t adc_vref;
struct adc_sequence sequence = {
	.buffer      = &sample,
	.buffer_size = sizeof(sample)
};

void battery_init()
{
	int err;
	if (!device_is_ready(adc_channels[0].dev)) {
		LOG_ERR("ADC controller device not ready\n");
		return;
	}
	err = adc_channel_setup_dt(&adc_channels[0]);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)\n", 0, err);
		return;
	}
	LOG_INF("- %s, channel %d: ",
			adc_channels[0].dev->name,
			adc_channels[0].channel_id);

	adc_vref = adc_ref_internal(adc_channels[0].dev);
	LOG_INF("battery_init() vref = %d",adc_vref);
}

void battery_start()
{
	(void)adc_sequence_init_dt(&adc_channels[0], &sequence);
	int err = adc_read(adc_channels[0].dev, &sequence);
	if (err != 0) {
		LOG_ERR("ADC reading failed with error %d.\n", err);
		return;
	}else{
		LOG_DBG("battery_start> adc_read()");
	}
}

int32_t battery_get_mv()
{
	int32_t mv_value = sample;
	LOG_DBG("battery_get_mv() raw = %d",mv_value);
	adc_raw_to_millivolts_dt(&adc_channels[0], &mv_value);
	return mv_value;
}
