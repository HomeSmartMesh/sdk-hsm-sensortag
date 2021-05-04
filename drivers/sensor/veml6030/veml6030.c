#define DT_DRV_COMPAT vishay_veml6030

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <stdio.h>

#include <sensor/veml6030.h>

LOG_MODULE_REGISTER(VEML6030, CONFIG_SENSOR_LOG_LEVEL);

#define SENSOR_LOG_LEVEL_OFF      0
#define SENSOR_LOG_LEVEL_ERROR    1
#define SENSOR_LOG_LEVEL_WARNING  2
#define SENSOR_LOG_LEVEL_INFO     3
#define SENSOR_LOG_LEVEL_DEBUG    4

const float ref_gain_25_ms_1_8 = 1.8432;
float lut_gain[] = {1,2,0.125,0.25};
uint16_t lut_integration_ms[] = {100,200,400,800,0,0,0,0,50,0,0,0,25,0,0,0};

// * Strategy tries to diminish gain first
// * avoids edges by roughly ~ 2%, the last value within range would not tell if it is correct or saturated
// * (8) takes UINT32_MAX because that's the lowest possible gain and integration time
float modes_illum[] = {
	230.0,	// (0)
	460.0,	// (1)
	1840.0,	// (2)
	3680.0,	// (3)
	7360.0,	// (4)
	14720.0,	// (5)
	29440.0,	// (6)
	58880.0,	// (7)
	200000.0,	// (8)
	};
uint16_t modes_flags[] = {
	VEML6030_ALS_CONF_ALS_IT_800_ms | VEML6030_ALS_CONF_ALS_GAIN_x2,	// (0)
	VEML6030_ALS_CONF_ALS_IT_800_ms | VEML6030_ALS_CONF_ALS_GAIN_x1,	// (1)
	VEML6030_ALS_CONF_ALS_IT_800_ms | VEML6030_ALS_CONF_ALS_GAIN_x1_4,	// (2)
	VEML6030_ALS_CONF_ALS_IT_800_ms | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (3)
	VEML6030_ALS_CONF_ALS_IT_400_ms | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (4)
	VEML6030_ALS_CONF_ALS_IT_200_ms | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (5)
	VEML6030_ALS_CONF_ALS_IT_100_ms | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (6)
	VEML6030_ALS_CONF_ALS_IT_50_ms  | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (7)
	VEML6030_ALS_CONF_ALS_IT_25_ms  | VEML6030_ALS_CONF_ALS_GAIN_x1_8,	// (8)
};

float reg_to_lum_lux(uint16_t val, uint16_t it_ms, float gain)
{
	float resolution = 1.8432 / ((gain/0.125) * (it_ms/25));
	//uint32_t res = resolution * 1000;
	//if(gain == 0.125)LOG_DBG("gain = 0.125");
	//else if(gain == 0.25)LOG_DBG("gain = 0.25");
	//else if(gain == 1)LOG_DBG("gain = 1");
	//else if(gain == 2)LOG_DBG("gain = 2");
	//else LOG_DBG("gain = %d uGain",(int)(gain*1000));
	//LOG_DBG("integration time = %d ; resolution = %d mRes",it_ms,res);
	float lum_lux = val;
	float x = lum_lux * resolution;
	//y = 6.0135E-13x4 - 9.3924E-09x3 + 8.1488E-05x2 + 1.0023E+00x
	//correction will always be applied to avoid edges
	const float C0 = 1.0023;
	const float C1 = 8.1488E-05;
	const float C2 = -9.3924E-09;
	const float C3 = 6.0135E-13;
	float x_2 = x * x;
	float x_3 = x_2 * x;
	float x_4 = x_3 * x;
	float y = x_4 * C3 + x_3 * C2 + x_2 * C1 + x * C0;
	//printf("measure = %f ; corrected measure = %f",x,y);
	return y;
}

uint8_t get_optimal_mode(uint16_t sample,float lum_lux)
{
	//actually, on saturation, set the highest mode to avoid slow raming up
	if(sample == 65535){
		return 8;
	}
	else
	{
		uint8_t index = 0;
		while((lum_lux > modes_illum[index])&&(index<8))
		{
			index++;
		}
		return index;
	}
}

static int veml6030_reg_read(struct veml6030_data *drv_data, uint8_t reg,uint16_t *val)
{
	uint8_t valb[2];
	if (i2c_burst_read(drv_data->i2c , VEML6030_I2C_ADDRESS, reg, valb, 2)) {
		return -EIO;
	}

	(*val) = valb[1];
	(*val) = ((*val)<<8) + valb[0];

	return 0;
}

static int veml6030_reg_write(struct veml6030_data *drv_data, uint8_t reg,uint16_t val)
{
	uint8_t bdata[2];
	bdata[0] = (uint8_t)(val&0xFF);
	bdata[1] = (uint8_t)(val>>8);

	return i2c_burst_write(drv_data->i2c , VEML6030_I2C_ADDRESS, reg, bdata, 2);
}

static int veml6030_reg_update(struct veml6030_data *drv_data, uint8_t reg,uint8_t mask, uint8_t val)
{
	uint16_t old_val = 0U;
	uint16_t new_val = 0U;

	if (veml6030_reg_read(drv_data, reg, &old_val) != 0) {
		return -EIO;
	}

	new_val = old_val & ~mask;
	new_val |= val & mask;

	return veml6030_reg_write(drv_data, reg, new_val);
}

static int veml6030_attr_set(	const struct device *dev,enum sensor_channel chan,
			     				enum sensor_attribute attr,const struct sensor_value *val)
{
	struct veml6030_data *drv_data = dev->data;

	if (veml6030_reg_update(drv_data, VEML6030_REG_ALS_CONF, VEML6030_ALS_CONF_ALS_SD_MASK, VEML6030_ALS_CONF_ALS_SD_ON))
	{
		return -EIO;
	}

	return 0;
}

static int veml6030_sample_fetch(const struct device *dev,enum sensor_channel chan)
{
	struct veml6030_data *drv_data = dev->data;
	uint16_t val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT);

	drv_data->sample = 0U;

	if(veml6030_reg_read(drv_data , VEML6030_REG_ALS, &val)){
		return -EIO;
	}
	drv_data->sample = val;
	return 0;
}

static int veml6030_channel_get(const struct device *dev,enum sensor_channel chan,struct sensor_value *val)
{
	struct veml6030_data *drv_data = dev->data;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}
	float resolution = 1.8432 / ((drv_data->gain/0.125) * (drv_data->integration_ms/25));
	val->val1 = drv_data->sample * resolution;										//tunc
	val->val2 = (drv_data->sample * resolution * 1000000) - (val->val1 * 1000000);	//fraction - millionth

	return 0;
}

static const struct sensor_driver_api veml6030_driver_api = {
	.attr_set = veml6030_attr_set,
	.sample_fetch = veml6030_sample_fetch,
	.channel_get = veml6030_channel_get,
};

static int veml6030_power_update(struct veml6030_data *drv_data, uint16_t val)
{
	uint16_t old_config = 0U;
	uint16_t new_config = 0U;

	if (veml6030_reg_read(drv_data, VEML6030_REG_ALS_CONF, &old_config) != 0) {
		return -EIO;
	}
	//update params needed in every config update - here in power with existing config from sensors only
	uint16_t integration_bits = old_config & VEML6030_ALS_CONF_ALS_IT_MASK;
	uint16_t gain_bits = old_config & VEML6030_ALS_CONF_ALS_GAIN_MASK;
	drv_data->config_it_gain = integration_bits | gain_bits;
	drv_data->integration_ms = lut_integration_ms[integration_bits>>6];
	drv_data->gain = lut_gain[gain_bits>>11];

	new_config = old_config & ~VEML6030_ALS_CONF_ALS_SD_MASK;
	new_config |= val & VEML6030_ALS_CONF_ALS_SD_MASK;

	return veml6030_reg_write(drv_data, VEML6030_REG_ALS_CONF, new_config);
}

static int veml6030_it_gain_update(struct veml6030_data *drv_data, uint16_t val)
{
	uint16_t old_config = 0U;
	uint16_t new_config = 0U;
	int ret = 0;

	if (veml6030_reg_read(drv_data, VEML6030_REG_ALS_CONF, &old_config) != 0) {
		LOG_ERR("veml6030_reg_read() failed");
		return -EIO;
	}
	uint16_t mask = (VEML6030_ALS_CONF_ALS_IT_MASK | VEML6030_ALS_CONF_ALS_GAIN_MASK);
	new_config = old_config & ~mask;
	new_config |= val & mask;

	uint16_t integration_bits = new_config & VEML6030_ALS_CONF_ALS_IT_MASK;
	uint16_t gain_bits = new_config & VEML6030_ALS_CONF_ALS_GAIN_MASK;
	drv_data->config_it_gain = integration_bits | gain_bits;
	drv_data->integration_ms = lut_integration_ms[integration_bits>>6];
	drv_data->gain = lut_gain[gain_bits>>11];

	ret =  veml6030_reg_write(drv_data, VEML6030_REG_ALS_CONF, new_config);
	if(ret){
		LOG_ERR("veml6030_reg_write() failed");
	}
	return ret;
}

int veml6030_power_on(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	int ret = veml6030_power_update(drv_data,VEML6030_ALS_CONF_ALS_SD_ON);
	if (ret) {
		LOG_ERR("veml6030_power_update() failed");
	}
	return ret;
}

int veml6030_power_off(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	int ret = veml6030_power_update(drv_data,VEML6030_ALS_CONF_ALS_SD_OFF);
	if (ret) {
		LOG_ERR("veml6030_power_update() failed");
	}
	return ret;
}

int veml6030_init(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	LOG_INF("veml6030_init()");
	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static struct veml6030_data veml6030_drv_data;

DEVICE_DT_INST_DEFINE(0, veml6030_init, device_pm_control_nop,
	    &veml6030_drv_data, NULL, POST_KERNEL,
	    CONFIG_SENSOR_INIT_PRIORITY, &veml6030_driver_api);

float veml6030_fetch_lux(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	if (veml6030_sample_fetch(dev, SENSOR_CHAN_LIGHT) != 0) {
		LOG_ERR("sensor: sample fetch fail.");
		return 0;
	}

	return reg_to_lum_lux(drv_data->sample,drv_data->integration_ms, drv_data->gain);
}

//a measure might not be good (e.g. saturated) if not taken with optimal config that is checked after the measure has taken place
float veml6030_auto_measure(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	float measure_lux = 0;

	veml6030_power_on(dev);//4 ms min after power on => will update the drv_data ->integration_ms and ->gain

	bool optimal_measure = false;// no measure yet
	while(!optimal_measure)
	{
		k_sleep(K_MSEC(2*drv_data->integration_ms+5));//wait the integration time +5 ms clock drift safety : x2 as x1 fail with reading 0
		measure_lux = veml6030_fetch_lux(dev);
		uint8_t optimal_mode = get_optimal_mode(drv_data->sample,measure_lux);
		if(modes_flags[optimal_mode] == drv_data->config_it_gain)//optimal mode already selected
		{
			optimal_measure = true;
		}
		else
		{
			printf("auto_measure>sample %d not optimal ; gain = %.3f ; it = %d ms\n",
					drv_data->sample, drv_data->gain, drv_data->integration_ms);
			//update both integration and gain to the new optimal value
			veml6030_it_gain_update(drv_data,modes_flags[optimal_mode]);
			printf("auto_measure>new params => gain = %f ; it = %d\n",drv_data->gain,drv_data->integration_ms);
			optimal_measure = false;
		}
	}

	veml6030_power_off(dev);
	drv_data->lum_lux = measure_lux;
	drv_data->lum_lux = measure_lux;
	return measure_lux;
}
