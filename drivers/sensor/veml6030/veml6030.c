/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vishay_veml6030

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include <sensor/veml6030.h>

LOG_MODULE_REGISTER(VEML6030, CONFIG_SENSOR_LOG_LEVEL);

static int veml6030_reg_read(struct veml6030_data *drv_data, uint8_t reg,uint8_t *val, bool send_stop)
{
	struct i2c_msg msgs[2] = {
		{
			.buf = &reg,
			.len = 1,
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = val,
			.len = 2,
			.flags = I2C_MSG_READ,
		},
	};

	if (send_stop) {
		msgs[1].flags |= I2C_MSG_STOP;
	}

	if (i2c_transfer(drv_data->i2c, msgs, 2, VEML6030_I2C_ADDRESS) != 0) {
		return -EIO;
	}

	return 0;
}

static int veml6030_reg_write(struct veml6030_data *drv_data, uint8_t reg,uint16_t val)
{
	uint8_t tx_buf[3] = {reg, (uint8_t)(val&0xFF), (uint8_t)(val>>8)};

	return i2c_write(drv_data->i2c, tx_buf, sizeof(tx_buf), VEML6030_I2C_ADDRESS);
}

static int veml6030_reg_update(struct veml6030_data *drv_data, uint8_t reg,uint8_t mask, uint8_t val)
{
	uint16_t old_val = 0U;
	uint16_t new_val = 0U;

	if (veml6030_reg_read(drv_data, reg, (uint8_t*)&old_val, true) != 0) {
		return -EIO;
	}

	new_val = old_val & ~mask;
	new_val |= val & mask;

	return veml6030_reg_write(drv_data, reg, new_val);
}

static int veml6030_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct veml6030_data *drv_data = dev->data;

	if (veml6030_reg_update(drv_data, VEML6030_REG_ALS_CONF, VEML6030_ALS_CONF_ALS_SD, VEML6030_ALS_CONF_ALS_SD_ON))
	{
		return -EIO;
	}

	return 0;
}

static int veml6030_sample_fetch(const struct device *dev,enum sensor_channel chan)
{
	struct veml6030_data *drv_data = dev->data;
	uint8_t vals[2];
	uint16_t val;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT);

	drv_data->sample = 0U;

	if(i2c_burst_read(drv_data->i2c , VEML6030_I2C_ADDRESS, VEML6030_REG_ALS, vals, 2)){
		return -EIO;
	}
	//if (veml6030_reg_read(drv_data, VEML6030_REG_ALS, vals,true) != 0) {
	//	return -EIO;
	//}
	val = vals[1];
	val = val * 256 + vals[0];
	drv_data->sample = val;

	return 0;
}

static int veml6030_channel_get(const struct device *dev,enum sensor_channel chan,struct sensor_value *val)
{
	struct veml6030_data *drv_data = dev->data;

	if (chan != SENSOR_CHAN_LIGHT) {
		return -ENOTSUP;
	}

	val->val1 = drv_data->sample;

	return 0;
}

static const struct sensor_driver_api veml6030_driver_api = {
	.attr_set = veml6030_attr_set,
	.sample_fetch = veml6030_sample_fetch,
	.channel_get = veml6030_channel_get,
};

int veml6030_init(const struct device *dev)
{
	struct veml6030_data *drv_data = dev->data;
	LOG_INF("veml6030_init() power on");
	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}
	uint8_t wdata[2];
	uint8_t rdata[2];
	wdata[0] = 1;
	wdata[1] = 0;
	int ret = i2c_burst_write(drv_data->i2c , VEML6030_I2C_ADDRESS, VEML6030_REG_ALS_CONF, wdata, 2);
	if (ret) {
		LOG_ERR("i2c_burst_write() failed");
	} else {
		LOG_INF("i2c_burst_write() success");
		ret = i2c_burst_read(drv_data->i2c , VEML6030_I2C_ADDRESS, VEML6030_REG_ALS_CONF, rdata, 2);
		if (ret) {
			LOG_ERR("i2c_burst_read() failed");
		} else {
			LOG_INF("i2c_burst_read() success 0x%X:0x%X",rdata[1],rdata[0]);
		}
	}

	return 0;
}

static struct veml6030_data veml6030_drv_data;

DEVICE_DT_INST_DEFINE(0, veml6030_init, device_pm_control_nop,
	    &veml6030_drv_data, NULL, POST_KERNEL,
	    CONFIG_SENSOR_INIT_PRIORITY, &veml6030_driver_api);
