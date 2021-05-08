/**
 * \file ms8607.c
 *
 * \brief MS8607 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms8607 datasheet :
 * http://www.meas-spec.com/downloads/MS8607D.pdf
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <stdio.h>
#include <sensor/ms8607.h>


#include "ms8607_i2c.h"

LOG_MODULE_REGISTER(MS8607, CONFIG_SENSOR_LOG_LEVEL);

#define SENSOR_LOG_LEVEL_OFF      0
#define SENSOR_LOG_LEVEL_ERROR    1
#define SENSOR_LOG_LEVEL_WARNING  2
#define SENSOR_LOG_LEVEL_INFO     3
#define SENSOR_LOG_LEVEL_DEBUG    4



struct ms8607_data *drv_data;

void delay_ms(uint32_t time)
{
    k_sleep(K_MSEC(time));
}

void i2c_master_init(const struct device *dev)
{
    drv_data = dev->data;
    LOG_INF("ms8607_init> i2c_master_init()");
}

enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet)
{
	if (i2c_read(drv_data->i2c , packet->data, packet->data_length, packet->address)) {
		return STATUS_ERR_TIMEOUT;
	}

    return STATUS_OK;
}

enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet)
{
	if(i2c_write(drv_data->i2c , packet->data, packet->data_length, packet->address)){
        return STATUS_ERR_TIMEOUT;
    }

    return STATUS_OK;
}

enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet)
{
    struct i2c_msg msg;

	msg.buf = packet->data;
	msg.len = packet->data_length;
	msg.flags = I2C_MSG_WRITE;// | I2C_MSG_STOP removed

	if(i2c_transfer(drv_data->i2c, &msg, 1, packet->address)){
        return STATUS_ERR_TIMEOUT;
    }

    return STATUS_OK;
}


#ifdef __cplusplus
}
#endif
