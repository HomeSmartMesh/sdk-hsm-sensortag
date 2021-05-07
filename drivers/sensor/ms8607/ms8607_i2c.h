/**
 * \brief MS8607 Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to ms8607 datasheet :
 * http://www.meas-spec.com/downloads/MS8607D.pdf
 *
 */

#ifndef MS8607_I2C_H_INCLUDED
#define MS8607_I2C_H_INCLUDED

#include <stdint.h>
#include <device.h>

enum i2c_transfer_direction {
	I2C_TRANSFER_WRITE = 0,
	I2C_TRANSFER_READ  = 1,
};

enum status_code {
	STATUS_OK           = 0x00,
	STATUS_ERR_OVERFLOW	= 0x01,
	STATUS_ERR_TIMEOUT  = 0x02,
};

struct i2c_master_packet {
	// Address to slave device
	uint16_t address;
	// Length of data array
	uint16_t data_length;
	// Data array containing all data to be transferred
	uint8_t *data;
};

struct ms8607_data {
	const struct device *i2c;
	uint16_t 	sample;
	float		humidity;
	float		pressure;
	float		temperature;
};


void delay_ms(uint32_t time);
void i2c_master_init(const struct device *dev);
enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);
enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);
enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);



#endif /* MS8607_I2C_H_INCLUDED */
