#ifndef ZEPHYR_DRIVERS_SENSOR_VEML6030_VEML6030_H_
#define ZEPHYR_DRIVERS_SENSOR_VEML6030_VEML6030_H_

#include <sys/util.h>

#define VEML6030_I2C_ADDRESS	DT_INST_REG_ADDR(0)

//----------------------------------------------------------------------
#define VEML6030_REG_ALS_CONF		0x00
#define VEML6030_REG_ALS_WH			0x01
#define VEML6030_REG_ALS_WL			0x02
#define VEML6030_REG_POWER_SAVING	0x03
#define VEML6030_REG_ALS			0x04
#define VEML6030_REG_WHITE			0x05
#define VEML6030_REG_ALS_INT		0x06

#define VEML6030_ALS_CONF_ALS_SD		BIT(0)
#define VEML6030_ALS_CONF_ALS_INT_EN	BIT(1)

//Configuration Register
#define VEML6030_ALS_CONF_ALS_SD_MASK				BIT(0)
#define VEML6030_ALS_CONF_ALS_SD_ON					0x00
#define VEML6030_ALS_CONF_ALS_SD_OFF				BIT(0)

#define VEML6030_ALS_CONF_ALS_ALS_INT_EN_MASK		BIT(1)
#define VEML6030_ALS_CONF_ALS_ALS_INT_EN_ENABLE		BIT(1)
#define VEML6030_ALS_CONF_ALS_ALS_INT_EN_DISABLE	0x00

#define VEML6030_ALS_CONF_ALS_PRES_MASK				(BIT(4)|BIT(5))
#define VEML6030_ALS_CONF_ALS_PERS_1				0x00
#define VEML6030_ALS_CONF_ALS_PERS_2				BIT(4)
#define VEML6030_ALS_CONF_ALS_PERS_4				BIT(5)
#define VEML6030_ALS_CONF_ALS_PERS_8				(BIT(4)|BIT(5))

#define VEML6030_ALS_CONF_ALS_IT_MASK				(BIT(6)|BIT(7)|BIT(8)|BIT(9))
#define VEML6030_ALS_CONF_ALS_IT_25_ms				(BIT(8)|BIT(9))
#define VEML6030_ALS_CONF_ALS_IT_50_ms				BIT(9)
#define VEML6030_ALS_CONF_ALS_IT_100_ms				0x00
#define VEML6030_ALS_CONF_ALS_IT_200_ms				BIT(6)
#define VEML6030_ALS_CONF_ALS_IT_400_ms				BIT(7)
#define VEML6030_ALS_CONF_ALS_IT_800_ms				(BIT(6)|BIT(7))

#define VEML6030_ALS_CONF_ALS_GAIN_MASK				(BIT(11)|BIT(12))
#define VEML6030_ALS_CONF_ALS_GAIN_x1				0x00
#define VEML6030_ALS_CONF_ALS_GAIN_x2				BIT(11)
#define VEML6030_ALS_CONF_ALS_GAIN_x1_8				BIT(12)
#define VEML6030_ALS_CONF_ALS_GAIN_x1_4				(BIT(11)|BIT(12))


struct veml6030_data {
	const struct device *i2c;
	uint16_t sample;
};

//----------------- additional specific API ---------------------

int veml6030_power_on(const struct device *dev);
int veml6030_power_off(const struct device *dev);

#endif /* _SENSOR_VEML6030_ */
