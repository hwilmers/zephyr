/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2019 Wilmers Messtechnik
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMP388_BMP388_H_
#define ZEPHYR_DRIVERS_SENSOR_BMP388_BMP388_H_

#include <zephyr/types.h>
#include <device.h>

/* Header includes */
#include "bmp3_defs.h"
#include "bmp3.h"


/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif


struct bmp388_data {
#ifdef DT_BOSCH_BMP388_BUS_I2C
	struct device *i2c_master;
	u16_t i2c_slave_addr;
#elif defined DT_BOSCH_BMP388_BUS_SPI
	struct device *spi;
	struct spi_config spi_cfg;
#else
#error "BMP388 device type not specified"
#endif

	/* Compensated values. */
	struct bmp3_data val;

};


static int8_t cal_crc(uint8_t seed, uint8_t data);
static int8_t validate_trimming_param(const struct bmp3_dev *dev);
static int8_t analyze_sensor_data(const struct bmp3_data *sens_data);
int8_t bosch_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bosch_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmp_delay_ms(uint32_t period);

static int bmp388_reg_read(struct bmp388_data *data, u8_t start, u8_t *buf, int size);
static int bmp388_reg_write(struct bmp388_data *data, u8_t reg, u8_t val);


#define BMP388_CTRL_MEAS_VAL            (BMP388_PRESS_OVER | \
					 BMP388_TEMP_OVER |  \
					 BMP388_MODE_NORMAL)
#define BMP388_CONFIG_VAL               (BMP388_STANDBY | \
					 BMP388_FILTER |  \
					 BMP388_SPI_3W_DISABLE)


#define BMP3_SENSOR_OK                                     UINT8_C(0)
#define BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE           UINT8_C(10)
#define BMP3_TRIMMING_DATA_OUT_OF_BOUND                    UINT8_C(20)
#define BMP3_TEMPERATURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT UINT8_C(30)
#define BMP3_PRESSURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT    UINT8_C(31)
#define BMP3_IMPLAUSIBLE_TEMPERATURE                       UINT8_C(40)
#define BMP3_IMPLAUSIBLE_PRESSURE                          UINT8_C(41)
#endif /* ZEPHYR_DRIVERS_SENSOR_BME280_BME280_H_ */
