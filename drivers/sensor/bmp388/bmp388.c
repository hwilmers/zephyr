/* bmp388.c - Driver for Bosch BMP388 temperature and pressure sensor */

/*
 * Copyright (c) 2016, 2017 Intel Corporation
 * Copyright (c) 2017 IpTronix S.r.l.
 * Copyright (c) 2019 Wilmers Messtechnik
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>

#include <logging/log.h>
#include "bmp388.h"

#ifdef DT_BOSCH_BMP388_BUS_I2C
#include <drivers/i2c.h>
#elif defined DT_BOSCH_BMP388_BUS_SPI
#include <drivers/spi.h>
#endif


static struct bmp388_data *bmp388_global;


LOG_MODULE_REGISTER(BMP388, CONFIG_SENSOR_LOG_LEVEL);


/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x1D;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;
		} else   {
			var2 = 0;
		}
		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}


static int8_t validate_trimming_param(const struct bmp3_dev *dev)
{
	int8_t rslt;
	uint8_t crc = 0xFF;
	uint8_t stored_crc;
	uint8_t trim_param[21];
	uint8_t i;

	rslt = bmp3_get_regs(BMP3_CALIB_DATA_ADDR, trim_param, 21, dev);
	if (rslt == BMP3_SENSOR_OK) {
		for (i = 0; i < 21; i++) {
			crc = (uint8_t)cal_crc(crc, trim_param[i]);
		}
		crc = (crc ^ 0xFF);
		rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
		if (stored_crc != crc) {
			rslt = BMP3_TRIMMING_DATA_OUT_OF_BOUND;
		}

	}

	return rslt;

}

/*!
 * @brief  Function to analyze the sensor data
 */
static int8_t analyze_sensor_data(const struct bmp3_data *sens_data)
{
	int8_t rslt = BMP3_SENSOR_OK;

	if ((sens_data->temperature < 0) || (sens_data->temperature > 4000)) {
		rslt = BMP3_IMPLAUSIBLE_TEMPERATURE;
	}
	if (rslt == BMP3_SENSOR_OK) {
		if ((sens_data->pressure / 100 < 90000) || (sens_data->pressure / 100 > 110000)) {
			rslt = BMP3_IMPLAUSIBLE_PRESSURE;
		}
	}

	return rslt;
}
void bmp_delay_ms(uint32_t period)
{
	k_sleep(K_MSEC(period));
}
int8_t bosch_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int result = bmp388_reg_read(bmp388_global, reg_addr, data, (int) len);

	return result;
}

/*!
 * @brief This API writes data to specific register with certain length
 *
 * @param[in] dev_id : Device ID
 *			 reg_addr : Address of the Register
 *           data : data pointer for writing
 *           len  : length of the data array to write
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */

int8_t bosch_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int result =  bmp388_reg_write(bmp388_global, reg_addr, *data);

	return result;
}

static int bmp388_reg_read(struct bmp388_data *data, u8_t start, u8_t *buf, int size)
{
#ifdef DT_BOSCH_BMP388_BUS_I2C
	return i2c_burst_read(data->i2c_master, data->i2c_slave_addr,
			      start, buf, size);
#elif defined DT_BOSCH_BMP388_BUS_SPI
	u8_t addr;
	const struct spi_buf tx_buf = {
		.buf = &addr,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[2];
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};
	int i;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].len = 1;

	for (i = 0; i < size; i++) {
		int ret;

		addr = (start + i) | 0x80;
		rx_buf[1].buf = &buf[i];

		ret = spi_transceive(data->spi, &data->spi_cfg, &tx, &rx);
		if (ret) {
			LOG_DBG("spi_transceive FAIL %d\n", ret);
			return ret;
		}
	}
#endif
	return 0;
}

static int bmp388_reg_write(struct bmp388_data *data, u8_t reg, u8_t val)
{
#ifdef DT_BOSCH_BMP388_BUS_I2C
	return i2c_reg_write_byte(data->i2c_master, data->i2c_slave_addr,
				  reg, val);
#elif defined DT_BOSCH_BMP388_BUS_SPI
	u8_t cmd[2] = { reg & 0x7F, val };
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = 2
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	int ret;

	ret = spi_write(data->spi, &data->spi_cfg, &tx);
	if (ret) {
		LOG_DBG("spi_write FAIL %d\n", ret);
		return ret;
	}
#endif
	return 0;
}

static int bmp388_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bmp388_data *data = dev->driver_data;

	int8_t rslt;
	struct bmp3_data data_sens = { 0 };
	uint8_t sensor_comp;

	/* Used to select the settings user needs to change */
	uint16_t settings_sel;
	struct bmp3_dev t_dev;

	t_dev.chip_id = BMP3_CHIP_ID;
	t_dev.read = &bosch_read;
	t_dev.write = &bosch_write;
	t_dev.intf = BMP3_I2C_INTF;
	t_dev.delay_ms = &bmp_delay_ms;

	/* resetting of the sensor */
	rslt = bmp3_soft_reset(&t_dev);
	if (rslt != BMP3_SENSOR_OK) {
		LOG_DBG("BMP388 Soft Reset Failed");
	}

	/* initializing Bosch Driver Code */
	rslt = bmp3_init(&t_dev);

	if (rslt == BMP3_E_COMM_FAIL || rslt == BMP3_E_DEV_NOT_FOUND) {
		rslt = BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
	}

	if (rslt == BMP3_SENSOR_OK) {
		rslt = validate_trimming_param(&t_dev);
	}

	if (rslt == BMP3_SENSOR_OK) {
		t_dev.settings.press_en = BMP3_ENABLE;
		t_dev.settings.temp_en = BMP3_ENABLE;

		/* Select the output data rate and over sampling settings for pressure and temperature */
		t_dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_32X;
		t_dev.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
		t_dev.settings.odr_filter.odr = BMP3_ODR_25_HZ;

		/* Assign the settings which needs to be set in the sensor */
		settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
		rslt = bmp3_set_sensor_settings(settings_sel, &t_dev);

		if (rslt == BMP3_SENSOR_OK) {
			t_dev.settings.op_mode = BMP3_FORCED_MODE;
			rslt = bmp3_set_op_mode(&t_dev);
			if (rslt == BMP3_SENSOR_OK) {
				t_dev.delay_ms(100);
				sensor_comp = BMP3_PRESS | BMP3_TEMP;
				/* Temperature and Pressure data are read and stored in the bmp3_data instance */
				rslt = bmp3_get_sensor_data(sensor_comp, &data_sens, &t_dev);

			}
		}

		if (rslt == BMP3_SENSOR_OK) {
			t_dev.settings.op_mode = BMP3_SLEEP_MODE;
			rslt = bmp3_set_op_mode(&t_dev);
		}
	}

	data->val = data_sens;
	return 0;
}

static int bmp388_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bmp388_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->comp_temp has a resolution of 0.01 degC.  So
		 * 5123 equals 51.23 degC.
		 */
		val->val1 = data->val.temperature / 100;
		val->val2 = data->val.temperature % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->comp_press has a resolution of 0.01 Pa. So
		 * 9638625 = 96386.25 Pa = 96.38625 kPa
		 */
		val->val1 = data->val.pressure / 100000;
		val->val2 = data->val.pressure % 100000 * 10;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bmp388_api_funcs = {
	.sample_fetch = bmp388_sample_fetch,
	.channel_get = bmp388_channel_get,
};


static int bmp388_chip_init(struct device *dev)
{
	struct bmp388_data *data =  dev->driver_data;
	uint8_t rslt;
	struct bmp3_data data_sens = { 0 };
	uint8_t sensor_comp;

	/* Used to select the settings user needs to change */
	uint16_t settings_sel;
	struct bmp3_dev t_dev;

	t_dev.chip_id = BMP3_CHIP_ID;
	t_dev.read = &bosch_read;
	t_dev.write = &bosch_write;
	t_dev.intf = BMP3_I2C_INTF;
	t_dev.delay_ms = &bmp_delay_ms;

	/* resetting of the sensor */
	rslt = bmp3_soft_reset(&t_dev);
	if (rslt != BMP3_SENSOR_OK) {
		LOG_DBG("BMP388 Soft Reset Failed");
	}

	/* initializing Bosch Driver Code */
	rslt = bmp3_init(&t_dev);

	if (rslt == BMP3_E_COMM_FAIL || rslt == BMP3_E_DEV_NOT_FOUND) {
		rslt = BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
	}

	if (rslt == BMP3_SENSOR_OK) {
		rslt = validate_trimming_param(&t_dev);
	}

	if (rslt == BMP3_SENSOR_OK) {
		t_dev.settings.press_en = BMP3_ENABLE;
		t_dev.settings.temp_en = BMP3_ENABLE;

		/* Select the output data rate and over sampling settings for pressure and temperature */
		t_dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_32X;
		t_dev.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
		t_dev.settings.odr_filter.odr = BMP3_ODR_25_HZ;

		/* Assign the settings which needs to be set in the sensor */
		settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
		rslt = bmp3_set_sensor_settings(settings_sel, &t_dev);

		if (rslt == BMP3_SENSOR_OK) {
			t_dev.settings.op_mode = BMP3_FORCED_MODE;
			rslt = bmp3_set_op_mode(&t_dev);
			if (rslt == BMP3_SENSOR_OK) {
				t_dev.delay_ms(40);
				sensor_comp = BMP3_PRESS | BMP3_TEMP;
				/* Temperature and Pressure data are read and stored in the bmp3_data instance */
				rslt = bmp3_get_sensor_data(sensor_comp, &data_sens, &t_dev);

			}
		}
		if (rslt == BMP3_SENSOR_OK) {
			rslt = analyze_sensor_data(&data_sens);

			/* Set the power mode to sleep mode */
//    	  if (rslt == BMP3_SENSOR_OK)
//    	  {

//    	     t_dev.settings.op_mode = BMP3_SLEEP_MODE;
//    	     rslt = bmp3_set_op_mode(&t_dev);

//    	  }
		}
	}
	return 0;
}

#ifdef DT_BOSCH_BMP388_BUS_SPI
static inline int bmp388_spi_init(struct bmp388_data *data)
{
	data->spi = device_get_binding(DT_INST_0_BOSCH_BMP388_BUS_NAME);
	if (!data->spi) {
		LOG_DBG("spi device not found: %s",
			DT_INST_0_BOSCH_BMP388_BUS_NAME);
		return -EINVAL;
	}

	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
				  SPI_MODE_CPOL | SPI_MODE_CPHA;
	data->spi_cfg.frequency = DT_INST_0_BOSCH_BMP388_SPI_MAX_FREQUENCY;
	data->spi_cfg.slave = DT_INST_0_BOSCH_BMP388_BASE_ADDRESS;

	return 0;
}
#endif

int bmp388_init(struct device *dev)
{
	struct bmp388_data *data = dev->driver_data;

	bmp388_global = dev->driver_data;
#ifdef DT_BOSCH_BMP388_BUS_I2C
	data->i2c_master = device_get_binding(DT_INST_0_BOSCH_BMP388_BUS_NAME);
	if (!data->i2c_master) {
		LOG_DBG("i2c master not found: %s",
			DT_INST_0_BOSCH_BMP388_BUS_NAME);
		return -EINVAL;
	}

	data->i2c_slave_addr = DT_INST_0_BOSCH_BMP388_BASE_ADDRESS;
#elif defined DT_BOSCH_BMP388_BUS_SPI
	if (BMP388_spi_init(data) < 0) {
		LOG_DBG("spi master not found: %s",
			DT_INST_0_BOSCH_BMP388_BUS_NAME);
		return -EINVAL;
	}
#endif
	if (bmp388_chip_init(dev) < 0) {
		return -EINVAL;
	}
	bmp388_global = data;
	return 0;
}

static struct bmp388_data bmp388_data;


DEVICE_AND_API_INIT(BMP388, DT_INST_0_BOSCH_BMP388_LABEL, bmp388_init, &bmp388_data,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &bmp388_api_funcs);
