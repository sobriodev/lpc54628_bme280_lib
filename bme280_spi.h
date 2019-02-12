/*
 * bme280_spi.h
 *
 *  Created on: 11 lut 2019
 *      Author: sobriodev
 */

#ifndef BME280_SPI_H_
#define BME280_SPI_H_

#define PINS_CNT 4 /* SCK, MOSI, MISO, SSEL */

#define SCK_PORT 3
#define SCK_PIN 20
#define MOSI_PORT 3
#define MOSI_PIN 21
#define MISO_PORT 3
#define MISO_PIN 22
#define SSEL_PORT 2
#define SSEL_PIN 0

#define IOCON_DUMMY_DATA 0

#define BME280_ID 0
#define BME280_DUMMY_REG_ADDR 0

#define BME280_SUCCESS 0
#define BME280_FAILURE -1

#define SPI_BAUDRATE 6000000 /* 6MHz */

/* -------------------------------------------------------------------------------------- */
/* ------------------------ Functions required by BME280 library ------------------------ */
/* -------------------------------------------------------------------------------------- */

/**
 * @brief           Read data from bme280 sensor
 * @param dev_id    Device id. Must be zero
 * @param reg_addr	Register address
 * @param reg_data  Output data buffer base address
 * @param len       Data length
 * @return          Zero for success, non-zero for failure
 */
int8_t bme280_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief 			Write data to bme280 sensor
 * @param dev_id 	Device id. Must be zero
 * @param reg_addr	Register address
 * @param reg_data 	Input data buffer base address
 * @param len 		Data length
 * @return 			Zero for success, non-zero for failure
 */
int8_t bme280_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief 			Delay function
 * @param period	Time period in milliseconds
 */
void bme280_delay(uint32_t period);

/* -------------------------------------------------------------------------------------- */
/* ---------------------------------- Other functions ----------------------------------- */
/* -------------------------------------------------------------------------------------- */

/**
 * @brief Initialize SPI registers and set up needed pins
 */
void bme280_spi_init(void);

/**
 * @brief 			Continuously read sensor data
 * @param dev		Bme280 configuration struct base address
 * @return 			Zero for success, non-zero for failure
 */
int8_t bme280_stream_sensor_data_normal_mode(struct bme280_dev *dev);


#endif /* BME280_SPI_H_ */
