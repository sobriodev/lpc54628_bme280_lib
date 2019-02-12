/*
 * bme280_spi.c
 *
 *  Created on: 11 lut 2019
 *      Author: sobriodev
 */

#include "LPC54628.h"
#include "bme280.h"
#include "bme280_spi.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

/* ------------------------------------------------------------------------------------------- */
/* ----------------------------- Private functions and variables ----------------------------- */
/* ------------------------------------------------------------------------------------------- */

/**
 * @brief Remaining delay time (in milliseconds)
 */
static volatile uint32_t ticks = 0;

/**
 * @brief Systick ISR decrementing remaining delay time
 */
void SysTick_Handler(void)
{
	if (ticks) {
		ticks--;
	}
}

/**
 * @brief 		Calculate the number of data chunks (each chunk contains 8 byte frames) to prevent data overwrite.
 * 				This is because receive FIFO contains 8 data entries
 * @param len	Data length
 * @return 		Number of data chunks
 */
static uint16_t calculate_chunks(uint16_t len)
{
    uint16_t chunks = 1;
    bool add = false;
    for (int i = 1; i <= len; i++) {
        chunks += add;
        add = ((i % 8) == 0);
    }
    return chunks;
}

/**
 * @brief Set SSEL pin to logic zero
 */
static inline void __ssel_set(void)
{
	GPIO_PinWrite(GPIO, SSEL_PORT, SSEL_PIN, false);
}

/**
 * @brief Set SSEL pin to logic one
 */
static inline void __ssel_reset(void)
{
	GPIO_PinWrite(GPIO, SSEL_PORT, SSEL_PIN, true);
}

/**
 * @brief Flush RX FIFO
 */
static inline void __flush_rx_fifo(void) {
	while (SPI_GetStatusFlags(SPI9) & kSPI_RxNotEmptyFlag) {
		SPI_ReadData(SPI9);
	}
}

/**
 * @brief Wait until SPI controller is idle
 */
static inline void __wait_idle(void)
{
	while (!(SPI9->STAT & SPI_STAT_MSTIDLE_MASK)) {
		__asm("NOP");
	}
}

/**
 * @brief Wait until TX FIFO is not full
 */
static inline void __wait_tx_not_full(void)
{
	while (!(SPI_GetStatusFlags(SPI9) & kSPI_TxNotFullFlag)) {
		__asm("NOP");
	}
}

/**
 * @brief Wait until TX FIFO is empty
 */
static inline void __wait_tx_empty(void)
{
	while (!(SPI_GetStatusFlags(SPI9) & kSPI_TxEmptyFlag)) {
		__asm("NOP");
	}
}

/* ------------------------------------------------------------------------------------------- */
/* -------------------------------------- API functions -------------------------------------- */
/* ------------------------------------------------------------------------------------------- */

void print_sensor_data(struct bme280_data *comp_data)
{
	printf("%d, %d, %d\r\n", (int) comp_data->temperature, (int) comp_data->pressure, (int) comp_data->humidity);
}

void bme280_spi_init(void)
{
    CLOCK_EnableClock(kCLOCK_Iocon); /* Ensure IOCON clock is enabled */

    /* SPI pinmux */
    iocon_group_t pinmux[PINS_CNT] = {
    		{ SCK_PORT, SCK_PIN, IOCON_DUMMY_DATA, IOCON_FUNC1 | IOCON_DIGITAL_EN },   /* SCK */
			{ MOSI_PORT, MOSI_PIN, IOCON_DUMMY_DATA, IOCON_FUNC1 | IOCON_DIGITAL_EN }, /* MOSI */
			{ MISO_PORT, MISO_PIN, IOCON_DUMMY_DATA, IOCON_FUNC1 | IOCON_DIGITAL_EN},  /* MISO */
			{ SSEL_PORT, SSEL_PIN, IOCON_DUMMY_DATA, IOCON_FUNC0 | IOCON_DIGITAL_EN  } /* SSEL as GPIO pin */
    };
    IOCON_SetPinMuxing(IOCON, pinmux, PINS_CNT);

    /* SSEL GPIO config */
    gpio_pin_config_t sselconfig = { kGPIO_DigitalOutput, true };
    GPIO_PortInit(GPIO, SSEL_PORT); /* No need to enable GPIO clock. This function initializes it */
    GPIO_PinInit(GPIO, SSEL_PORT, SSEL_PIN, &sselconfig);

    /* SPI config */
    spi_master_config_t master_config;
    SPI_MasterGetDefaultConfig(&master_config);
    master_config.baudRate_Bps = SPI_BAUDRATE;

    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM9);
    uint32_t source_clock = CLOCK_GetFreq(kCLOCK_Flexcomm9);
    SPI_MasterInit(SPI9, &master_config, source_clock);
}

int8_t bme280_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/* Device ID must be zero */
	if (dev_id != BME280_ID) {
		return BME280_FAILURE;
	}

	uint16_t chunks = calculate_chunks(len);

	__ssel_set();

	/* Send register address */
	__wait_tx_not_full();
	SPI_WriteData(SPI9, reg_addr, kSPI_FrameAssert | SPI_FIFOWR_RXIGNORE_MASK); /* Set RXIGNORE flag to skip slave data */
	__wait_idle();

	__flush_rx_fifo(); /* Ensure RX FIFO is empty to prevent transfer halting */

	for (int i = 1; i <= chunks; i++) {
		uint8_t startOffset = (i-1)*8;
		uint8_t stopOffset;
		if (i == chunks) {
			stopOffset = len-1;
		} else {
			stopOffset = i*8-1;
		}

		 /* Send dummy register address to read sensor data */
		__wait_tx_empty();
		for (int i = startOffset; i <= stopOffset; i++) {
			SPI_WriteData(SPI9, BME280_DUMMY_REG_ADDR, kSPI_FrameAssert);
		}

		/* Wait for SPI transfer finish and collect incoming data in output register */
		__wait_idle();
		for (int i = startOffset; i <= stopOffset; i++) {
			reg_data[i] = SPI_ReadData(SPI9);
		}

	}

	__ssel_reset();

	return BME280_SUCCESS;
}

int8_t bme280_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	/* Device ID must be zero */
	if (dev_id != BME280_ID) {
		return BME280_FAILURE;
	}

	__ssel_set();

	/* Send register address */
	__wait_tx_not_full();
	SPI_WriteData(SPI9, reg_addr, kSPI_FrameAssert | SPI_FIFOWR_RXIGNORE_MASK); /* Set RXIGNORE flag to skip slave data */

	/* Send data */
	for (int i = 0; i < len; i++) {
		__wait_tx_not_full();
		SPI_WriteData(SPI9, reg_data[i], kSPI_FrameAssert | SPI_FIFOWR_RXIGNORE_MASK); /* Set RXIGNORE flag to skip slave data */
	}

	__wait_idle();

	__ssel_reset();

	return BME280_SUCCESS;
}

void bme280_delay(uint32_t period)
{
	ticks = period;
	while (ticks) {
		__asm("NOP");
	}
}

int8_t bme280_stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	printf("Temperature, Pressure, Humidity\r\n");
	while (1) {
		/* Delay while the sensor completes a measurement */
		dev->delay_ms(70);
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
		print_sensor_data(&comp_data);
	}

	return rslt;
}
