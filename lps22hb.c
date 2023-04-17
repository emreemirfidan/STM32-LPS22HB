/*
 * lps22hb.c
 *
 *  Created on: Mar 13, 2023
 *      Author: Emre Emir Fidan
 */

#include "lps22hb.h"
#include "math.h"

#define SPI3WIRE

#define CHECK_STATUS

#define MA_FILTER_COUNT 20

#define CS_PIN_PORT		GPIOA
#define CS_PIN			GPIO_PIN_3


LPS22HB_STATUS lps22hb_writeReg(lps22hb_t *lps22hb, uint8_t address, uint8_t reg) {
	uint8_t ret;
	uint8_t buffer[2] = {address, reg};
	HAL_GPIO_WritePin(CS_PIN_PORT, CS_PIN, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(lps22hb->hspi, buffer, 2, 100);
	HAL_GPIO_WritePin(CS_PIN_PORT, CS_PIN, GPIO_PIN_SET);

	if (ret != HAL_OK)
		return LPS22HB_ERROR;

	return LPS22HB_OK;
}

LPS22HB_STATUS lps22hb_readReg(lps22hb_t *lps22hb, uint8_t address, uint8_t *data, uint8_t size) {
	uint8_t ret;

	address |= (0x80U);

	HAL_GPIO_WritePin(CS_PIN_PORT, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lps22hb->hspi, &address, 1, 100);
	ret = HAL_SPI_Receive(lps22hb->hspi, data, size, 100);
	HAL_GPIO_WritePin(CS_PIN_PORT, CS_PIN, GPIO_PIN_SET);

	if (ret != HAL_OK)
		return LPS22HB_ERROR;

	return LPS22HB_OK;
}

LPS22HB_STATUS lps22hb_enableSPI(lps22hb_t *lps22hb, SPI_HandleTypeDef *hspi) {
	lps22hb->hspi = hspi;
	HAL_GPIO_WritePin(CS_PIN_PORT, CS_PIN, GPIO_PIN_SET);
	return LPS22HB_OK;
}

LPS22HB_STATUS lps22hb_init(lps22hb_t *lps22hb, lps22hb_odr_t odr, uint8_t lpfp_en, uint8_t lpfp_cfg) {
	uint8_t ret = 0;

	lps22hb->status = 0;

	/* CTRL_REG1 */
	ret = lps22hb_writeReg(lps22hb, LPS22HB_CTRL_REG1, (odr << 4) | (lpfp_en << 3) | (lpfp_cfg << 2) | (1));

	/* Read CHIP ID (0xB1) */
	ret = lps22hb_readReg(lps22hb, LPS22HB_WHO_AM_I, &lps22hb->chip_id, 1);

	if (lps22hb->chip_id != LPS22HB_ID || ret != LPS22HB_OK) {
		lps22hb->status = LPS22HB_ERROR;
		return LPS22HB_ERROR;
	}

	/* Calculate Bandwith */
	switch(odr) {
	case 1:
		lps22hb->bandwith = 1.0f;
		break;
	case 2:
		lps22hb->bandwith = 10.0f;
		break;
	case 3:
		lps22hb->bandwith = 25.0f;
		break;
	case 4:
		lps22hb->bandwith = 50.0f;
		break;
	case 5:
		lps22hb->bandwith = 75.0f;
		break;
	default:
		lps22hb->bandwith = 0.0f;
		break;
	}

	if (lpfp_en == 0) {
		lps22hb->bandwith /= 2.0f;
	}else if (lpfp_en == 1 && lpfp_cfg == 0) {
		lps22hb->bandwith /= 9.0f;
	}else if (lpfp_en == 1 && lpfp_cfg == 1) {
		lps22hb->bandwith /= 20.0f;
	}

	lps22hb->compute_time = (uint32_t)(1000.0f / lps22hb->bandwith);

	/* Set Altitude Offset Zero */
	lps22hb->altitude_offset = 0;


	lps22hb->status = LPS22HB_OK;
	return LPS22HB_OK;
}

LPS22HB_STATUS lps22hb_readPressure(lps22hb_t *lps22hb) {
	if (lps22hb->status == LPS22HB_OK) {
		uint8_t ret;

		uint8_t buffer[3];

		ret = lps22hb_readReg(lps22hb, LPS22HB_PRESS_OUT_XL, buffer, 3);

		if (ret != LPS22HB_OK)
			return LPS22HB_ERROR;

		lps22hb->raw_pressure = ((buffer[2] << 16) | (buffer[1] << 8) | buffer[0]);
		lps22hb->pressure = (float)lps22hb->raw_pressure / 4096.0f;

		return LPS22HB_OK;
	}
	return LPS22HB_NCP;
}

LPS22HB_STATUS lps22hb_readTemperature(lps22hb_t *lps22hb) {
	if (lps22hb->status == LPS22HB_OK) {
		uint8_t ret;

		uint8_t buffer[2];

		ret = lps22hb_readReg(lps22hb, LPS22HB_TEMP_OUT_L, buffer, 2);

		if (ret != LPS22HB_OK)
			return LPS22HB_ERROR;

		lps22hb->temperature = (float)((int16_t)((buffer[1] << 8) | buffer[0]));
		lps22hb->temperature /= 100.0f;

		return LPS22HB_OK;
	}
	return LPS22HB_NCP;
}

LPS22HB_STATUS lps22hb_readAltitude(lps22hb_t *lps22hb) {
	if (lps22hb->status == LPS22HB_OK) {
		// https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
		lps22hb->altitude = (1 - (pow((lps22hb->pressure / 1013.25f), 0.190284f))) * 44307.69f;
		lps22hb->altitude -= lps22hb->altitude_offset;

		return LPS22HB_OK;
	}
	return LPS22HB_NCP;
}

LPS22HB_STATUS lps22hb_setAltitudeOffset(lps22hb_t *lps22hb, float altitude) {
	lps22hb->altitude_offset = altitude;
	return LPS22HB_OK;
}

LPS22HB_STATUS lps22hb_setAltitudeZero(lps22hb_t *lps22hb) {
	float alt_total;
	for (uint8_t i = 0; i < MA_FILTER_COUNT; i++) {
		lps22hb_readPressure(lps22hb);
		lps22hb_readAltitude(lps22hb);

		alt_total += lps22hb->altitude;

		HAL_Delay(lps22hb->compute_time);
	}
	alt_total /= MA_FILTER_COUNT;
	lps22hb_setAltitudeOffset(lps22hb, alt_total);
	return LPS22HB_OK;
}
