/*
 * lps22hb.h
 *
 *  Created on: Mar 13, 2023
 *      Author: Emre Emir Fidan
 */

#ifndef INC_LPS22HB_H_
#define INC_LPS22HB_H_

#include "stm32l4xx_hal.h"

#define LPS22HB_ID             0xB1U

#define LPS22HB_INTERRUPT_CFG  0x0BU
#define LPS22HB_THS_P_L        0x0CU
#define LPS22HB_THS_P_H        0x0DU
#define LPS22HB_WHO_AM_I       0x0FU
#define LPS22HB_CTRL_REG1      0x10U
#define LPS22HB_CTRL_REG2      0x11U
#define LPS22HB_CTRL_REG3      0x12U
#define LPS22HB_FIFO_CTRL      0x14U
#define LPS22HB_REF_P_XL       0x15U
#define LPS22HB_REF_P_L        0x16U
#define LPS22HB_REF_P_H        0x17U
#define LPS22HB_RPDS_L         0x18U
#define LPS22HB_RPDS_H         0x19U
#define LPS22HB_RES_CONF       0x1AU
#define LPS22HB_INT_SOURCE     0x25U
#define LPS22HB_FIFO_STATUS    0x26U
#define LPS22HB_STATUS_REG     0x27U
#define LPS22HB_PRESS_OUT_XL   0x28U
#define LPS22HB_PRESS_OUT_L    0x29U
#define LPS22HB_PRESS_OUT_H    0x2AU
#define LPS22HB_TEMP_OUT_L     0x2BU
#define LPS22HB_TEMP_OUT_H     0x2CU
#define LPS22HB_LPFP_RES       0x33U


typedef enum {
	LPS22HB_ERROR = 0,
	LPS22HB_OK,
	LPS22HB_BUSY,
	LPS22HB_NCP // Not Configurated Properly
}LPS22HB_STATUS;

typedef enum
{
  LPS22HB_POWER_DOWN = 0,
  LPS22HB_ODR_1_Hz,
  LPS22HB_ODR_10_Hz,
  LPS22HB_ODR_25_Hz,
  LPS22HB_ODR_50_Hz,
  LPS22HB_ODR_75_Hz
}lps22hb_odr_t;

typedef struct {
	SPI_HandleTypeDef *hspi;

	uint8_t chip_id;

	uint8_t status;
	float bandwith;
	uint32_t compute_time;

	float temperature;

	int32_t raw_pressure:24;
	float pressure;

	float altitude;
	float altitude_offset;
}lps22hb_t;

LPS22HB_STATUS lps22hb_writeReg(lps22hb_t *lps22hb, uint8_t address, uint8_t reg);

LPS22HB_STATUS lps22hb_readReg(lps22hb_t *lps22hb, uint8_t address, uint8_t *data, uint8_t size);

LPS22HB_STATUS lps22hb_enableSPI(lps22hb_t *lps22hb, SPI_HandleTypeDef *hspi);

LPS22HB_STATUS lps22hb_init(lps22hb_t *lps22hb, lps22hb_odr_t odr, uint8_t lpfp_en, uint8_t lpfp_cfg);

LPS22HB_STATUS lps22hb_readPressure(lps22hb_t *lps22hb);

LPS22HB_STATUS lps22hb_readTemperature(lps22hb_t *lps22hb);

LPS22HB_STATUS lps22hb_readAltitude(lps22hb_t *lps22hb);

LPS22HB_STATUS lps22hb_setAltitudeOffset(lps22hb_t *lps22hb, float altitude);

LPS22HB_STATUS lps22hb_setAltitudeZero(lps22hb_t *lps22hb);

#endif /* INC_LPS22HB_H_ */
