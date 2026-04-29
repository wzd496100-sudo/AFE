/*
 * BQ76940 I2C communication helpers.
 *
 * This module ports the open-source transaction style:
 * - single-byte register read/write
 * - optional SMBus PEC (CRC-8, polynomial 0x07)
 */

#ifndef BQ76940_I2C_H
#define BQ76940_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

#define BQ76940_I2C_ADDR_7BIT      (0x08u)
#define BQ76940_I2C_TIMEOUT_MS     (20u)
#define BQ76940_I2C_RETRY_COUNT    (3u)

/*
 * I2C switch select level for AFE path.
 * This hardware uses active-low switch enable on PB13.
 */
#define BQ76940_I2C_SW_AFE_LEVEL   GPIO_PIN_RESET

/* Frequently used BQ76940 registers during bring-up. */
#define BQ76940_REG_SYS_STAT       (0x00u)
#define BQ76940_REG_SYS_CTRL1      (0x04u)
#define BQ76940_REG_SYS_CTRL2      (0x05u)

typedef struct
{
	uint32_t last_hal_status;
	uint32_t last_api_status;
	uint32_t last_hal_i2c_error;
	uint8_t last_reg_addr;
	uint8_t last_rx_data;
	uint8_t last_rx_pec;
	uint8_t last_calc_pec;
} BQ76940_I2C_Debug_t;

extern volatile BQ76940_I2C_Debug_t g_bq76940_i2c_dbg;

void BQ76940_I2C_InitComms(void);
void BQ76940_I2C_SelectAfeBus(void);

HAL_StatusTypeDef I2CReadRegisterByte(uint8_t devAddr7, uint8_t regAddr, uint8_t *data);
HAL_StatusTypeDef I2CWriteRegisterByte(uint8_t devAddr7, uint8_t regAddr, uint8_t data);
HAL_StatusTypeDef I2CReadRegisterByteWithCRC(uint8_t devAddr7, uint8_t regAddr, uint8_t *data);
HAL_StatusTypeDef I2CWriteRegisterByteWithCRC(uint8_t devAddr7, uint8_t regAddr, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* BQ76940_I2C_H */
