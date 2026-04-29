#include "bq76940_i2c.h"

volatile BQ76940_I2C_Debug_t g_bq76940_i2c_dbg = {0u};

static uint8_t BQ76940_Crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0u;
  uint8_t i;
  uint8_t bit;

  for (i = 0u; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0u; bit < 8u; bit++)
    {
      if ((crc & 0x80u) != 0u)
      {
        crc = (uint8_t)((crc << 1u) ^ 0x07u);
      }
      else
      {
        crc <<= 1u;
      }
    }
  }

  return crc;
}

static HAL_StatusTypeDef BQ76940_MemReadRetry(uint8_t devAddr7, uint8_t regAddr, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t retry;

  for (retry = 0u; retry < BQ76940_I2C_RETRY_COUNT; retry++)
  {
    status = HAL_I2C_Mem_Read(&hi2c2,
                              (uint16_t)(devAddr7 << 1u),
                              regAddr,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              len,
                              BQ76940_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }
  }

  return status;
}

static HAL_StatusTypeDef BQ76940_MemWriteRetry(uint8_t devAddr7, uint8_t regAddr, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t retry;

  for (retry = 0u; retry < BQ76940_I2C_RETRY_COUNT; retry++)
  {
    status = HAL_I2C_Mem_Write(&hi2c2,
                               (uint16_t)(devAddr7 << 1u),
                               regAddr,
                               I2C_MEMADD_SIZE_8BIT,
                               data,
                               len,
                               BQ76940_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }
  }

  return status;
}

static HAL_StatusTypeDef BQ76940_MasterTxRetry(uint8_t devAddr7, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t retry;

  for (retry = 0u; retry < BQ76940_I2C_RETRY_COUNT; retry++)
  {
    status = HAL_I2C_Master_Transmit(&hi2c2,
                                     (uint16_t)(devAddr7 << 1u),
                                     data,
                                     len,
                                     BQ76940_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      break;
    }
  }

  return status;
}

void BQ76940_I2C_SelectAfeBus(void)
{
#if defined(I2C_SW_GPIO_Port) && defined(I2C_SW_Pin)
  HAL_GPIO_WritePin(I2C_SW_GPIO_Port, I2C_SW_Pin, BQ76940_I2C_SW_AFE_LEVEL);
  for (volatile uint32_t i = 0u; i < 200u; i++)
  {
    __NOP();
  }
#endif
}

void BQ76940_I2C_InitComms(void)
{
#if defined(WAKE_AFE_GPIO_Port) && defined(WAKE_AFE_Pin)
  HAL_GPIO_WritePin(WAKE_AFE_GPIO_Port, WAKE_AFE_Pin, GPIO_PIN_SET);
#endif
  BQ76940_I2C_SelectAfeBus();
}

HAL_StatusTypeDef I2CReadRegisterByte(uint8_t devAddr7, uint8_t regAddr, uint8_t *data)
{
  g_bq76940_i2c_dbg.last_reg_addr = regAddr;

  if (data == NULL)
  {
    g_bq76940_i2c_dbg.last_api_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  BQ76940_I2C_SelectAfeBus();
  g_bq76940_i2c_dbg.last_hal_status = (uint32_t)BQ76940_MemReadRetry(devAddr7, regAddr, data, 1u);
  g_bq76940_i2c_dbg.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);

  if ((HAL_StatusTypeDef)g_bq76940_i2c_dbg.last_hal_status == HAL_OK)
  {
    g_bq76940_i2c_dbg.last_rx_data = *data;
  }

  g_bq76940_i2c_dbg.last_api_status = g_bq76940_i2c_dbg.last_hal_status;
  return (HAL_StatusTypeDef)g_bq76940_i2c_dbg.last_api_status;
}

HAL_StatusTypeDef I2CWriteRegisterByte(uint8_t devAddr7, uint8_t regAddr, uint8_t data)
{
  HAL_StatusTypeDef status;

  g_bq76940_i2c_dbg.last_reg_addr = regAddr;
  BQ76940_I2C_SelectAfeBus();
  status = BQ76940_MemWriteRetry(devAddr7, regAddr, &data, 1u);

  g_bq76940_i2c_dbg.last_hal_status = (uint32_t)status;
  g_bq76940_i2c_dbg.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);
  g_bq76940_i2c_dbg.last_api_status = (uint32_t)status;

  return status;
}

HAL_StatusTypeDef I2CReadRegisterByteWithCRC(uint8_t devAddr7, uint8_t regAddr, uint8_t *data)
{
  HAL_StatusTypeDef status;
  uint8_t rxBuf[2];
  uint8_t crcInput[2];

  g_bq76940_i2c_dbg.last_reg_addr = regAddr;

  if (data == NULL)
  {
    g_bq76940_i2c_dbg.last_api_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  BQ76940_I2C_SelectAfeBus();

  status = BQ76940_MemReadRetry(devAddr7, regAddr, rxBuf, 2u);
  g_bq76940_i2c_dbg.last_hal_status = (uint32_t)status;
  g_bq76940_i2c_dbg.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);
  if (status != HAL_OK)
  {
    g_bq76940_i2c_dbg.last_api_status = (uint32_t)status;
    return status;
  }

  g_bq76940_i2c_dbg.last_rx_data = rxBuf[0];
  g_bq76940_i2c_dbg.last_rx_pec = rxBuf[1];

  /*
   * For read transactions on CRC-enabled bq769x0 devices,
   * the first PEC is calculated over <SlaveAddress+R><Data1>.
   */
  crcInput[0] = (uint8_t)((devAddr7 << 1u) | 0x01u);
  crcInput[1] = rxBuf[0];
  g_bq76940_i2c_dbg.last_calc_pec = BQ76940_Crc8(crcInput, 2u);

  if (g_bq76940_i2c_dbg.last_calc_pec != rxBuf[1])
  {
    g_bq76940_i2c_dbg.last_api_status = (uint32_t)HAL_ERROR;
    return HAL_ERROR;
  }

  *data = rxBuf[0];
  g_bq76940_i2c_dbg.last_api_status = (uint32_t)HAL_OK;
  return HAL_OK;
}

HAL_StatusTypeDef I2CWriteRegisterByteWithCRC(uint8_t devAddr7, uint8_t regAddr, uint8_t data)
{
  HAL_StatusTypeDef status;
  uint8_t txBuf[3];
  uint8_t crcInput[3];

  g_bq76940_i2c_dbg.last_reg_addr = regAddr;
  BQ76940_I2C_SelectAfeBus();

  txBuf[0] = regAddr;
  txBuf[1] = data;

  crcInput[0] = (uint8_t)(devAddr7 << 1u);
  crcInput[1] = regAddr;
  crcInput[2] = data;
  txBuf[2] = BQ76940_Crc8(crcInput, 3u);
  g_bq76940_i2c_dbg.last_calc_pec = txBuf[2];

  status = BQ76940_MasterTxRetry(devAddr7, txBuf, 3u);
  g_bq76940_i2c_dbg.last_hal_status = (uint32_t)status;
  g_bq76940_i2c_dbg.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);
  g_bq76940_i2c_dbg.last_api_status = (uint32_t)status;

  return status;
}
