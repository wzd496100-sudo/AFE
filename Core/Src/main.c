/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bq76940_i2c.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BQ76940_CELL_COUNT        (15u)
#define BQ76940_ACTIVE_CELL_COUNT (13u)
static void BQ76200_EnableChargePump(void);
#define BQ76940_REG_FIRST         (0x00u)
#define BQ76940_REG_LAST          (0x33u)
#define BQ76940_REG_COUNT         (BQ76940_REG_LAST - BQ76940_REG_FIRST + 1u)
#define BQ76940_REG_CELLBAL1      (0x01u)
#define BQ76940_REG_CELLBAL2      (0x02u)
#define BQ76940_REG_CELLBAL3      (0x03u)
#define BQ76940_REG_CC_CFG        (0x0Bu)
#define BQ76940_REG_ADCGAIN1      (0x50u)
#define BQ76940_REG_ADCOFFSET     (0x51u)
#define BQ76940_REG_ADCGAIN2      (0x59u)
#define BQ76940_SYS_CTRL1_ADC_EN  (0x10u)
#define BQ76940_SYS_CTRL1_TEMP_SEL (0x08u)
#define BQ76940_DEFAULT_ADC_GAIN_UV (382u)
#define BQ76940_RSNS_MOHM         (1.0f)

/* Physical pair indices (0-based) to map into logical cells for 13-cell config.
   These are the physical cell pairs to read (pair 0 = VC1-VC0, pair 1 = VC2-VC1, ...).
   For the board's 13-cell wiring we skip physical pairs 8 and 13 (0-based),
   so mapping becomes: 0,1,2,3,4,5,6,7,9,10,11,12,14.
*/
static const uint8_t s_phys_pair_for_cell13[BQ76940_ACTIVE_CELL_COUNT] = {
  0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 9u, 10u, 11u, 12u, 14u
};
#define BQ76940_TS_BIAS_MV        (3300.0f)
#define BQ76940_TS_PULLUP_OHM     (10000.0f)
#define BQ76940_TS_NTC_R0_OHM     (10000.0f)
#define BQ76940_TS_NTC_BETA       (3435.0f)

/* Set to 1 to enable internal CAN loopback test (no transceiver required) */
#define BQ76940_CAN_LOOPBACK_TEST  (1u)

/* LED pack voltage thresholds for 13-cell configuration:
   Min pack: 2.5V * 13 = 32500 mV
   Max pack: 4.2V * 13 = 54600 mV
   Range: 22100 mV */
#define BQ76940_PACK_MV_MIN       (32500u)
#define BQ76940_PACK_MV_MAX       (54600u)
#define BQ76940_PACK_MV_25PCT     (38025u)  /* 32500 + (54600-32500)*0.25 */
#define BQ76940_PACK_MV_50PCT     (43550u)  /* 32500 + (54600-32500)*0.50 */
#define BQ76940_PACK_MV_75PCT     (49075u)  /* 32500 + (54600-32500)*0.75 */
#define BQ76940_SYS_STAT_OCD      (0x01u)
#define BQ76940_SYS_STAT_SCD      (0x02u)
#define BQ76940_SYS_STAT_OV       (0x04u)
#define BQ76940_SYS_STAT_UV       (0x08u)
#define BQ76940_SYS_STAT_OVRD_ALERT (0x10u)
#define BQ76940_SYS_STAT_DEVICE_XREADY (0x20u)
#define BQ76940_SYS_STAT_CC_READY (0x80u)
#define BQ76940_SYS_STAT_FAULT_MASK (BQ76940_SYS_STAT_OCD | \
                                      BQ76940_SYS_STAT_SCD | \
                                      BQ76940_SYS_STAT_OV | \
                                      BQ76940_SYS_STAT_UV | \
                                      BQ76940_SYS_STAT_DEVICE_XREADY)
#define BQ76940_SYS_STAT_CLEAR_NORMAL_MASK (BQ76940_SYS_STAT_CC_READY | \
                                            BQ76940_SYS_STAT_OVRD_ALERT)
#define BQ76940_RECOVER_BLOCK_NONE         (0u)
#define BQ76940_RECOVER_BLOCK_COMM         (1u)
#define BQ76940_RECOVER_BLOCK_FAULT        (2u)
#define BQ76940_RECOVER_BLOCK_OVRD_ALERT   (3u)
#define BQ76940_RECOVER_BLOCK_DISABLED     (4u)
#define BQ76940_SYS_CTRL2_CHG_ON  (0x01u)
#define BQ76940_SYS_CTRL2_DSG_ON  (0x02u)
#define BQ76940_SYS_CTRL2_CC_ONESHOT (0x20u)
#define BQ76940_SYS_CTRL2_CC_EN   (0x40u)
#define BQ76940_BALANCE_TEST_ENABLE (0u)
#define BQ76940_BALANCE_TEST_CELL   (1u)
#define BQ76940_BALANCE_ENABLE      (1u)
#define BQ76940_BALANCE_MIN_CELL_MV (3300u)
#define BQ76940_BALANCE_START_DELTA_MV (30u)
#define BQ76940_BALANCE_STOP_DELTA_MV  (15u)
#define BQ76940_BALANCE_DEFAULT_TIMEOUT_MS (30000u)
#define BQ76940_BALANCE_COOLDOWN_MS (5000u)
#define BQ76940_CAN_TX_PERIOD_MS    (500u)
#define BQ76940_CAN_ID_STATUS       (0x500u)
#define BQ76940_CAN_ID_CELLS_1_4    (0x501u)
#define BQ76940_CAN_ID_CELLS_5_8    (0x502u)
#define BQ76940_CAN_ID_CELLS_9_12   (0x503u)
#define BQ76940_CAN_ID_CELLS_13     (0x504u)
#define BQ76940_DEBUG_HISTORY_COUNT (16u)
#define BQ76940_STATUS_LED_PERIOD_MS (200u)
#define BQ76940_LED_SERVICE_STEP_MS  (1u)
#define BQ76940_LED_BREATH_PERIOD_MS (2400u)
#define BQ76940_LED_PWM_PERIOD_MS    (20u)
#define BQ76940_LED_PWM_STEPS        (20u)
#define BQ76940_LED_MASK_1           (0x01u)
#define BQ76940_LED_MASK_2           (0x02u)
#define BQ76940_LED_MASK_3           (0x04u)
#define BQ76940_LED_MASK_4           (0x08u)

typedef struct
{
  uint8_t pause_loop;
  uint8_t enable_auto_dsg_recover;
  uint8_t auto_clear_cc_ready;
  uint8_t manual_clear_sys_stat;
  uint8_t manual_enable_dsg;
  uint8_t clear_recover_latch;
  uint8_t balance_cell;
  uint8_t balance_start;
  uint8_t balance_stop;
  uint8_t balance_active_cell;
  uint8_t balance_test_started;
  uint8_t recover_latched_off;
  uint8_t balance_auto_enable;
  uint8_t balance_candidate_cell;
  uint16_t balance_cell_delta_mv;
  uint16_t balance_cell_min_mv;
  uint16_t balance_cell_max_mv;
  uint8_t last_recover_block_reason;
  uint8_t last_recover_sys_stat;
  uint8_t last_recover_sys_ctrl2;
  uint32_t pause_enter_count;
  uint32_t manual_clear_count;
  uint32_t manual_enable_dsg_count;
  uint32_t recover_latch_count;
  uint32_t balance_timeout_ms;
  uint32_t balance_started_tick_ms;
  uint32_t balance_elapsed_ms;
  uint32_t balance_start_count;
  uint32_t balance_stop_count;
  uint32_t balance_last_stop_tick_ms;
  uint32_t can_tx_count;
  uint32_t can_tx_fail_count;
  uint32_t last_can_tx_status;
  uint32_t can_rx_count;
  uint32_t last_can_rx_id;
  uint8_t last_can_rx_dlc;
  uint8_t last_can_rx_data[8];
} BQ76940_Control_t;

typedef struct
{
  uint32_t tick_ms;
  uint32_t pack_mv;
  uint32_t last_hal_i2c_error;
  uint8_t sys_stat;
  uint8_t sys_ctrl2;
  uint8_t alert_afe_pin_level;
  uint8_t alert_gauge_pin_level;
  uint8_t alert_afe_asserted_low;
  uint8_t alert_gauge_asserted_low;
  uint8_t alert_afe_asserted_high;
  uint8_t alert_gauge_asserted_high;
  uint8_t comm_error;
  uint8_t using_plain_fallback;
  uint8_t fault_mask;
  uint8_t ocd;
  uint8_t scd;
  uint8_t ov;
  uint8_t uv;
  uint8_t ovrd_alert;
  uint8_t device_xready;
  uint8_t cc_ready;
  uint8_t chg_on;
  uint8_t dsg_on;
  uint8_t cc_oneshot;
  uint8_t cc_en;
  uint8_t cellbal1;
  uint8_t cellbal2;
  uint8_t cellbal3;
  uint8_t balance_active_cell;
  uint8_t balance_candidate_cell;
  uint16_t balance_cell_delta_mv;
  uint32_t fet_recover_count;
  uint8_t recover_block_reason;
  uint8_t led_bar_mask;
  uint8_t led_breath_duty;
} BQ76940_DebugSnapshot_t;

typedef struct
{
  BQ76940_DebugSnapshot_t now;
  BQ76940_DebugSnapshot_t history[BQ76940_DEBUG_HISTORY_COUNT];
  uint8_t history_next;
  uint8_t history_count;
} BQ76940_Debug_t;

typedef struct
{
  uint32_t loop_count;
  uint32_t last_tick_ms;
  uint32_t crc_read_ok_count;
  uint32_t crc_read_fail_count;
  uint32_t crc_mismatch_count;
  uint32_t plain_fallback_ok_count;
  uint32_t last_crc_read_status;
  uint32_t last_plain_read_status;
  uint32_t last_hal_i2c_error;
  uint8_t sys_stat_crc;
  uint8_t sys_stat_plain;
  uint8_t sys_stat_effective;
  uint8_t using_plain_fallback;
  uint8_t led1_is_blinking;
  uint8_t dsg_on;
  uint8_t led_bar_mask;
  uint8_t led_breath_duty;
  uint32_t fet_recover_count;
  uint8_t alert_afe_level;
  uint8_t alert_gauge_level;
  uint8_t i2c_sw_level;
  uint8_t led1_level;
  uint8_t reg_value[BQ76940_REG_COUNT];
  uint8_t reg_source[BQ76940_REG_COUNT];
  uint8_t adc_gain1_raw;
  uint8_t adc_gain2_raw;
  uint8_t adc_offset_raw;
  uint8_t calibration_valid;
  uint16_t adc_gain_uv_per_lsb;
  int16_t adc_offset_mv;
  uint16_t phys_cell_adc_raw[BQ76940_CELL_COUNT];
  uint16_t phys_cell_mv[BQ76940_CELL_COUNT];
  uint16_t cell_adc_raw[BQ76940_CELL_COUNT];
  uint16_t cell_mv[BQ76940_CELL_COUNT];
  uint16_t cell_raw_min;
  uint16_t cell_raw_max;
  uint16_t pack_adc_raw;
  uint32_t pack_mv;
  uint32_t cell_sum_mv;
  uint16_t ts_adc_raw[3];
  uint16_t ts_mv[3];
  float ts_degC[3];
  int16_t cc_raw;
  float cc_mA;
  uint16_t pack_pin_adc_raw;
  float pack_pin_mv;
} BQ76940_Watch_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile BQ76940_Watch_t g_bq76940_watch = {0u};
volatile BQ76940_Debug_t g_bq76940_debug = {0u};
volatile BQ76940_Control_t g_bq76940_control = {
  .pause_loop = 0u,
  .enable_auto_dsg_recover = 0u,
  .auto_clear_cc_ready = 0u,
  .manual_clear_sys_stat = 0u,
  .manual_enable_dsg = 0u,
  .clear_recover_latch = 0u,
  .balance_cell = 0u,
  .balance_start = 0u,
  .balance_stop = 0u,
  .balance_active_cell = 0u,
  .balance_test_started = 0u,
  .recover_latched_off = 0u,
  .balance_auto_enable = BQ76940_BALANCE_ENABLE,
  .balance_candidate_cell = 0u,
  .balance_cell_delta_mv = 0u,
  .balance_cell_min_mv = 0u,
  .balance_cell_max_mv = 0u,
  .last_recover_block_reason = BQ76940_RECOVER_BLOCK_NONE,
  .last_recover_sys_stat = 0u,
  .last_recover_sys_ctrl2 = 0u,
  .pause_enter_count = 0u,
  .manual_clear_count = 0u,
  .manual_enable_dsg_count = 0u,
  .recover_latch_count = 0u,
  .balance_timeout_ms = BQ76940_BALANCE_DEFAULT_TIMEOUT_MS,
  .balance_started_tick_ms = 0u,
  .balance_elapsed_ms = 0u,
  .balance_start_count = 0u,
  .balance_stop_count = 0u,
  .balance_last_stop_tick_ms = 0u,
  .can_tx_count = 0u,
  .can_tx_fail_count = 0u,
  .last_can_tx_status = 0u
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef BQ76940_ReadRegisterFallback(uint8_t reg, uint8_t *value);
static void BQ76940_StartMeasurements(void);
static void BQ76940_UpdateRegisterSnapshot(void);
static void BQ76940_UpdateCalibration(void);
static void BQ76940_UpdateEngineeringValues(void);
static void BQ76940_SamplePackAdcPin(void);
static uint16_t BQ76940_CellRawToMv(uint16_t raw);
static uint32_t BQ76940_PackRawToMv(uint16_t raw);
static float BQ76940_TsMvToDegC(uint16_t tsMv);
static void BQ76940_CAN_PollRx(void);
static void BQ76940_UpdateStatusLeds(HAL_StatusTypeDef crcReadStatus);
static void BQ76940_DelayWithStatusLedService(uint32_t delayMs, HAL_StatusTypeDef crcReadStatus);
static void BQ76940_WriteStatusLedMask(uint8_t ledMask, uint8_t enable);
static uint8_t BQ76940_GetPackLedMask(void);
static uint8_t BQ76940_GetLedBreathOn(uint32_t tickMs);
static void BQ76940_UpdateDebugSnapshot(HAL_StatusTypeDef crcReadStatus,
                                        HAL_StatusTypeDef plainReadStatus);
static void BQ76940_RecoverDischargeFet(void);
static void BQ76940_ServiceDebugControl(void);
static void BQ76940_ServiceDebugPauseLoop(void);
static HAL_StatusTypeDef BQ76940_SetCellBalance(uint8_t logicalCell, uint8_t enable);
static void BQ76940_StopCellBalance(void);
static void BQ76940_UpdateCellBalanceAlgorithm(void);
static void BQ76940_CAN_InitTx(void);
static void BQ76940_CAN_SendStatus(void);
static HAL_StatusTypeDef BQ76940_CAN_SendFrame(uint32_t stdId, const uint8_t data[8]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  /* If enabled, reconfigure CAN to internal loopback for software-only test */
#if (BQ76940_CAN_LOOPBACK_TEST != 0u)
  {
    HAL_CAN_Stop(&hcan);
    hcan.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan) == HAL_OK)
    {
      /* Re-configure filter and start */
      CAN_FilterTypeDef filter;
      filter.FilterBank = 0u;
      filter.FilterMode = CAN_FILTERMODE_IDMASK;
      filter.FilterScale = CAN_FILTERSCALE_32BIT;
      filter.FilterIdHigh = 0u;
      filter.FilterIdLow = 0u;
      filter.FilterMaskIdHigh = 0u;
      filter.FilterMaskIdLow = 0u;
      filter.FilterFIFOAssignment = CAN_RX_FIFO0;
      filter.FilterActivation = ENABLE;
      filter.SlaveStartFilterBank = 14u;
      (void)HAL_CAN_ConfigFilter(&hcan, &filter);
      (void)HAL_CAN_Start(&hcan);
      (void)HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR);
    }
  }
#endif
  /* USER CODE BEGIN 2 */
  BQ76200_EnableChargePump();
  BQ76940_CAN_InitTx();
  BQ76940_I2C_InitComms();
  BQ76940_StartMeasurements();
  if (BQ76940_BALANCE_TEST_ENABLE != 0u)
  {
    g_bq76940_control.balance_cell = BQ76940_BALANCE_TEST_CELL;
    g_bq76940_control.balance_start = 1u;
  }
  HAL_Delay(800u);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_StatusTypeDef crcReadStatus;
    HAL_StatusTypeDef plainReadStatus;
    uint8_t sysStatCrc = 0u;
    uint8_t sysStatPlain = 0u;

    g_bq76940_watch.loop_count++;
    g_bq76940_watch.last_tick_ms = HAL_GetTick();
    BQ76940_ServiceDebugControl();

    crcReadStatus = I2CReadRegisterByteWithCRC(BQ76940_I2C_ADDR_7BIT,
                                               BQ76940_REG_SYS_STAT,
                                               &sysStatCrc);
    plainReadStatus = I2CReadRegisterByte(BQ76940_I2C_ADDR_7BIT,
                                          BQ76940_REG_SYS_STAT,
                                          &sysStatPlain);

    g_bq76940_watch.last_crc_read_status = (uint32_t)crcReadStatus;
    g_bq76940_watch.last_plain_read_status = (uint32_t)plainReadStatus;
    g_bq76940_watch.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);
    g_bq76940_watch.using_plain_fallback = 0u;

    BQ76940_UpdateRegisterSnapshot();
    BQ76940_UpdateEngineeringValues();
    BQ76940_SamplePackAdcPin();
    BQ76940_CAN_PollRx();
    BQ76940_UpdateCellBalanceAlgorithm();

    if (crcReadStatus == HAL_OK)
    {
      g_bq76940_watch.sys_stat_crc = sysStatCrc;
    }

    if (plainReadStatus == HAL_OK)
    {
      g_bq76940_watch.sys_stat_plain = sysStatPlain;
    }

    if (crcReadStatus == HAL_OK)
    {
      g_bq76940_watch.crc_read_ok_count++;
      g_bq76940_watch.sys_stat_effective = g_bq76940_watch.sys_stat_crc;
    }
    else if ((plainReadStatus == HAL_OK) && (g_bq76940_watch.last_hal_i2c_error == HAL_I2C_ERROR_NONE))
    {
      g_bq76940_watch.crc_read_fail_count++;
      g_bq76940_watch.crc_mismatch_count++;
      g_bq76940_watch.plain_fallback_ok_count++;
      g_bq76940_watch.using_plain_fallback = 1u;
      g_bq76940_watch.sys_stat_effective = g_bq76940_watch.sys_stat_plain;
    }
    else
    {
      g_bq76940_watch.crc_read_fail_count++;
    }

    BQ76940_RecoverDischargeFet();

    /* Update LED4-1 based on pack voltage state and communication status */
    BQ76940_UpdateStatusLeds(crcReadStatus);

    g_bq76940_watch.alert_afe_level = (uint8_t)HAL_GPIO_ReadPin(ALERT_AFE_GPIO_Port, ALERT_AFE_Pin);
    g_bq76940_watch.alert_gauge_level = (uint8_t)HAL_GPIO_ReadPin(ALERT_GAUGE_GPIO_Port, ALERT_GAUGE_Pin);
    g_bq76940_watch.i2c_sw_level = (uint8_t)HAL_GPIO_ReadPin(I2C_SW_GPIO_Port, I2C_SW_Pin);
    BQ76940_UpdateDebugSnapshot(crcReadStatus, plainReadStatus);
    BQ76940_CAN_SendStatus();

    BQ76940_DelayWithStatusLedService(BQ76940_STATUS_LED_PERIOD_MS, crcReadStatus);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static HAL_StatusTypeDef BQ76940_ReadRegisterFallback(uint8_t reg, uint8_t *value)
{
  if (I2CReadRegisterByteWithCRC(BQ76940_I2C_ADDR_7BIT, reg, value) == HAL_OK)
  {
    return HAL_OK;
  }

  return I2CReadRegisterByte(BQ76940_I2C_ADDR_7BIT, reg, value);
}

static HAL_StatusTypeDef BQ76940_WriteRegisterFallback(uint8_t reg, uint8_t value)
{
  if (I2CWriteRegisterByteWithCRC(BQ76940_I2C_ADDR_7BIT, reg, value) == HAL_OK)
  {
    return HAL_OK;
  }

  return I2CWriteRegisterByte(BQ76940_I2C_ADDR_7BIT, reg, value);
}

static void BQ76200_EnableChargePump(void)
{
  HAL_GPIO_WritePin(CHARGE_PUMP_GPIO_Port, CHARGE_PUMP_Pin, GPIO_PIN_SET);
}

static void BQ76940_StartMeasurements(void)
{
  uint8_t sysCtrl1;
  uint8_t sysCtrl2;

  (void)BQ76940_WriteRegisterFallback(BQ76940_REG_CC_CFG, 0x19u);

  sysCtrl1 = (uint8_t)(BQ76940_SYS_CTRL1_ADC_EN | BQ76940_SYS_CTRL1_TEMP_SEL);
  (void)BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_CTRL1, sysCtrl1);

  /* Set SYS_STAT to 0xFF to clear any latched faults (e.g. DEVICE_XREADY) before enabling FETs */
  (void)BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_STAT, 0xFFu);

  /* Enable Coulomb Counter AND Discharge FET in SYS_CTRL2 */
  if (BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_CTRL2, &sysCtrl2) == HAL_OK)
  {
    /* Bit 6 = CC_EN, Bit 1 = DSG_ON */
    sysCtrl2 |= (0x40u | 0x02u);
    (void)BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_CTRL2, sysCtrl2);
  }
}

static void BQ76940_UpdateRegisterSnapshot(void)
{
  uint8_t reg;

  for (reg = BQ76940_REG_FIRST; reg <= BQ76940_REG_LAST; reg++)
  {
    uint8_t value = 0u;

    if (I2CReadRegisterByteWithCRC(BQ76940_I2C_ADDR_7BIT, reg, &value) == HAL_OK)
    {
      g_bq76940_watch.reg_value[reg] = value;
      g_bq76940_watch.reg_source[reg] = 2u;
    }
    else if (I2CReadRegisterByte(BQ76940_I2C_ADDR_7BIT, reg, &value) == HAL_OK)
    {
      g_bq76940_watch.reg_value[reg] = value;
      g_bq76940_watch.reg_source[reg] = 1u;
    }
    else
    {
      g_bq76940_watch.reg_value[reg] = 0u;
      g_bq76940_watch.reg_source[reg] = 0u;
    }
  }
}

static void BQ76940_UpdateCalibration(void)
{
  uint8_t gain1;
  uint8_t gain2;
  uint8_t offset;
  uint8_t gainCode;

  g_bq76940_watch.calibration_valid = 0u;
  g_bq76940_watch.adc_gain_uv_per_lsb = BQ76940_DEFAULT_ADC_GAIN_UV;
  g_bq76940_watch.adc_offset_mv = 0;

  if ((BQ76940_ReadRegisterFallback(BQ76940_REG_ADCGAIN1, &gain1) != HAL_OK) ||
      (BQ76940_ReadRegisterFallback(BQ76940_REG_ADCOFFSET, &offset) != HAL_OK) ||
      (BQ76940_ReadRegisterFallback(BQ76940_REG_ADCGAIN2, &gain2) != HAL_OK))
  {
    return;
  }

  gainCode = (uint8_t)(((gain1 & 0x0Cu) << 1u) | ((gain2 & 0xE0u) >> 5u));

  g_bq76940_watch.adc_gain1_raw = gain1;
  g_bq76940_watch.adc_gain2_raw = gain2;
  g_bq76940_watch.adc_offset_raw = offset;
  g_bq76940_watch.adc_gain_uv_per_lsb = (uint16_t)(365u + gainCode);
  g_bq76940_watch.adc_offset_mv = (offset < 0x80u) ? (int16_t)offset : (int16_t)((int16_t)offset - 256);
  g_bq76940_watch.calibration_valid = 1u;
}

static uint16_t BQ76940_CellRawToMv(uint16_t raw)
{
  int32_t mv;

  mv = (int32_t)((((uint32_t)raw * (uint32_t)g_bq76940_watch.adc_gain_uv_per_lsb) + 500u) / 1000u);
  mv += (int32_t)g_bq76940_watch.adc_offset_mv;

  if (mv <= 0)
  {
    return 0u;
  }

  if (mv > 0xFFFF)
  {
    return 0xFFFFu;
  }

  return (uint16_t)mv;
}

static uint32_t BQ76940_PackRawToMv(uint16_t raw)
{
  int32_t mv;

  mv = (int32_t)((((uint32_t)raw * 4u * (uint32_t)g_bq76940_watch.adc_gain_uv_per_lsb) + 500u) / 1000u);
  mv += (int32_t)BQ76940_ACTIVE_CELL_COUNT * (int32_t)g_bq76940_watch.adc_offset_mv;

  if (mv <= 0)
  {
    return 0u;
  }

  return (uint32_t)mv;
}

static float BQ76940_TsMvToDegC(uint16_t tsMv)
{
  float tsVoltageMv;
  float ntcOhm;
  float invTempK;

  if ((tsMv == 0u) || ((float)tsMv >= BQ76940_TS_BIAS_MV))
  {
    return -273.15f;
  }

  tsVoltageMv = (float)tsMv;
  ntcOhm = (BQ76940_TS_PULLUP_OHM * tsVoltageMv) / (BQ76940_TS_BIAS_MV - tsVoltageMv);
  invTempK = (1.0f / 298.15f) + (logf(ntcOhm / BQ76940_TS_NTC_R0_OHM) / BQ76940_TS_NTC_BETA);

  return (1.0f / invTempK) - 273.15f;
}

static void BQ76940_CAN_PollRx(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  /* Poll FIFO0 fill level and read all pending messages */
  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0u)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      g_bq76940_control.can_rx_count = (g_bq76940_control.can_rx_count + 1u);
      g_bq76940_control.last_can_rx_id = rxHeader.StdId;
      g_bq76940_control.last_can_rx_dlc = rxHeader.DLC;
      for (uint8_t i = 0u; i < 8u; i++)
      {
        g_bq76940_control.last_can_rx_data[i] = rxData[i];
      }
    }
    else
    {
      break;
    }
  }
}

static void BQ76940_UpdateEngineeringValues(void)
{
  uint8_t idx;
  uint8_t hiReg;
  uint8_t loReg;
  uint16_t raw;

  BQ76940_UpdateCalibration();

  g_bq76940_watch.cell_raw_min = 0xFFFFu;
  g_bq76940_watch.cell_raw_max = 0u;
  g_bq76940_watch.cell_sum_mv = 0u;

  for (idx = 0u; idx < BQ76940_CELL_COUNT; idx++)
  {
    hiReg = (uint8_t)(0x0Cu + (idx * 2u));
    loReg = (uint8_t)(hiReg + 1u);

    if ((g_bq76940_watch.reg_source[hiReg] != 0u) && (g_bq76940_watch.reg_source[loReg] != 0u))
    {
      raw = (uint16_t)((((uint16_t)(g_bq76940_watch.reg_value[hiReg] & 0x3Fu)) << 8) |
                       g_bq76940_watch.reg_value[loReg]);
      g_bq76940_watch.phys_cell_adc_raw[idx] = raw;
      g_bq76940_watch.phys_cell_mv[idx] = BQ76940_CellRawToMv(raw);

      if (raw < g_bq76940_watch.cell_raw_min)
      {
        g_bq76940_watch.cell_raw_min = raw;
      }

      if (raw > g_bq76940_watch.cell_raw_max)
      {
        g_bq76940_watch.cell_raw_max = raw;
      }
    }
    else
    {
      g_bq76940_watch.phys_cell_adc_raw[idx] = 0u;
      g_bq76940_watch.phys_cell_mv[idx] = 0u;
    }
  }

  /* Fill logical cell array using physical pair mapping for 13-cell wiring. */
  for (idx = 0u; idx < BQ76940_ACTIVE_CELL_COUNT; idx++)
  {
    uint8_t phys = s_phys_pair_for_cell13[idx];

    g_bq76940_watch.cell_adc_raw[idx] = g_bq76940_watch.phys_cell_adc_raw[phys];
    g_bq76940_watch.cell_mv[idx] = g_bq76940_watch.phys_cell_mv[phys];
    g_bq76940_watch.cell_sum_mv += g_bq76940_watch.cell_mv[idx];
  }

  for (idx = BQ76940_ACTIVE_CELL_COUNT; idx < BQ76940_CELL_COUNT; idx++)
  {
    g_bq76940_watch.cell_adc_raw[idx] = 0u;
    g_bq76940_watch.cell_mv[idx] = 0u;
  }

  if (g_bq76940_watch.cell_raw_min == 0xFFFFu)
  {
    g_bq76940_watch.cell_raw_min = 0u;
  }

  if ((g_bq76940_watch.reg_source[0x2Au] != 0u) && (g_bq76940_watch.reg_source[0x2Bu] != 0u))
  {
    g_bq76940_watch.pack_adc_raw = (uint16_t)(((uint16_t)g_bq76940_watch.reg_value[0x2Au] << 8) |
                                              g_bq76940_watch.reg_value[0x2Bu]);
  }
  else
  {
    g_bq76940_watch.pack_adc_raw = 0u;
  }

  g_bq76940_watch.pack_mv = BQ76940_PackRawToMv(g_bq76940_watch.pack_adc_raw);

  for (idx = 0u; idx < 3u; idx++)
  {
    hiReg = (uint8_t)(0x2Cu + (idx * 2u));
    loReg = (uint8_t)(hiReg + 1u);

    if ((g_bq76940_watch.reg_source[hiReg] != 0u) && (g_bq76940_watch.reg_source[loReg] != 0u))
    {
      raw = (uint16_t)((((uint16_t)(g_bq76940_watch.reg_value[hiReg] & 0x3Fu)) << 8) |
                       g_bq76940_watch.reg_value[loReg]);
      g_bq76940_watch.ts_adc_raw[idx] = raw;
      g_bq76940_watch.ts_mv[idx] = raw;
      g_bq76940_watch.ts_degC[idx] = BQ76940_TsMvToDegC(raw);
    }
    else
    {
      g_bq76940_watch.ts_adc_raw[idx] = 0u;
      g_bq76940_watch.ts_mv[idx] = 0u;
      g_bq76940_watch.ts_degC[idx] = -273.15f;
    }
  }

  if ((g_bq76940_watch.reg_source[0x32u] != 0u) && (g_bq76940_watch.reg_source[0x33u] != 0u))
  {
    g_bq76940_watch.cc_raw = (int16_t)(((uint16_t)g_bq76940_watch.reg_value[0x32u] << 8) |
                                       g_bq76940_watch.reg_value[0x33u]);
  }
  else
  {
    g_bq76940_watch.cc_raw = 0;
  }

  g_bq76940_watch.cc_mA = ((float)g_bq76940_watch.cc_raw * 8.44f) / BQ76940_RSNS_MOHM;
}

static void BQ76940_SamplePackAdcPin(void)
{
  uint32_t adcRaw;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    g_bq76940_watch.pack_pin_adc_raw = 0u;
    g_bq76940_watch.pack_pin_mv = 0.0f;
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10u) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    g_bq76940_watch.pack_pin_adc_raw = 0u;
    g_bq76940_watch.pack_pin_mv = 0.0f;
    return;
  }

  adcRaw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  g_bq76940_watch.pack_pin_adc_raw = (uint16_t)adcRaw;
  g_bq76940_watch.pack_pin_mv = ((float)adcRaw * 3300.0f) / 4095.0f;
}

static void BQ76940_UpdateStatusLeds(HAL_StatusTypeDef crcReadStatus)
{
  uint8_t ledMask;
  uint8_t ledEnable;
  uint8_t isFault;
  uint8_t isCommError;
  uint8_t isDischarging;

  /* Determine if communication failed */
  isCommError = (crcReadStatus != HAL_OK) ? 1u : 0u;

  /* Determine if protection fault occurred (only real protection bits) */
  isFault = (g_bq76940_watch.sys_stat_effective & BQ76940_SYS_STAT_FAULT_MASK) != 0u ? 1u : 0u;
  isDischarging = ((g_bq76940_watch.reg_source[BQ76940_REG_SYS_CTRL2] != 0u) &&
                   ((g_bq76940_watch.reg_value[BQ76940_REG_SYS_CTRL2] & BQ76940_SYS_CTRL2_DSG_ON) != 0u)) ? 1u : 0u;
  g_bq76940_watch.dsg_on = isDischarging;

  if (isCommError || isFault)
  {
    /* Communication error or fault detected: all LEDs blink together. */
    g_bq76940_watch.led1_is_blinking = 1u;
    ledEnable = (((HAL_GetTick() / BQ76940_STATUS_LED_PERIOD_MS) & 0x01u) == 0u) ? 1u : 0u;
    BQ76940_WriteStatusLedMask((uint8_t)(BQ76940_LED_MASK_1 |
                                         BQ76940_LED_MASK_2 |
                                         BQ76940_LED_MASK_3 |
                                         BQ76940_LED_MASK_4),
                               ledEnable);
  }
  else
  {
    g_bq76940_watch.led1_is_blinking = 0u;
    ledMask = BQ76940_GetPackLedMask();
    ledEnable = (isDischarging != 0u) ? BQ76940_GetLedBreathOn(HAL_GetTick()) : 1u;

    BQ76940_WriteStatusLedMask(ledMask, ledEnable);
  }

  /* Update LED1 level observer */
  g_bq76940_watch.led1_level = (uint8_t)HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin);
}

static void BQ76940_DelayWithStatusLedService(uint32_t delayMs, HAL_StatusTypeDef crcReadStatus)
{
  uint32_t startTick;

  startTick = HAL_GetTick();
  while ((HAL_GetTick() - startTick) < delayMs)
  {
    BQ76940_UpdateStatusLeds(crcReadStatus);
    HAL_Delay(BQ76940_LED_SERVICE_STEP_MS);
  }
}

static void BQ76940_WriteStatusLedMask(uint8_t ledMask, uint8_t enable)
{
  GPIO_PinState offState;

  offState = GPIO_PIN_RESET;
  g_bq76940_watch.led_bar_mask = ledMask;

  HAL_GPIO_WritePin(LED1_GPIO_Port,
                    LED1_Pin,
                    ((enable != 0u) && ((ledMask & BQ76940_LED_MASK_1) != 0u)) ? GPIO_PIN_SET : offState);
  HAL_GPIO_WritePin(LED2_GPIO_Port,
                    LED2_Pin,
                    ((enable != 0u) && ((ledMask & BQ76940_LED_MASK_2) != 0u)) ? GPIO_PIN_SET : offState);
  HAL_GPIO_WritePin(LED3_GPIO_Port,
                    LED3_Pin,
                    ((enable != 0u) && ((ledMask & BQ76940_LED_MASK_3) != 0u)) ? GPIO_PIN_SET : offState);
  HAL_GPIO_WritePin(LED4_GPIO_Port,
                    LED4_Pin,
                    ((enable != 0u) && ((ledMask & BQ76940_LED_MASK_4) != 0u)) ? GPIO_PIN_SET : offState);
}

static uint8_t BQ76940_GetPackLedMask(void)
{
  uint8_t ledMask;

  ledMask = 0u;

  /* Board order is LED4 -> LED3 -> LED2 -> LED1 for 25/50/75/100%. */
  if (g_bq76940_watch.pack_mv >= BQ76940_PACK_MV_25PCT)
  {
    ledMask |= BQ76940_LED_MASK_4;
  }

  if (g_bq76940_watch.pack_mv >= BQ76940_PACK_MV_50PCT)
  {
    ledMask |= BQ76940_LED_MASK_3;
  }

  if (g_bq76940_watch.pack_mv >= BQ76940_PACK_MV_75PCT)
  {
    ledMask |= BQ76940_LED_MASK_2;
  }

  if (g_bq76940_watch.pack_mv >= BQ76940_PACK_MV_MAX)
  {
    ledMask |= BQ76940_LED_MASK_1;
  }

  return ledMask;
}

static uint8_t BQ76940_GetLedBreathOn(uint32_t tickMs)
{
  uint32_t breathPhase;
  uint32_t pwmPhase;
  uint32_t level;
  uint32_t duty;
  uint32_t halfPeriod;

  halfPeriod = BQ76940_LED_BREATH_PERIOD_MS / 2u;
  breathPhase = tickMs % BQ76940_LED_BREATH_PERIOD_MS;

  if (breathPhase > halfPeriod)
  {
    breathPhase = BQ76940_LED_BREATH_PERIOD_MS - breathPhase;
  }

  level = (breathPhase * 1000u) / halfPeriod;
  level = (level * level * (3000u - (2u * level))) / 1000000u;
  duty = 1u + ((level * (BQ76940_LED_PWM_STEPS - 2u)) / 1000u);
  pwmPhase = (tickMs % BQ76940_LED_PWM_PERIOD_MS) * BQ76940_LED_PWM_STEPS;
  pwmPhase /= BQ76940_LED_PWM_PERIOD_MS;

  g_bq76940_watch.led_breath_duty = (uint8_t)duty;

  return (pwmPhase < duty) ? 1u : 0u;
}

static void BQ76940_RecoverDischargeFet(void)
{
  uint8_t sysStat;
  uint8_t clearMask;
  uint8_t sysCtrl2;

  g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_NONE;
  sysStat = g_bq76940_watch.sys_stat_effective;
  clearMask = (uint8_t)(sysStat & BQ76940_SYS_STAT_CLEAR_NORMAL_MASK);

  if ((g_bq76940_control.recover_latched_off != 0u) ||
      ((sysStat & BQ76940_SYS_STAT_OVRD_ALERT) != 0u))
  {
    if ((sysStat & BQ76940_SYS_STAT_OVRD_ALERT) != 0u)
    {
      g_bq76940_control.recover_latched_off = 1u;
      g_bq76940_control.recover_latch_count++;
      g_bq76940_control.last_recover_sys_stat = sysStat;
    }

    g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_OVRD_ALERT;
    return;
  }

  if (((sysStat & BQ76940_SYS_STAT_CC_READY) != 0u) &&
      (g_bq76940_control.auto_clear_cc_ready != 0u))
  {
    if (BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_STAT, BQ76940_SYS_STAT_CC_READY) == HAL_OK)
    {
      sysStat = (uint8_t)(sysStat & (uint8_t)(~BQ76940_SYS_STAT_CC_READY));
      g_bq76940_watch.sys_stat_effective = sysStat;
      g_bq76940_watch.sys_stat_crc = (uint8_t)(g_bq76940_watch.sys_stat_crc &
                                               (uint8_t)(~BQ76940_SYS_STAT_CC_READY));
      g_bq76940_watch.sys_stat_plain = (uint8_t)(g_bq76940_watch.sys_stat_plain &
                                                 (uint8_t)(~BQ76940_SYS_STAT_CC_READY));
      g_bq76940_watch.reg_value[BQ76940_REG_SYS_STAT] = sysStat;
      clearMask = (uint8_t)(sysStat & BQ76940_SYS_STAT_CLEAR_NORMAL_MASK);
    }
  }

  if (g_bq76940_control.enable_auto_dsg_recover == 0u)
  {
    g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_DISABLED;
    return;
  }

  if (clearMask != 0u)
  {
    if (BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_STAT, clearMask) != HAL_OK)
    {
      g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_COMM;
      return;
    }

    g_bq76940_watch.sys_stat_effective = (uint8_t)(g_bq76940_watch.sys_stat_effective & (uint8_t)(~clearMask));
    g_bq76940_watch.sys_stat_crc = (uint8_t)(g_bq76940_watch.sys_stat_crc & (uint8_t)(~clearMask));
    g_bq76940_watch.sys_stat_plain = (uint8_t)(g_bq76940_watch.sys_stat_plain & (uint8_t)(~clearMask));
    g_bq76940_watch.reg_value[BQ76940_REG_SYS_STAT] = (uint8_t)(g_bq76940_watch.reg_value[BQ76940_REG_SYS_STAT] &
                                                               (uint8_t)(~clearMask));

    if ((clearMask & BQ76940_SYS_STAT_OVRD_ALERT) != 0u)
    {
      if (BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_STAT, &sysStat) != HAL_OK)
      {
        g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_COMM;
        return;
      }

      g_bq76940_watch.sys_stat_effective = sysStat;
      g_bq76940_watch.reg_value[BQ76940_REG_SYS_STAT] = sysStat;

      if ((sysStat & BQ76940_SYS_STAT_OVRD_ALERT) != 0u)
      {
        g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_OVRD_ALERT;
        g_bq76940_control.last_recover_sys_stat = sysStat;
        return;
      }
    }
  }

  if ((g_bq76940_watch.sys_stat_effective & BQ76940_SYS_STAT_FAULT_MASK) != 0u)
  {
    g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_FAULT;
    g_bq76940_control.last_recover_sys_stat = g_bq76940_watch.sys_stat_effective;
    return;
  }

  if (BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_CTRL2, &sysCtrl2) != HAL_OK)
  {
    g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_COMM;
    return;
  }

  g_bq76940_control.last_recover_sys_ctrl2 = sysCtrl2;

  if ((sysCtrl2 & BQ76940_SYS_CTRL2_DSG_ON) == 0u)
  {
    sysCtrl2 |= (BQ76940_SYS_CTRL2_CC_EN | BQ76940_SYS_CTRL2_DSG_ON);
    if (BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_CTRL2, sysCtrl2) == HAL_OK)
    {
      g_bq76940_watch.fet_recover_count++;
      g_bq76940_watch.reg_value[BQ76940_REG_SYS_CTRL2] = sysCtrl2;
      g_bq76940_watch.reg_source[BQ76940_REG_SYS_CTRL2] = 1u;
      g_bq76940_control.last_recover_sys_ctrl2 = sysCtrl2;
    }
    else
    {
      g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_COMM;
    }
  }
}

static void BQ76940_ServiceDebugControl(void)
{
  uint8_t sysStat;
  uint8_t sysCtrl2;

  if (g_bq76940_control.pause_loop != 0u)
  {
    g_bq76940_control.pause_enter_count++;
    while (g_bq76940_control.pause_loop != 0u)
    {
      BQ76940_ServiceDebugPauseLoop();
      HAL_Delay(10u);
    }
  }

  if (g_bq76940_control.clear_recover_latch != 0u)
  {
    g_bq76940_control.clear_recover_latch = 0u;
    g_bq76940_control.recover_latched_off = 0u;
    g_bq76940_control.last_recover_block_reason = BQ76940_RECOVER_BLOCK_NONE;
  }

  if (g_bq76940_control.manual_clear_sys_stat != 0u)
  {
    g_bq76940_control.manual_clear_sys_stat = 0u;
    if (BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_STAT, 0xFFu) == HAL_OK)
    {
      g_bq76940_control.manual_clear_count++;
    }
  }

  if (g_bq76940_control.manual_enable_dsg != 0u)
  {
    g_bq76940_control.manual_enable_dsg = 0u;

    if ((BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_STAT, &sysStat) == HAL_OK) &&
        ((sysStat & (uint8_t)(BQ76940_SYS_STAT_FAULT_MASK | BQ76940_SYS_STAT_OVRD_ALERT)) == 0u) &&
        (BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_CTRL2, &sysCtrl2) == HAL_OK))
    {
      sysCtrl2 |= (BQ76940_SYS_CTRL2_CC_EN | BQ76940_SYS_CTRL2_DSG_ON);
      if (BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_CTRL2, sysCtrl2) == HAL_OK)
      {
        g_bq76940_control.manual_enable_dsg_count++;
      }
    }
  }

  if (g_bq76940_control.balance_stop != 0u)
  {
    g_bq76940_control.balance_stop = 0u;
    BQ76940_StopCellBalance();
  }

  if (g_bq76940_control.balance_start != 0u)
  {
    g_bq76940_control.balance_start = 0u;

    if (BQ76940_SetCellBalance(g_bq76940_control.balance_cell, 1u) == HAL_OK)
    {
      g_bq76940_control.balance_active_cell = g_bq76940_control.balance_cell;
      g_bq76940_control.balance_started_tick_ms = HAL_GetTick();
      g_bq76940_control.balance_elapsed_ms = 0u;
      g_bq76940_control.balance_start_count++;
    }
  }

  if (g_bq76940_control.balance_active_cell != 0u)
  {
    g_bq76940_control.balance_elapsed_ms = HAL_GetTick() - g_bq76940_control.balance_started_tick_ms;
    if ((g_bq76940_control.balance_timeout_ms == 0u) ||
        (g_bq76940_control.balance_elapsed_ms >= g_bq76940_control.balance_timeout_ms))
    {
      BQ76940_StopCellBalance();
    }
  }
}

static HAL_StatusTypeDef BQ76940_SetCellBalance(uint8_t logicalCell, uint8_t enable)
{
  uint8_t physCell;
  uint8_t balanceReg;
  uint8_t balanceBit;
  HAL_StatusTypeDef status;

  status = BQ76940_WriteRegisterFallback(BQ76940_REG_CELLBAL1, 0u);
  if (status != HAL_OK)
  {
    return status;
  }

  status = BQ76940_WriteRegisterFallback(BQ76940_REG_CELLBAL2, 0u);
  if (status != HAL_OK)
  {
    return status;
  }

  status = BQ76940_WriteRegisterFallback(BQ76940_REG_CELLBAL3, 0u);
  if (status != HAL_OK)
  {
    return status;
  }

  g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL1] = 0u;
  g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL2] = 0u;
  g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL3] = 0u;
  g_bq76940_watch.reg_source[BQ76940_REG_CELLBAL1] = 1u;
  g_bq76940_watch.reg_source[BQ76940_REG_CELLBAL2] = 1u;
  g_bq76940_watch.reg_source[BQ76940_REG_CELLBAL3] = 1u;

  if (enable == 0u)
  {
    return HAL_OK;
  }

  if ((logicalCell == 0u) || (logicalCell > BQ76940_ACTIVE_CELL_COUNT))
  {
    return HAL_ERROR;
  }

  physCell = s_phys_pair_for_cell13[logicalCell - 1u];
  balanceReg = (uint8_t)(BQ76940_REG_CELLBAL1 + (physCell / 5u));
  balanceBit = (uint8_t)(1u << (physCell % 5u));

  status = BQ76940_WriteRegisterFallback(balanceReg, balanceBit);
  if (status == HAL_OK)
  {
    g_bq76940_watch.reg_value[balanceReg] = balanceBit;
    g_bq76940_watch.reg_source[balanceReg] = 1u;
  }

  return status;
}

static void BQ76940_StopCellBalance(void)
{
  if (BQ76940_SetCellBalance(0u, 0u) == HAL_OK)
  {
    g_bq76940_control.balance_active_cell = 0u;
    g_bq76940_control.balance_elapsed_ms = 0u;
    g_bq76940_control.balance_last_stop_tick_ms = HAL_GetTick();
    g_bq76940_control.balance_stop_count++;
  }
}

static void BQ76940_UpdateCellBalanceAlgorithm(void)
{
  uint8_t idx;
  uint8_t candidateCell;
  uint16_t cellMv;
  uint16_t cellMinMv;
  uint16_t cellMaxMv;
  uint16_t maxDeltaMv;
  uint32_t nowTick;

  nowTick = HAL_GetTick();
  candidateCell = 0u;
  cellMinMv = 0xFFFFu;
  cellMaxMv = 0u;
  maxDeltaMv = 0u;

  if (g_bq76940_control.balance_auto_enable == 0u)
  {
    if (g_bq76940_control.balance_active_cell != 0u)
    {
      BQ76940_StopCellBalance();
    }
    return;
  }

  if ((g_bq76940_watch.sys_stat_effective & (uint8_t)(BQ76940_SYS_STAT_FAULT_MASK |
                                                       BQ76940_SYS_STAT_OVRD_ALERT)) != 0u)
  {
    if (g_bq76940_control.balance_active_cell != 0u)
    {
      BQ76940_StopCellBalance();
    }
    return;
  }

  for (idx = 0u; idx < BQ76940_ACTIVE_CELL_COUNT; idx++)
  {
    cellMv = g_bq76940_watch.cell_mv[idx];
    if (cellMv == 0u)
    {
      return;
    }

    if (cellMv < cellMinMv)
    {
      cellMinMv = cellMv;
    }

    if (cellMv > cellMaxMv)
    {
      cellMaxMv = cellMv;
    }
  }

  g_bq76940_control.balance_cell_min_mv = cellMinMv;
  g_bq76940_control.balance_cell_max_mv = cellMaxMv;

  if (cellMinMv < BQ76940_BALANCE_MIN_CELL_MV)
  {
    if (g_bq76940_control.balance_active_cell != 0u)
    {
      BQ76940_StopCellBalance();
    }
    return;
  }

  for (idx = 0u; idx < BQ76940_ACTIVE_CELL_COUNT; idx++)
  {
    uint16_t deltaMv;

    cellMv = g_bq76940_watch.cell_mv[idx];
    deltaMv = (uint16_t)(cellMv - cellMinMv);
    if (deltaMv > maxDeltaMv)
    {
      maxDeltaMv = deltaMv;
      candidateCell = (uint8_t)(idx + 1u);
    }
  }

  g_bq76940_control.balance_candidate_cell = candidateCell;
  g_bq76940_control.balance_cell_delta_mv = maxDeltaMv;

  if (g_bq76940_control.balance_active_cell != 0u)
  {
    uint8_t activeIdx;
    uint16_t activeDeltaMv;

    activeIdx = (uint8_t)(g_bq76940_control.balance_active_cell - 1u);
    activeDeltaMv = (uint16_t)(g_bq76940_watch.cell_mv[activeIdx] - cellMinMv);
    g_bq76940_control.balance_elapsed_ms = nowTick - g_bq76940_control.balance_started_tick_ms;

    if ((activeDeltaMv <= BQ76940_BALANCE_STOP_DELTA_MV) ||
        (g_bq76940_control.balance_elapsed_ms >= g_bq76940_control.balance_timeout_ms))
    {
      BQ76940_StopCellBalance();
    }
    return;
  }

  if ((nowTick - g_bq76940_control.balance_last_stop_tick_ms) < BQ76940_BALANCE_COOLDOWN_MS)
  {
    return;
  }

  if ((candidateCell != 0u) && (maxDeltaMv >= BQ76940_BALANCE_START_DELTA_MV))
  {
    if (BQ76940_SetCellBalance(candidateCell, 1u) == HAL_OK)
    {
      g_bq76940_control.balance_active_cell = candidateCell;
      g_bq76940_control.balance_started_tick_ms = nowTick;
      g_bq76940_control.balance_elapsed_ms = 0u;
      g_bq76940_control.balance_start_count++;
    }
  }
}

static void BQ76940_CAN_InitTx(void)
{
  CAN_FilterTypeDef filter;

  filter.FilterBank = 0u;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0u;
  filter.FilterIdLow = 0u;
  filter.FilterMaskIdHigh = 0u;
  filter.FilterMaskIdLow = 0u;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14u;

  (void)HAL_CAN_ConfigFilter(&hcan, &filter);
  (void)HAL_CAN_Start(&hcan);
  /* Enable RX interrupt and basic TX/error notifications for diagnostics */
  (void)HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR);
}

/**
  * @brief Callback when a CAN message is pending in RX FIFO0.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
  {
    /* Store into control structure for Watch inspection */
    g_bq76940_control.can_rx_count = (g_bq76940_control.can_rx_count + 1u);
    g_bq76940_control.last_can_rx_id = rxHeader.StdId;
    g_bq76940_control.last_can_rx_dlc = rxHeader.DLC;
    for (uint8_t i = 0u; i < 8u; i++)
    {
      g_bq76940_control.last_can_rx_data[i] = rxData[i];
    }
  }
}

static void BQ76940_CAN_SendStatus(void)
{
  uint8_t data[8];
  uint32_t nowTick;
  uint8_t sysCtrl2;

  static uint32_t s_lastCanTxTick = 0u;

  nowTick = HAL_GetTick();
  if ((nowTick - s_lastCanTxTick) < BQ76940_CAN_TX_PERIOD_MS)
  {
    return;
  }
  s_lastCanTxTick = nowTick;

  sysCtrl2 = 0u;
  if (g_bq76940_watch.reg_source[BQ76940_REG_SYS_CTRL2] != 0u)
  {
    sysCtrl2 = g_bq76940_watch.reg_value[BQ76940_REG_SYS_CTRL2];
  }

  data[0] = (uint8_t)(g_bq76940_watch.pack_mv & 0xFFu);
  data[1] = (uint8_t)((g_bq76940_watch.pack_mv >> 8u) & 0xFFu);
  data[2] = (uint8_t)((g_bq76940_watch.pack_mv >> 16u) & 0xFFu);
  data[3] = (uint8_t)((g_bq76940_watch.pack_mv >> 24u) & 0xFFu);
  data[4] = g_bq76940_watch.sys_stat_effective;
  data[5] = sysCtrl2;
  data[6] = g_bq76940_control.balance_active_cell;
  data[7] = g_bq76940_control.balance_candidate_cell;
  (void)BQ76940_CAN_SendFrame(BQ76940_CAN_ID_STATUS, data);

  data[0] = (uint8_t)(g_bq76940_watch.cell_mv[0] & 0xFFu);
  data[1] = (uint8_t)(g_bq76940_watch.cell_mv[0] >> 8u);
  data[2] = (uint8_t)(g_bq76940_watch.cell_mv[1] & 0xFFu);
  data[3] = (uint8_t)(g_bq76940_watch.cell_mv[1] >> 8u);
  data[4] = (uint8_t)(g_bq76940_watch.cell_mv[2] & 0xFFu);
  data[5] = (uint8_t)(g_bq76940_watch.cell_mv[2] >> 8u);
  data[6] = (uint8_t)(g_bq76940_watch.cell_mv[3] & 0xFFu);
  data[7] = (uint8_t)(g_bq76940_watch.cell_mv[3] >> 8u);
  (void)BQ76940_CAN_SendFrame(BQ76940_CAN_ID_CELLS_1_4, data);

  data[0] = (uint8_t)(g_bq76940_watch.cell_mv[4] & 0xFFu);
  data[1] = (uint8_t)(g_bq76940_watch.cell_mv[4] >> 8u);
  data[2] = (uint8_t)(g_bq76940_watch.cell_mv[5] & 0xFFu);
  data[3] = (uint8_t)(g_bq76940_watch.cell_mv[5] >> 8u);
  data[4] = (uint8_t)(g_bq76940_watch.cell_mv[6] & 0xFFu);
  data[5] = (uint8_t)(g_bq76940_watch.cell_mv[6] >> 8u);
  data[6] = (uint8_t)(g_bq76940_watch.cell_mv[7] & 0xFFu);
  data[7] = (uint8_t)(g_bq76940_watch.cell_mv[7] >> 8u);
  (void)BQ76940_CAN_SendFrame(BQ76940_CAN_ID_CELLS_5_8, data);

  data[0] = (uint8_t)(g_bq76940_watch.cell_mv[8] & 0xFFu);
  data[1] = (uint8_t)(g_bq76940_watch.cell_mv[8] >> 8u);
  data[2] = (uint8_t)(g_bq76940_watch.cell_mv[9] & 0xFFu);
  data[3] = (uint8_t)(g_bq76940_watch.cell_mv[9] >> 8u);
  data[4] = (uint8_t)(g_bq76940_watch.cell_mv[10] & 0xFFu);
  data[5] = (uint8_t)(g_bq76940_watch.cell_mv[10] >> 8u);
  data[6] = (uint8_t)(g_bq76940_watch.cell_mv[11] & 0xFFu);
  data[7] = (uint8_t)(g_bq76940_watch.cell_mv[11] >> 8u);
  (void)BQ76940_CAN_SendFrame(BQ76940_CAN_ID_CELLS_9_12, data);

  data[0] = (uint8_t)(g_bq76940_watch.cell_mv[12] & 0xFFu);
  data[1] = (uint8_t)(g_bq76940_watch.cell_mv[12] >> 8u);
  data[2] = (uint8_t)(g_bq76940_control.balance_cell_min_mv & 0xFFu);
  data[3] = (uint8_t)(g_bq76940_control.balance_cell_min_mv >> 8u);
  data[4] = (uint8_t)(g_bq76940_control.balance_cell_max_mv & 0xFFu);
  data[5] = (uint8_t)(g_bq76940_control.balance_cell_max_mv >> 8u);
  data[6] = (uint8_t)(g_bq76940_control.balance_cell_delta_mv & 0xFFu);
  data[7] = (uint8_t)(g_bq76940_control.balance_cell_delta_mv >> 8u);
  (void)BQ76940_CAN_SendFrame(BQ76940_CAN_ID_CELLS_13, data);
}

static HAL_StatusTypeDef BQ76940_CAN_SendFrame(uint32_t stdId, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
  uint32_t startTick;
  HAL_StatusTypeDef status;

  txHeader.StdId = stdId;
  txHeader.ExtId = 0u;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8u;
  txHeader.TransmitGlobalTime = DISABLE;

  startTick = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0u)
  {
    if ((HAL_GetTick() - startTick) > 5u)
    {
      g_bq76940_control.last_can_tx_status = (uint32_t)HAL_TIMEOUT;
      g_bq76940_control.can_tx_fail_count++;
      return HAL_TIMEOUT;
    }
  }

  status = HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)data, &txMailbox);
  g_bq76940_control.last_can_tx_status = (uint32_t)status;
  if (status == HAL_OK)
  {
    g_bq76940_control.can_tx_count++;
  }
  else
  {
    g_bq76940_control.can_tx_fail_count++;
  }

  return status;
}

static void BQ76940_ServiceDebugPauseLoop(void)
{
  HAL_StatusTypeDef sysStatStatus;
  HAL_StatusTypeDef sysCtrl2Status;
  uint8_t sysStat;
  uint8_t sysCtrl2;

  sysStat = 0u;
  sysCtrl2 = 0u;
  sysStatStatus = BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_STAT, &sysStat);
  sysCtrl2Status = BQ76940_ReadRegisterFallback(BQ76940_REG_SYS_CTRL2, &sysCtrl2);

  if (sysStatStatus == HAL_OK)
  {
    g_bq76940_watch.sys_stat_effective = sysStat;
    g_bq76940_watch.sys_stat_crc = sysStat;
    g_bq76940_watch.sys_stat_plain = sysStat;
    g_bq76940_watch.reg_value[BQ76940_REG_SYS_STAT] = sysStat;
    g_bq76940_watch.reg_source[BQ76940_REG_SYS_STAT] = 1u;

    if (((sysStat & BQ76940_SYS_STAT_CC_READY) != 0u) &&
        (g_bq76940_control.auto_clear_cc_ready != 0u))
    {
      (void)BQ76940_WriteRegisterFallback(BQ76940_REG_SYS_STAT, BQ76940_SYS_STAT_CC_READY);
    }
  }

  if (sysCtrl2Status == HAL_OK)
  {
    g_bq76940_watch.reg_value[BQ76940_REG_SYS_CTRL2] = sysCtrl2;
    g_bq76940_watch.reg_source[BQ76940_REG_SYS_CTRL2] = 1u;
    g_bq76940_watch.dsg_on = ((sysCtrl2 & BQ76940_SYS_CTRL2_DSG_ON) != 0u) ? 1u : 0u;
  }

  g_bq76940_watch.alert_afe_level = (uint8_t)HAL_GPIO_ReadPin(ALERT_AFE_GPIO_Port, ALERT_AFE_Pin);
  g_bq76940_watch.alert_gauge_level = (uint8_t)HAL_GPIO_ReadPin(ALERT_GAUGE_GPIO_Port, ALERT_GAUGE_Pin);
  g_bq76940_watch.last_hal_i2c_error = HAL_I2C_GetError(&hi2c2);

  BQ76940_UpdateDebugSnapshot(sysStatStatus, sysCtrl2Status);
}

static void BQ76940_UpdateDebugSnapshot(HAL_StatusTypeDef crcReadStatus,
                                        HAL_StatusTypeDef plainReadStatus)
{
  BQ76940_DebugSnapshot_t snapshot;
  uint8_t sysStat;
  uint8_t sysCtrl2;
  uint8_t historyNext;

  sysStat = g_bq76940_watch.sys_stat_effective;
  sysCtrl2 = 0u;
  if (g_bq76940_watch.reg_source[BQ76940_REG_SYS_CTRL2] != 0u)
  {
    sysCtrl2 = g_bq76940_watch.reg_value[BQ76940_REG_SYS_CTRL2];
  }

  snapshot.tick_ms = HAL_GetTick();
  snapshot.pack_mv = g_bq76940_watch.pack_mv;
  snapshot.last_hal_i2c_error = g_bq76940_watch.last_hal_i2c_error;
  snapshot.sys_stat = sysStat;
  snapshot.sys_ctrl2 = sysCtrl2;
  snapshot.alert_afe_pin_level = g_bq76940_watch.alert_afe_level;
  snapshot.alert_gauge_pin_level = g_bq76940_watch.alert_gauge_level;
  snapshot.alert_afe_asserted_low = (g_bq76940_watch.alert_afe_level == (uint8_t)GPIO_PIN_RESET) ? 1u : 0u;
  snapshot.alert_gauge_asserted_low = (g_bq76940_watch.alert_gauge_level == (uint8_t)GPIO_PIN_RESET) ? 1u : 0u;
  snapshot.alert_afe_asserted_high = (g_bq76940_watch.alert_afe_level == (uint8_t)GPIO_PIN_SET) ? 1u : 0u;
  snapshot.alert_gauge_asserted_high = (g_bq76940_watch.alert_gauge_level == (uint8_t)GPIO_PIN_SET) ? 1u : 0u;
  snapshot.comm_error = ((crcReadStatus != HAL_OK) && (plainReadStatus != HAL_OK)) ? 1u : 0u;
  snapshot.using_plain_fallback = g_bq76940_watch.using_plain_fallback;
  snapshot.fault_mask = (uint8_t)(sysStat & BQ76940_SYS_STAT_FAULT_MASK);
  snapshot.ocd = ((sysStat & BQ76940_SYS_STAT_OCD) != 0u) ? 1u : 0u;
  snapshot.scd = ((sysStat & BQ76940_SYS_STAT_SCD) != 0u) ? 1u : 0u;
  snapshot.ov = ((sysStat & BQ76940_SYS_STAT_OV) != 0u) ? 1u : 0u;
  snapshot.uv = ((sysStat & BQ76940_SYS_STAT_UV) != 0u) ? 1u : 0u;
  snapshot.ovrd_alert = ((sysStat & BQ76940_SYS_STAT_OVRD_ALERT) != 0u) ? 1u : 0u;
  snapshot.device_xready = ((sysStat & BQ76940_SYS_STAT_DEVICE_XREADY) != 0u) ? 1u : 0u;
  snapshot.cc_ready = ((sysStat & BQ76940_SYS_STAT_CC_READY) != 0u) ? 1u : 0u;
  snapshot.chg_on = ((sysCtrl2 & BQ76940_SYS_CTRL2_CHG_ON) != 0u) ? 1u : 0u;
  snapshot.dsg_on = ((sysCtrl2 & BQ76940_SYS_CTRL2_DSG_ON) != 0u) ? 1u : 0u;
  snapshot.cc_oneshot = ((sysCtrl2 & BQ76940_SYS_CTRL2_CC_ONESHOT) != 0u) ? 1u : 0u;
  snapshot.cc_en = ((sysCtrl2 & BQ76940_SYS_CTRL2_CC_EN) != 0u) ? 1u : 0u;
  snapshot.cellbal1 = g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL1];
  snapshot.cellbal2 = g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL2];
  snapshot.cellbal3 = g_bq76940_watch.reg_value[BQ76940_REG_CELLBAL3];
  snapshot.balance_active_cell = g_bq76940_control.balance_active_cell;
  snapshot.fet_recover_count = g_bq76940_watch.fet_recover_count;
  snapshot.recover_block_reason = g_bq76940_control.last_recover_block_reason;
  snapshot.led_bar_mask = g_bq76940_watch.led_bar_mask;
  snapshot.led_breath_duty = g_bq76940_watch.led_breath_duty;

  g_bq76940_debug.now = snapshot;

  historyNext = g_bq76940_debug.history_next;
  g_bq76940_debug.history[historyNext] = snapshot;
  historyNext++;
  if (historyNext >= BQ76940_DEBUG_HISTORY_COUNT)
  {
    historyNext = 0u;
  }

  g_bq76940_debug.history_next = historyNext;
  if (g_bq76940_debug.history_count < BQ76940_DEBUG_HISTORY_COUNT)
  {
    g_bq76940_debug.history_count++;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
