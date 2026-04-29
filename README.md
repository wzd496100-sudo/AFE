# AFE BQ76940 CAN/I2C Demo

This project runs on `STM32F103`, reads `BQ76940` data over `I2C2`, and sends pack status over `CAN1`.

## Recent changes

1. Real CAN bus mode is now the default.
   `BQ76940_CAN_LOOPBACK_TEST` is set to `0`.
2. CAN interrupt handlers were added.
   `USB_HP_CAN1_TX_IRQHandler`, `USB_LP_CAN1_RX0_IRQHandler`, and `CAN1_SCE_IRQHandler` now call `HAL_CAN_IRQHandler`.
3. CAN error interrupt was enabled in NVIC.
4. AFE charge control logic was added.
5. Extra CAN debug frames were added.

## CAN settings

Current CAN timing in `Core/Src/can.c`:

- `Prescaler = 1`
- `BS1 = 13TQ`
- `BS2 = 2TQ`
- `SJW = 1TQ`
- `Mode = CAN_MODE_NORMAL`

With the current clock tree:

- `SYSCLK = 8 MHz`
- `APB1 = 8 MHz`
- CAN bit time = `16 TQ`
- CAN bitrate = `500 kbps`

Your CAN analyzer should be configured for:

- `500 kbps`
- standard frame
- normal mode, not `Listen Only`

## Hardware notes

The MCU pins `PA11` and `PA12` must connect to a CAN transceiver.
Do not connect the analyzer directly to MCU pins.

The bus also needs:

- `CANH` / `CANL`
- common ground
- 120 ohm termination at both ends

## CAN transmit frames

The firmware sends one group of frames every `500 ms`.

Application frames:

- `0x500`: pack status
- `0x501`: cell 1 to 4 voltage
- `0x502`: cell 5 to 8 voltage
- `0x503`: cell 9 to 12 voltage
- `0x504`: cell 13 and balance summary

Debug frames:

- `0x505`: fault bits, SYS_CTRL2, charge/discharge block reason, I2C/CAN status, alert flags
- `0x506`: coulomb counter raw value, max cell voltage, charge recover count, discharge recover count, CAN RX/TX error counters

## Charge logic

Charge FET control is handled in `BQ76940_UpdateChargeFet()` in `Core/Src/main.c`.

Behavior:

- At startup, `CC_EN`, `CHG_ON`, and `DSG_ON` are requested.
- If `OV`, `UV`, `OCD`, `SCD`, `DEVICE_XREADY`, or `OVRD_ALERT` is active, charge is turned off.
- If the highest cell voltage reaches `4200 mV`, charge is turned off.
- Charge turns back on only after the highest cell voltage drops to `4150 mV` or lower.

Current charge-related constants:

- `BQ76940_CHARGE_ENABLE = 1`
- `BQ76940_CHARGE_RESUME_CELL_MV = 4150`
- `BQ76940_CHARGE_STOP_CELL_MV = 4200`

## Useful watch variables

CAN:

- `g_bq76940_control.can_tx_count`
- `g_bq76940_control.can_tx_fail_count`
- `g_bq76940_control.last_can_tx_status`
- `g_bq76940_control.can_rx_count`
- `g_bq76940_control.last_can_rx_id`
- `g_bq76940_control.last_can_rx_data[8]`

Charge / discharge:

- `g_bq76940_watch.chg_on`
- `g_bq76940_watch.dsg_on`
- `g_bq76940_watch.cell_mv_max`
- `g_bq76940_control.charge_recover_count`
- `g_bq76940_control.last_charge_block_reason`
- `g_bq76940_control.last_recover_block_reason`

## Block reason values

- `0`: none
- `1`: communication error
- `2`: protection fault
- `3`: override alert
- `4`: auto control disabled
- `5`: cell voltage too high for charge enable

## Quick test flow

1. Flash the firmware.
2. Set the analyzer to `500 kbps`.
3. Confirm the board uses a CAN transceiver.
4. Check that frames `0x500` to `0x506` appear on the bus.
5. Observe `g_bq76940_watch.chg_on` and `0x505` / `0x506` while changing cell voltage conditions.

## Build note

Local build was not completed in this terminal session because `cmake` is not installed in the current shell environment.

## PC CAN monitor

A simple Windows GUI monitor was added under `tools/`.

Files:

- `tools/afe_can_monitor.py`
- `tools/install_can_monitor.bat`
- `tools/start_can_monitor.bat`

Use it like this:

1. Run `tools\install_can_monitor.bat`
2. Run `tools\start_can_monitor.bat`
3. Set interface, channel, bitrate, then click `Connect`

The monitor decodes frames:

- `0x500` to `0x504` application data
- `0x505` to `0x506` debug data

Typical settings:

- `slcan`: channel `COM3`, serial baud `115200`, bitrate `500000`
- `pcan`: channel like `PCAN_USBBUS1`, bitrate `500000`
- `vector`: channel like `0`, bitrate `500000`

What you will see:

- pack voltage
- all 13 cell voltages
- `SYS_STAT` and decoded fault flags
- `SYS_CTRL2`
- charge / discharge FET state
- balance state
- I2C error and CAN TX status
- raw CAN frame log
