# AFE BQ76940 CAN/I2C Demo

这个工程基于 `STM32F103`，通过 `I2C2` 读取 `BQ76940` 数据，并周期性通过 `CAN1` 发送电池包状态。

## 这次已修改的内容

1. `Core/Src/main.c`
   将 `BQ76940_CAN_LOOPBACK_TEST` 从 `1` 改为 `0`，默认使用真实 CAN 总线模式。
2. `Core/Src/stm32f1xx_it.c`
   补充了 `USB_HP_CAN1_TX_IRQHandler`、`USB_LP_CAN1_RX0_IRQHandler`、`CAN1_SCE_IRQHandler`，让 HAL 的 CAN 中断链路完整。
3. `Core/Src/can.c`
   补充 `CAN1_SCE_IRQn` 的 NVIC 使能，便于错误中断进入 HAL。

## 为什么你在 CAN 分析仪上看不到数据

当前代码原来开启了内部环回：

`Core/Src/main.c`
`#define BQ76940_CAN_LOOPBACK_TEST  (1u)`

在这个模式下：

- MCU 发送的帧只会回到 MCU 自己的接收 FIFO
- 外部 CAN 分析仪看不到板子发出的报文
- 外部 CAN 分析仪发来的报文，MCU 也不会真正从总线上收到

这就是“分析仪发了数据，但板子没有接收或发送到任何数据”的最主要原因。

## 当前 CAN 配置

`Core/Src/can.c` 中当前参数：

- `Prescaler = 1`
- `BS1 = 13TQ`
- `BS2 = 2TQ`
- `SJW = 1TQ`
- `Mode = CAN_MODE_NORMAL`

在当前时钟配置下：

- `SYSCLK = HSI = 8 MHz`
- `APB1 = 8 MHz`
- CAN 位时间 = `1 + 13 + 2 = 16 TQ`
- CAN 波特率 = `8 MHz / 16 = 500 kbps`

所以你的 CAN 分析仪也需要设置为：

- `500 kbps`
- 标准帧
- 非监听模式

如果分析仪开了 `Listen Only / Silent`，它不会给 ACK，MCU 的发送邮箱可能会一直重发，表现出来就像“发不出去”。

## 硬件联调前提

要让 `STM32F103` 和 CAN 分析仪正常通信，需要同时满足：

- MCU 的 `PA11`/`PA12` 连接到了 CAN 收发器，不是直接接分析仪
- 总线上有 `CANH` / `CANL`
- 总线两端有 120 欧终端电阻
- 分析仪与板子共地
- 波特率一致，建议先用 `500 kbps`

如果你的板子上没有 CAN 收发器，只有 MCU 的 `PA11/PA12`，那是不能直接接 CAN 分析仪的。

## 程序发送的 CAN 报文

程序每 `500 ms` 发送一次以下标准帧：

- `0x500` 状态帧
- `0x501` Cell1 ~ Cell4 电压
- `0x502` Cell5 ~ Cell8 电压
- `0x503` Cell9 ~ Cell12 电压
- `0x504` Cell13 + 均衡信息

接收过滤器当前是全开：

- 所有标准 ID 都会进 `FIFO0`

接收到的数据会写到这些观察变量里：

- `g_bq76940_control.can_rx_count`
- `g_bq76940_control.last_can_rx_id`
- `g_bq76940_control.last_can_rx_dlc`
- `g_bq76940_control.last_can_rx_data[8]`

发送状态可以看：

- `g_bq76940_control.can_tx_count`
- `g_bq76940_control.can_tx_fail_count`
- `g_bq76940_control.last_can_tx_status`

## 建议你现在这样排查

1. 先重新下载这版代码。
2. 把 CAN 分析仪设置成 `500 kbps`，并确认不是 `Listen Only`。
3. 确认板子和分析仪之间是通过 CAN 收发器连接，不是直接连 MCU 引脚。
4. 上电后先看分析仪能不能收到 `0x500` ~ `0x504`。
5. 再从分析仪主动发一个标准帧，观察 `g_bq76940_control.can_rx_count` 是否增加。

## 如果你还想继续改

如果你愿意，我下一步可以继续帮你加两类调试能力中的一种：

1. 串口 `printf` 打印 CAN 收发状态
2. 按你分析仪发送的那个具体 ID 和 8 字节数据，增加一段“收到后执行动作/回包”的逻辑

如果你把“分析仪发送的具体报文 ID 和 8 字节内容”发我，我可以直接帮你把接收解析也写进去。
