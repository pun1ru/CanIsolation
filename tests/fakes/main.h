#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
#define HAL_FDCAN_STATE_BUSY 2U
#define HAL_FDCAN_STATE_READY 1U
#define FDCAN_CCCR_INIT 1U
#define FDCAN_PROTOCOL_ERROR_NO_CHANGE 7U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_MASK 2U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_REJECT 2U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_ERROR_WARNING 0x40000U
#define FDCAN_IT_ERROR_PASSIVE 0x20000U
#define FDCAN_IT_BUS_OFF 0x80000U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_STORE_TX_EVENTS 0x00800000U
#define FDCAN_TIMESTAMP_PRESC_1 0U
#define FDCAN_TIMESTAMP_INTERNAL 1U
#define FDCAN_IT_TX_EVT_FIFO_NEW_DATA 0x1000U
#define FDCAN_IT_TX_EVT_FIFO_ELT_LOST 0x4000U
#define FDCAN_TXEFS_EFFL 7U
#define FDCAN_RX_FIFO0 0U
#define DMA_IT_HT 1U
#define __HAL_DMA_DISABLE_IT(handle, interrupt) ((void)(handle), (void)(interrupt))
#define RCC_CFGR_PPRE 0x7000U
#define RCC_HCLK_DIV1 0U
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM2_FORCE_RESET() Test_ResetTimer()
#define __HAL_RCC_TIM2_RELEASE_RESET() ((void)0)

typedef struct { uint32_t CFGR; } RCC_TypeDef;
typedef struct { uint32_t PSC, ARR, EGR, SR, CNT, CR1; } TIM_TypeDef;
extern RCC_TypeDef test_rcc;
extern TIM_TypeDef test_tim2;
#define RCC (&test_rcc)
#define TIM2 (&test_tim2)

void Test_ResetTimer(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);

typedef struct { uint32_t CCCR, TXBTO, TXBRP, TXEFS; } FDCAN_GlobalTypeDef;
typedef struct
{
  FDCAN_GlobalTypeDef *Instance;
  uint32_t State, ErrorCode, LatestTxFifoQRequest;
} FDCAN_HandleTypeDef;
typedef struct { void *hdmarx; } UART_HandleTypeDef;
typedef struct
{
  uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
typedef struct
{
  uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
  uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct
{
  uint32_t Identifier, IdType, RxFrameType, FDFormat, DataLength;
} FDCAN_RxHeaderTypeDef;
typedef struct
{
  uint32_t Identifier, IdType, TxFrameType, FDFormat, DataLength;
  uint32_t TxTimestamp, MessageMarker;
} FDCAN_TxEventFifoTypeDef;
typedef struct { uint32_t BusOff, LastErrorCode; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;

uint32_t HAL_GetTick(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, const FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *, FDCAN_TxEventFifoTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_IsTxBufferMessagePending(const FDCAN_HandleTypeDef *, uint32_t);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);

#endif
