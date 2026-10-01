#include "comm_bsp.h"

#include "main.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart5;

#define COMM_CAN_PERIOD_MS       100U
#define COMM_CAN_TX_TIMEOUT_MS   1000U
#define COMM_CAN_RECOVERY_MS     1000U
#define COMM_UART_PERIOD_MS      1000U
#define COMM_UART_RX_BUFFER_SIZE 256U
#define COMM_CAN_FORWARD_TIMEOUT_US 50000U

static uint8_t s_uart_rx_buffer[COMM_UART_RX_BUFFER_SIZE];
static volatile uint8_t s_uart_tx_busy;
static volatile uint32_t s_uart_rx_bytes;
static volatile uint32_t s_can1_rx_frames;
static volatile uint32_t s_can2_rx_frames;
static uint32_t s_last_can_tick;
static uint32_t s_last_uart_tick;

volatile CommBsp_CanStatus g_can1_status;
volatile CommBsp_CanStatus g_can2_status;
volatile CommBsp_CanLatency g_can1_to_can2_latency;
volatile CommBsp_CanForwardInterval g_can2_forward_interval;

enum
{
  COMM_FORWARD_IDLE,
  COMM_FORWARD_WAIT_RX,
  COMM_FORWARD_READY,
  COMM_FORWARD_WAIT_EVENTS
};

typedef struct
{
  uint32_t started_us;
  uint32_t received_us;
  uint32_t queued_us;
  uint32_t pending_buffer;
  uint16_t first_timestamp;
  uint16_t second_timestamp;
  uint8_t data[8];
  uint8_t phase;
  uint8_t first_marker;
  uint8_t first_seen;
  uint8_t second_seen;
  uint8_t cancel_requested;
} CommBsp_CanForwardState;

static volatile CommBsp_CanForwardState s_can2_forward;
static uint8_t s_can2_marker_seed;
static uint64_t s_can2_forward_total_us;

static volatile uint32_t s_can1_latency_started_us;
static volatile uint8_t s_can1_latency_sequence;
static volatile uint8_t s_can1_latency_armed;
static uint64_t s_can1_latency_total_us;

static void CommBsp_InitTimebase(void)
{
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

  if ((RCC->CFGR & RCC_CFGR_PPRE) != RCC_HCLK_DIV1)
  {
    timer_clock *= 2U;
  }
  if (timer_clock < 1000000U || timer_clock % 1000000U != 0U)
  {
    Error_Handler();
    return;
  }

  /* TIM2 is reserved by this BSP: a free-running 32-bit, 1 MHz timebase. */
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM2_FORCE_RESET();
  __HAL_RCC_TIM2_RELEASE_RESET();
  TIM2->PSC = timer_clock / 1000000U - 1U;
  TIM2->ARR = UINT32_MAX;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0U;
  TIM2->CNT = 0U;
  TIM2->CR1 = TIM_CR1_CEN;
}

static void CommBsp_RecordCanLatency(const FDCAN_RxHeaderTypeDef *header,
                                      const uint8_t *data,
                                      uint32_t received_us)
{
  static const uint8_t tail[] = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U};
  uint32_t elapsed;

  if (header->Identifier != 0x101U || header->IdType != FDCAN_STANDARD_ID ||
      header->RxFrameType != FDCAN_DATA_FRAME || header->FDFormat != FDCAN_CLASSIC_CAN ||
      header->DataLength != FDCAN_DLC_BYTES_8 ||
      data[0] != 0x55U || data[1] != 0xAAU ||
      memcmp(&data[3], tail, sizeof(tail)) != 0 ||
      s_can1_latency_armed == 0U || data[2] != s_can1_latency_sequence)
  {
    return;
  }

  elapsed = (uint32_t)(received_us - s_can1_latency_started_us);
  s_can1_latency_armed = 0U;
  g_can1_to_can2_latency.tx_started_us = s_can1_latency_started_us;
  g_can1_to_can2_latency.rx_received_us = received_us;
  g_can1_to_can2_latency.last_sequence = data[2];
  g_can1_to_can2_latency.last_us = elapsed;
  if (g_can1_to_can2_latency.sample_count == 0U ||
      elapsed < g_can1_to_can2_latency.min_us)
  {
    g_can1_to_can2_latency.min_us = elapsed;
  }
  if (elapsed > g_can1_to_can2_latency.max_us)
  {
    g_can1_to_can2_latency.max_us = elapsed;
  }
  g_can1_to_can2_latency.sample_count++;
  s_can1_latency_total_us += elapsed;
  g_can1_to_can2_latency.average_us =
      (uint32_t)(s_can1_latency_total_us / g_can1_to_can2_latency.sample_count);
}

typedef struct
{
  uint32_t pending_buffer;
  uint32_t tx_tick;
  uint32_t recovery_tick;
  uint8_t sequence;
  uint8_t recovering;
} CommBsp_CanTxState;

static CommBsp_CanTxState s_can1_tx;
static CommBsp_CanTxState s_can2_tx;

static const uint8_t s_uart_example_frame[] = {
  0x55U, 0xAAU, 0x05U, 0x10U, 0x20U, 0x30U, 0x40U, 0x50U
};

static void CommBsp_TryForwardCan(void)
{
  FDCAN_TxHeaderTypeDef header = {0};
  uint8_t data[8];
  HAL_StatusTypeDef result;
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  if (s_can2_forward.phase == COMM_FORWARD_READY &&
      (uint32_t)(TIM2->CNT - s_can2_forward.started_us) >= COMM_CAN_FORWARD_TIMEOUT_US)
  {
    s_can2_forward.phase = COMM_FORWARD_IDLE;
    g_can2_forward_interval.timeouts++;
  }
  if (s_can2_forward.phase != COMM_FORWARD_READY ||
      hfdcan2.State != HAL_FDCAN_STATE_BUSY ||
      (hfdcan2.Instance->CCCR & FDCAN_CCCR_INIT) != 0U ||
      HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    __set_PRIMASK(irq_mask);
    return;
  }

  header.Identifier = 0x202U;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
  header.MessageMarker = s_can2_forward.first_marker | 1U;
  for (uint32_t i = 0U; i < sizeof(data); i++)
  {
    data[i] = s_can2_forward.data[i];
  }

  s_can2_forward.queued_us = TIM2->CNT;
  s_can2_forward.phase = COMM_FORWARD_WAIT_EVENTS;
  result = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
  if (result == HAL_OK)
  {
    s_can2_forward.pending_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2);
    s_can2_forward.cancel_requested = 0U;
  }
  else
  {
    s_can2_forward.phase = COMM_FORWARD_READY;
    g_can2_forward_interval.enqueue_failures++;
  }
  __set_PRIMASK(irq_mask);
}

static void CommBsp_ReceiveCanForward(const FDCAN_RxHeaderTypeDef *header,
                                      const uint8_t *data,
                                      uint32_t received_us)
{
  if (s_can2_forward.phase != COMM_FORWARD_WAIT_RX ||
      header->Identifier != 0x202U || header->IdType != FDCAN_STANDARD_ID ||
      header->RxFrameType != FDCAN_DATA_FRAME || header->FDFormat != FDCAN_CLASSIC_CAN ||
      header->DataLength != FDCAN_DLC_BYTES_8)
  {
    return;
  }
  for (uint32_t i = 0U; i < 8U; i++)
  {
    if (data[i] != s_can2_forward.data[i])
    {
      return;
    }
  }

  s_can2_forward.received_us = received_us;
  s_can2_forward.phase = COMM_FORWARD_READY;
  CommBsp_TryForwardCan();
}

static void CommBsp_ServiceCanForward(void)
{
  uint32_t irq_mask = __get_PRIMASK();

  __disable_irq();
  if (s_can2_forward.phase != COMM_FORWARD_IDLE &&
      (uint32_t)(TIM2->CNT - s_can2_forward.started_us) >= COMM_CAN_FORWARD_TIMEOUT_US)
  {
    s_can2_forward.phase = COMM_FORWARD_IDLE;
    g_can2_forward_interval.timeouts++;
  }

  if (s_can2_forward.pending_buffer != 0U)
  {
    if (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, s_can2_forward.pending_buffer) != 0U)
    {
      if (s_can2_forward.phase == COMM_FORWARD_IDLE &&
          s_can2_forward.cancel_requested == 0U &&
          HAL_FDCAN_AbortTxRequest(&hfdcan2, s_can2_forward.pending_buffer) == HAL_OK)
      {
        s_can2_forward.cancel_requested = 1U;
        g_can2_status.tx_timeouts++;
      }
    }
    else
    {
      if ((hfdcan2.Instance->TXBTO & s_can2_forward.pending_buffer) != 0U)
      {
        g_can2_status.tx_completed++;
        g_can2_forward_interval.forwarded_frames++;
      }
      else
      {
        g_can2_status.tx_retries++;
        s_can2_forward.phase = COMM_FORWARD_IDLE;
      }
      s_can2_forward.pending_buffer = 0U;
    }
  }
  CommBsp_TryForwardCan();
  __set_PRIMASK(irq_mask);
}

static HAL_StatusTypeDef CommBsp_ConfigCan(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef filter = {0};
  uint32_t notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
      FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF;

  /* Accept every standard data frame and put it in Rx FIFO 0. */
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0U;
  filter.FilterID2 = 0U;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Do not accept frames not covered by the filter or remote frames. */
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (hfdcan == &hfdcan2)
  {
    /* At the configured 1 Mbps, one CAN bit-time timestamp tick is 1 us. */
    if (HAL_FDCAN_ConfigTimestampCounter(hfdcan, FDCAN_TIMESTAMP_PRESC_1) != HAL_OK ||
        HAL_FDCAN_EnableTimestampCounter(hfdcan, FDCAN_TIMESTAMP_INTERNAL) != HAL_OK)
    {
      return HAL_ERROR;
    }
    notifications |= FDCAN_IT_TX_EVT_FIFO_NEW_DATA | FDCAN_IT_TX_EVT_FIFO_ELT_LOST;
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan, notifications, 0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_FDCAN_Start(hfdcan);
}

static HAL_StatusTypeDef CommBsp_SendCan(FDCAN_HandleTypeDef *hfdcan,
                                         uint32_t identifier,
                                         uint8_t sequence,
                                         uint32_t *pending_buffer)
{
  FDCAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0x55U, 0xAAU, sequence, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U};
  uint32_t irq_mask = __get_PRIMASK();
  HAL_StatusTypeDef result;

  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  /* Publish state and capture the buffer index atomically with the TX request. */
  __disable_irq();
  if (hfdcan == &hfdcan1)
  {
    s_can1_latency_armed = 0U;
    s_can1_latency_sequence = sequence;
    s_can1_latency_started_us = TIM2->CNT;
    s_can1_latency_armed = 1U;
  }
  else if (hfdcan == &hfdcan2)
  {
    if (s_can2_forward.phase != COMM_FORWARD_IDLE || s_can2_forward.pending_buffer != 0U)
    {
      __set_PRIMASK(irq_mask);
      return HAL_BUSY;
    }
    s_can2_forward.first_marker = (uint8_t)(s_can2_marker_seed++ << 1U);
    s_can2_forward.first_seen = s_can2_forward.second_seen = 0U;
    for (uint32_t i = 0U; i < sizeof(data); i++)
    {
      s_can2_forward.data[i] = data[i];
    }
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    header.MessageMarker = s_can2_forward.first_marker;
    s_can2_forward.started_us = TIM2->CNT;
    s_can2_forward.phase = COMM_FORWARD_WAIT_RX;
  }

  result = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data);
  if (result == HAL_OK)
  {
    *pending_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(hfdcan);
  }
  else if (hfdcan == &hfdcan1)
  {
    s_can1_latency_armed = 0U;
  }
  else
  {
    s_can2_forward.phase = COMM_FORWARD_IDLE;
  }
  __set_PRIMASK(irq_mask);
  return result;
}

static void CommBsp_ServiceCan(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t identifier,
                               CommBsp_CanTxState *tx,
                               volatile CommBsp_CanStatus *status,
                               uint32_t now)
{
  FDCAN_ProtocolStatusTypeDef protocol;
  FDCAN_ErrorCountersTypeDef counters;
  HAL_StatusTypeDef result;

  (void)HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol);
  (void)HAL_FDCAN_GetErrorCounters(hfdcan, &counters);
  status->bus_off = protocol.BusOff;
  status->hal_error = hfdcan->ErrorCode;
  status->tx_error_count = counters.TxErrorCnt;
  status->rx_error_count = counters.RxErrorCnt;
  if (protocol.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
  {
    status->last_error_code = protocol.LastErrorCode;
  }

  if (protocol.BusOff != 0U)
  {
    /* Clear INIT once, then let hardware count the required recessive bits.
       Repeated restarts while BO is still set would interrupt recovery. */
    if ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U &&
        (uint32_t)(now - tx->recovery_tick) >= COMM_CAN_RECOVERY_MS)
    {
      tx->recovery_tick = now;
      status->recovery_attempts++;
      result = HAL_OK;
      if (hfdcan->State == HAL_FDCAN_STATE_BUSY)
      {
        result = HAL_FDCAN_Stop(hfdcan);
      }
      if (result == HAL_OK)
      {
        result = HAL_FDCAN_Start(hfdcan);
      }
      status->last_hal_status = result;
      if (result == HAL_OK)
      {
        tx->recovering = 1U;
      }
      else
      {
        status->recovery_failures++;
      }
    }
    return;
  }

  if (tx->recovering != 0U)
  {
    tx->recovering = 0U;
    tx->tx_tick = now;
    status->recovery_completed++;
  }

  if (tx->pending_buffer != 0U)
  {
    if (HAL_FDCAN_IsTxBufferMessagePending(hfdcan, tx->pending_buffer) != 0U)
    {
      if ((uint32_t)(now - tx->tx_tick) >= COMM_CAN_TX_TIMEOUT_MS)
      {
        /* Cancellation is asynchronous; wait for TXBRP to clear before reuse. */
        result = HAL_FDCAN_AbortTxRequest(hfdcan, tx->pending_buffer);
        status->last_hal_status = result;
        tx->tx_tick = now;
        if (result == HAL_OK)
        {
          status->tx_timeouts++;
        }
      }
      return;
    }

    if ((hfdcan->Instance->TXBTO & tx->pending_buffer) != 0U)
    {
      status->tx_completed++;
      tx->sequence++;
    }
    else
    {
      status->tx_retries++;
    }
    tx->pending_buffer = 0U;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
  {
    status->last_hal_status = HAL_BUSY;
    status->tx_enqueue_failures++;
    return;
  }

  if (hfdcan == &hfdcan2 &&
      (s_can2_forward.phase != COMM_FORWARD_IDLE || s_can2_forward.pending_buffer != 0U))
  {
    return;
  }

  result = CommBsp_SendCan(hfdcan, identifier, tx->sequence, &tx->pending_buffer);
  status->last_hal_status = result;
  if (result == HAL_OK)
  {
    tx->tx_tick = now;
  }
  else
  {
    status->tx_enqueue_failures++;
  }
}

void CommBsp_Init(void)
{
  CommBsp_InitTimebase();
  memset(&s_can1_tx, 0, sizeof(s_can1_tx));
  memset(&s_can2_tx, 0, sizeof(s_can2_tx));
  g_can1_status = (CommBsp_CanStatus){0};
  g_can2_status = (CommBsp_CanStatus){0};
  g_can1_to_can2_latency = (CommBsp_CanLatency){0};
  s_can1_latency_armed = 0U;
  s_can1_latency_total_us = 0U;
  s_can2_forward = (CommBsp_CanForwardState){0};
  g_can2_forward_interval = (CommBsp_CanForwardInterval){0};
  s_can2_forward_total_us = 0U;
  s_can2_marker_seed = 0U;

  if (CommBsp_ConfigCan(&hfdcan1) != HAL_OK ||
      CommBsp_ConfigCan(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  memset(s_uart_rx_buffer, 0, sizeof(s_uart_rx_buffer));
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                   s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

  s_uart_tx_busy = 0U;
  s_uart_rx_bytes = 0U;
  s_can1_rx_frames = 0U;
  s_can2_rx_frames = 0U;
  s_last_can_tick = HAL_GetTick();
  s_last_uart_tick = s_last_can_tick;
  s_can1_tx.recovery_tick = s_last_can_tick;
  s_can2_tx.recovery_tick = s_last_can_tick;
}

void CommBsp_Task(uint32_t now)
{
  CommBsp_ServiceCanForward();
  if ((uint32_t)(now - s_last_can_tick) >= COMM_CAN_PERIOD_MS)
  {
    s_last_can_tick = now;
    CommBsp_ServiceCan(&hfdcan1, 0x101U, &s_can1_tx, &g_can1_status, now);
    CommBsp_ServiceCan(&hfdcan2, 0x202U, &s_can2_tx, &g_can2_status, now);
  }

  if ((uint32_t)(now - s_last_uart_tick) >= COMM_UART_PERIOD_MS &&
      s_uart_tx_busy == 0U)
  {
    s_last_uart_tick = now;
    s_uart_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart5,
                              (uint8_t *)s_uart_example_frame,
                              sizeof(s_uart_example_frame)) != HAL_OK)
    {
      s_uart_tx_busy = 0U;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_it)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if ((rx_fifo0_it & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
  {
    uint32_t received_us = TIM2->CNT;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      break;
    }

    if (hfdcan == &hfdcan1)
    {
      s_can1_rx_frames++;
      CommBsp_ReceiveCanForward(&header, data, received_us);
    }
    else if (hfdcan == &hfdcan2)
    {
      s_can2_rx_frames++;
      CommBsp_RecordCanLatency(&header, data, received_us);
    }
  }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_event_it)
{
  FDCAN_TxEventFifoTypeDef event;

  if (hfdcan != &hfdcan2)
  {
    return;
  }
  if ((tx_event_it & FDCAN_IT_TX_EVT_FIFO_ELT_LOST) != 0U)
  {
    g_can2_forward_interval.tx_event_errors++;
    s_can2_forward.phase = COMM_FORWARD_IDLE;
  }

  while ((hfdcan->Instance->TXEFS & FDCAN_TXEFS_EFFL) != 0U)
  {
    if (HAL_FDCAN_GetTxEvent(hfdcan, &event) != HAL_OK)
    {
      g_can2_forward_interval.tx_event_errors++;
      s_can2_forward.phase = COMM_FORWARD_IDLE;
      break;
    }
    if (s_can2_forward.phase == COMM_FORWARD_IDLE ||
        event.Identifier != 0x202U || event.IdType != FDCAN_STANDARD_ID ||
        event.TxFrameType != FDCAN_DATA_FRAME || event.FDFormat != FDCAN_CLASSIC_CAN ||
        event.DataLength != FDCAN_DLC_BYTES_8)
    {
      continue;
    }

    if (event.MessageMarker == s_can2_forward.first_marker)
    {
      s_can2_forward.first_timestamp = (uint16_t)event.TxTimestamp;
      s_can2_forward.first_seen = 1U;
    }
    else if (event.MessageMarker == (uint32_t)(s_can2_forward.first_marker | 1U) &&
             s_can2_forward.phase == COMM_FORWARD_WAIT_EVENTS)
    {
      s_can2_forward.second_timestamp = (uint16_t)event.TxTimestamp;
      s_can2_forward.second_seen = 1U;
    }

    if (s_can2_forward.first_seen != 0U && s_can2_forward.second_seen != 0U)
    {
      /* Keep each attempt below a 16-bit hardware timestamp wrap (65.536 ms). */
      if ((uint32_t)(TIM2->CNT - s_can2_forward.started_us) < COMM_CAN_FORWARD_TIMEOUT_US)
      {
        uint32_t interval = (uint16_t)(s_can2_forward.second_timestamp -
                                       s_can2_forward.first_timestamp);

        g_can2_forward_interval.last_us = interval;
        g_can2_forward_interval.processing_us =
            (uint32_t)(s_can2_forward.queued_us - s_can2_forward.received_us);
        g_can2_forward_interval.enqueue_interval_us =
            (uint32_t)(s_can2_forward.queued_us - s_can2_forward.started_us);
        g_can2_forward_interval.first_tx_timestamp = s_can2_forward.first_timestamp;
        g_can2_forward_interval.second_tx_timestamp = s_can2_forward.second_timestamp;
        g_can2_forward_interval.last_sequence = s_can2_forward.data[2];
        if (g_can2_forward_interval.sample_count == 0U ||
            interval < g_can2_forward_interval.min_us)
        {
          g_can2_forward_interval.min_us = interval;
        }
        if (interval > g_can2_forward_interval.max_us)
        {
          g_can2_forward_interval.max_us = interval;
        }
        g_can2_forward_interval.sample_count++;
        s_can2_forward_total_us += interval;
        g_can2_forward_interval.average_us =
            (uint32_t)(s_can2_forward_total_us / g_can2_forward_interval.sample_count);
      }
      else
      {
        g_can2_forward_interval.timeouts++;
      }
      s_can2_forward.phase = COMM_FORWARD_IDLE;
    }
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t error_status_it)
{
  if (hfdcan == &hfdcan1)
  {
    g_can1_status.error_interrupts |= error_status_it;
  }
  else if (hfdcan == &hfdcan2)
  {
    g_can2_status.error_interrupts |= error_status_it;
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan == &hfdcan1)
  {
    g_can1_status.hal_error = hfdcan->ErrorCode;
  }
  else if (hfdcan == &hfdcan2)
  {
    g_can2_status.hal_error = hfdcan->ErrorCode;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    s_uart_tx_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart != &huart5)
  {
    return;
  }

  s_uart_rx_bytes = size;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                   s_uart_rx_buffer,
                                   sizeof(s_uart_rx_buffer)) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                     s_uart_rx_buffer,
                                     sizeof(s_uart_rx_buffer)) == HAL_OK)
    {
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
  }
}
