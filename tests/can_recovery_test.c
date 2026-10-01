#include <assert.h>
#include <stdio.h>
#include "../User/comm_bsp.c"

static FDCAN_GlobalTypeDef regs1, regs2;
FDCAN_HandleTypeDef hfdcan1 = {&regs1};
FDCAN_HandleTypeDef hfdcan2 = {&regs2};
UART_HandleTypeDef huart5;
RCC_TypeDef test_rcc;
TIM_TypeDef test_tim2;
static uint32_t irq_mask, pclk_frequency = 64000000U;
static uint32_t rx_ready;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
static FDCAN_TxEventFifoTypeDef tx_event;
static FDCAN_TxHeaderTypeDef last_tx_header;
static uint8_t last_tx_data[8];
static uint32_t can2_put_index, last_abort_mask, rx_read_delay_us;
static uint32_t timestamp_prescaler, timestamp_operation, can2_notifications;
static uint32_t bus_off, free_level, enqueue_calls, stop_calls, start_calls, abort_calls;
static uint8_t last_sequence;
static HAL_StatusTypeDef stop_result, enqueue_result;

static void Reset(void)
{
  regs1 = (FDCAN_GlobalTypeDef){0};
  regs2 = (FDCAN_GlobalTypeDef){0};
  hfdcan1.State = hfdcan2.State = HAL_FDCAN_STATE_BUSY;
  s_can1_tx = (CommBsp_CanTxState){0};
  s_can2_tx = (CommBsp_CanTxState){0};
  g_can1_status = (CommBsp_CanStatus){0};
  g_can2_status = (CommBsp_CanStatus){0};
  g_can1_to_can2_latency = (CommBsp_CanLatency){0};
  s_can1_latency_armed = 0;
  s_can1_latency_total_us = 0;
  s_can2_forward = (CommBsp_CanForwardState){0};
  g_can2_forward_interval = (CommBsp_CanForwardInterval){0};
  s_can2_forward_total_us = 0;
  s_can2_marker_seed = 0;
  can2_put_index = last_abort_mask = rx_read_delay_us = 0;
  timestamp_prescaler = timestamp_operation = UINT32_MAX;
  can2_notifications = 0;
  test_tim2 = (TIM_TypeDef){0};
  irq_mask = rx_ready = 0;
  bus_off = enqueue_calls = stop_calls = start_calls = abort_calls = 0;
  free_level = 3;
  stop_result = enqueue_result = HAL_OK;
}

static void Step(uint32_t now)
{
  CommBsp_ServiceCan(&hfdcan1, 0x101U, &s_can1_tx, &g_can1_status, now);
}

static void Receive(uint32_t timestamp_us, uint8_t sequence)
{
  static const uint8_t frame[] = {0x55, 0xAA, 0, 1, 2, 3, 4, 5};
  rx_header = (FDCAN_RxHeaderTypeDef){0x101U, FDCAN_STANDARD_ID,
      FDCAN_DATA_FRAME, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8};
  memcpy(rx_data, frame, sizeof(frame));
  rx_data[2] = sequence;
  test_tim2.CNT = timestamp_us;
  rx_ready = 1;
  HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void StepCan2(uint32_t now)
{
  CommBsp_ServiceCan(&hfdcan2, 0x202U, &s_can2_tx, &g_can2_status, now);
}

static void ReceiveForward(uint32_t timestamp_us, uint8_t sequence)
{
  static const uint8_t frame[] = {0x55, 0xAA, 0, 1, 2, 3, 4, 5};
  rx_header = (FDCAN_RxHeaderTypeDef){0x202U, FDCAN_STANDARD_ID,
      FDCAN_DATA_FRAME, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8};
  memcpy(rx_data, frame, sizeof(frame));
  rx_data[2] = sequence;
  test_tim2.CNT = timestamp_us;
  rx_ready = 1;
  HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void EmitTxEvent(uint32_t marker, uint16_t timestamp)
{
  tx_event = (FDCAN_TxEventFifoTypeDef){0x202U, FDCAN_STANDARD_ID,
      FDCAN_DATA_FRAME, FDCAN_CLASSIC_CAN, FDCAN_DLC_BYTES_8, timestamp, marker};
  regs2.TXEFS = 1;
  HAL_FDCAN_TxEventFifoCallback(&hfdcan2, FDCAN_IT_TX_EVT_FIFO_NEW_DATA);
}

int main(void)
{
  Reset();
  Step(100);
  Step(200);
  assert(enqueue_calls == 1 && last_sequence == 0);
  regs1.TXBRP = 0;
  regs1.TXBTO = 1;
  Step(300);
  assert(g_can1_status.tx_completed == 1 && last_sequence == 1);

  Reset();
  Step(100);
  Step(1100);
  assert(abort_calls == 1 && g_can1_status.tx_timeouts == 1);
  Step(1200);
  assert(enqueue_calls == 1);
  regs1.TXBRP = 0;
  Step(1300);
  assert(enqueue_calls == 2 && last_sequence == 0 && g_can1_status.tx_retries == 1);

  Reset();
  Step(100);
  bus_off = 1;
  regs1.CCCR = FDCAN_CCCR_INIT;
  Step(200);
  assert(stop_calls == 0);
  Step(1000);
  assert(stop_calls == 1 && start_calls == 1 && s_can1_tx.recovering == 1);
  Step(3000);
  assert(start_calls == 1 && abort_calls == 0 && enqueue_calls == 1);
  bus_off = 0;
  Step(3100);
  assert(g_can1_status.recovery_completed == 1 && regs1.TXBRP == 1);
  regs1.TXBRP = 0;
  regs1.TXBTO = 1;
  Step(3200);
  assert(g_can1_status.tx_completed == 1 && last_sequence == 1);
  bus_off = 1;
  regs1.CCCR = FDCAN_CCCR_INIT;
  Step(3300);
  assert(g_can1_status.recovery_attempts == 2);

  Reset();
  bus_off = 1;
  regs1.CCCR = FDCAN_CCCR_INIT;
  stop_result = HAL_ERROR;
  Step(1000);
  Step(1100);
  assert(stop_calls == 1 && start_calls == 0 && g_can1_status.recovery_failures == 1);
  stop_result = HAL_OK;
  Step(2000);
  assert(stop_calls == 2 && start_calls == 1);

  Reset();
  free_level = 0;
  Step(100);
  assert(enqueue_calls == 0 && g_can1_status.tx_enqueue_failures == 1);
  free_level = 3;
  enqueue_result = HAL_ERROR;
  Step(200);
  assert(s_can1_tx.pending_buffer == 0);
  enqueue_result = HAL_OK;
  Step(300);
  assert(last_sequence == 0 && s_can1_tx.pending_buffer == 1);

  Reset();
  Step(UINT32_MAX - 499U);
  Step(499U);
  assert(abort_calls == 0);
  Step(500U);
  assert(abort_calls == 1);

  Reset();
  bus_off = 1;
  regs1.CCCR = FDCAN_CCCR_INIT;
  Step(1000);
  CommBsp_ServiceCan(&hfdcan2, 0x202U, &s_can2_tx, &g_can2_status, 1000);
  assert(s_can2_tx.pending_buffer == 1 && g_can2_status.recovery_attempts == 0);

  Reset();
  test_rcc.CFGR = 0;
  CommBsp_InitTimebase();
  assert(test_tim2.PSC == 63 && test_tim2.ARR == UINT32_MAX);
  assert(test_tim2.CR1 == TIM_CR1_CEN && test_tim2.CNT == 0);
  test_rcc.CFGR = 0x4000U;
  pclk_frequency = 32000000U;
  CommBsp_InitTimebase();
  assert(test_tim2.PSC == 63);
  test_rcc.CFGR = 0;
  pclk_frequency = 64000000U;

  Reset();
  test_tim2.CNT = 1000;
  Step(100);
  Receive(1125, 0);
  assert(g_can1_to_can2_latency.sample_count == 1);
  assert(g_can1_to_can2_latency.last_us == 125);
  assert(g_can1_to_can2_latency.tx_started_us == 1000);
  assert(g_can1_to_can2_latency.rx_received_us == 1125);
  Receive(1140, 0);
  assert(g_can1_to_can2_latency.sample_count == 1);
  regs1.TXBRP = 0;
  regs1.TXBTO = 1;
  test_tim2.CNT = 2000;
  Step(200);
  Receive(2010, 0);
  assert(g_can1_to_can2_latency.sample_count == 1);
  Receive(2200, 1);
  assert(g_can1_to_can2_latency.sample_count == 2);
  assert(g_can1_to_can2_latency.last_sequence == 1);
  assert(g_can1_to_can2_latency.min_us == 125);
  assert(g_can1_to_can2_latency.max_us == 200);
  assert(g_can1_to_can2_latency.average_us == 162);

  Reset();
  test_tim2.CNT = UINT32_MAX - 49U;
  Step(100);
  Receive(75, 0);
  assert(g_can1_to_can2_latency.last_us == 125);

  Reset();
  enqueue_result = HAL_ERROR;
  Step(100);
  Receive(100, 0);
  assert(g_can1_to_can2_latency.sample_count == 0);
  enqueue_result = HAL_OK;
  irq_mask = 1;
  Step(200);
  assert(irq_mask == 1);
  irq_mask = 0;
  Receive(100, 0);
  rx_header.Identifier = 0x202U;
  s_can1_latency_armed = rx_ready = 1;
  HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(g_can1_to_can2_latency.sample_count == 1 && s_can1_latency_armed == 1);
  rx_header.Identifier = 0x101U;
  rx_data[7] = 0x99;
  rx_ready = 1;
  HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(g_can1_to_can2_latency.sample_count == 1 && s_can1_latency_armed == 1);
  rx_data[7] = 5;
  rx_header.DataLength = 7;
  rx_ready = 1;
  HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(g_can1_to_can2_latency.sample_count == 1 && s_can1_latency_armed == 1);

  Reset();
  test_tim2.CNT = 1000;
  Step(100);
  Step(1100);
  regs1.TXBRP = 0;
  test_tim2.CNT = 5000;
  Step(1200);
  Receive(5130, 0);
  assert(g_can1_to_can2_latency.last_us == 130);

  Reset();
  assert(CommBsp_ConfigCan(&hfdcan2) == HAL_OK);
  assert(timestamp_prescaler == FDCAN_TIMESTAMP_PRESC_1);
  assert(timestamp_operation == FDCAN_TIMESTAMP_INTERNAL);
  assert((can2_notifications & FDCAN_IT_TX_EVT_FIFO_NEW_DATA) != 0);
  assert((can2_notifications & FDCAN_IT_TX_EVT_FIFO_ELT_LOST) != 0);

  Reset();
  test_tim2.CNT = 1000;
  StepCan2(100);
  assert(last_tx_header.MessageMarker == 0 && s_can2_tx.pending_buffer == 1);
  assert(last_tx_header.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
  EmitTxEvent(0, 1010);
  ReceiveForward(1100, 3);
  assert(enqueue_calls == 1);
  regs2.TXBRP &= ~1U;
  regs2.TXBTO |= 1;
  rx_read_delay_us = 7;
  ReceiveForward(1130, 0);
  assert(enqueue_calls == 2 && last_tx_header.MessageMarker == 1);
  assert(s_can2_forward.pending_buffer == 2 && s_can2_tx.pending_buffer == 1);
  assert(last_tx_header.Identifier == 0x202 && last_tx_header.DataLength == 8);
  assert(memcmp(last_tx_data, rx_data, 8) == 0);
  ReceiveForward(1140, 0);
  assert(enqueue_calls == 2);
  StepCan2(200);
  assert(enqueue_calls == 2 && s_can2_tx.sequence == 1);
  regs2.TXBRP &= ~2U;
  regs2.TXBTO |= 2;
  EmitTxEvent(1, 1145);
  assert(g_can2_forward_interval.sample_count == 1);
  assert(g_can2_forward_interval.last_us == 135);
  assert(g_can2_forward_interval.processing_us == 7);
  assert(g_can2_forward_interval.enqueue_interval_us == 137);
  ReceiveForward(1150, 0);
  assert(enqueue_calls == 2);
  CommBsp_ServiceCanForward();
  assert(g_can2_forward_interval.forwarded_frames == 1 && g_can2_status.tx_completed == 2);
  StepCan2(300);
  assert(enqueue_calls == 3 && last_tx_header.MessageMarker == 2);
  EmitTxEvent(0, 1190);
  assert(s_can2_forward.first_seen == 0);
  regs2.TXBRP = 0;
  regs2.TXBTO |= 4;
  ReceiveForward(1200, 1);
  assert(last_tx_header.MessageMarker == 3);
  regs2.TXBRP = 0;
  regs2.TXBTO |= 1;
  EmitTxEvent(3, 1300);
  EmitTxEvent(2, 1200);
  assert(g_can2_forward_interval.sample_count == 2);
  assert(g_can2_forward_interval.min_us == 100);
  assert(g_can2_forward_interval.max_us == 135);
  assert(g_can2_forward_interval.average_us == 117);

  Reset();
  test_tim2.CNT = 1000;
  StepCan2(100);
  free_level = 0;
  ReceiveForward(1130, 0);
  assert(enqueue_calls == 1 && s_can2_forward.phase == COMM_FORWARD_READY);
  free_level = 3;
  enqueue_result = HAL_ERROR;
  CommBsp_ServiceCanForward();
  assert(s_can2_forward.phase == COMM_FORWARD_READY);
  assert(g_can2_forward_interval.enqueue_failures == 1);
  enqueue_result = HAL_OK;
  CommBsp_ServiceCanForward();
  assert(s_can2_forward.phase == COMM_FORWARD_WAIT_EVENTS);
  assert(last_tx_header.MessageMarker == 1);

  Reset();
  test_tim2.CNT = 1000;
  StepCan2(100);
  test_tim2.CNT = 51000;
  CommBsp_ServiceCanForward();
  assert(s_can2_forward.phase == COMM_FORWARD_IDLE && g_can2_forward_interval.timeouts == 1);
  ReceiveForward(51010, 0);
  EmitTxEvent(0, 1000);
  assert(enqueue_calls == 1 && g_can2_forward_interval.sample_count == 0);

  Reset();
  test_tim2.CNT = 1000;
  StepCan2(100);
  regs2.TXBRP = 0;
  regs2.TXBTO = 1;
  ReceiveForward(1130, 0);
  test_tim2.CNT = 51000;
  CommBsp_ServiceCanForward();
  CommBsp_ServiceCanForward();
  assert(abort_calls == 1 && last_abort_mask == 2);
  StepCan2(200);
  assert(enqueue_calls == 2);
  regs2.TXBRP = 0;
  CommBsp_ServiceCanForward();
  StepCan2(300);
  assert(enqueue_calls == 3 && last_tx_header.MessageMarker == 2);
  EmitTxEvent(0, 1000);
  EmitTxEvent(1, 1100);
  assert(g_can2_forward_interval.sample_count == 0);

  Reset();
  test_tim2.CNT = UINT32_MAX - 199U;
  StepCan2(100);
  EmitTxEvent(0, 65500);
  ReceiveForward(UINT32_MAX - 49U, 0);
  test_tim2.CNT = 75;
  EmitTxEvent(1, 100);
  assert(g_can2_forward_interval.last_us == 136);
  assert(g_can2_forward_interval.enqueue_interval_us == 150);

  Reset();
  StepCan2(100);
  HAL_FDCAN_TxEventFifoCallback(&hfdcan2, FDCAN_IT_TX_EVT_FIFO_ELT_LOST);
  ReceiveForward(130, 0);
  assert(g_can2_forward_interval.tx_event_errors == 1);
  assert(enqueue_calls == 1 && g_can2_forward_interval.sample_count == 0);

  Reset();
  StepCan2(100);
  ReceiveForward(COMM_CAN_FORWARD_TIMEOUT_US, 0);
  assert(g_can2_forward_interval.timeouts == 1 && enqueue_calls == 1);

  puts("PASS: recovery, CAN1 latency, hardware TX interval, exact one-time forwarding, event ordering/matching, FIFO/enqueue retry, timeout/cancel, timer rollover, event loss");
  return 0;
}

uint32_t HAL_GetTick(void) { return 0; }
void Test_ResetTimer(void) { test_tim2 = (TIM_TypeDef){0}; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk_frequency; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void Error_Handler(void) { assert(0); }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, const FDCAN_FilterTypeDef *f) { return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d) { return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b) { if (h == &hfdcan2) can2_notifications = a; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *h, uint32_t p) { timestamp_prescaler = p; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *h, uint32_t p) { timestamp_operation = p; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *h, FDCAN_TxEventFifoTypeDef *event)
{
  if (!h->Instance->TXEFS) return HAL_ERROR;
  *event = tx_event;
  h->Instance->TXEFS = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { start_calls++; h->State = HAL_FDCAN_STATE_BUSY; h->Instance->CCCR = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) { stop_calls++; if (stop_result == HAL_OK) h->State = HAL_FDCAN_STATE_READY; return stop_result; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, const FDCAN_TxHeaderTypeDef *head, const uint8_t *data)
{
  enqueue_calls++;
  last_sequence = data[2];
  last_tx_header = *head;
  memcpy(last_tx_data, data, sizeof(last_tx_data));
  assert(irq_mask == 1);
  assert(head->Identifier == (h == &hfdcan1 ? 0x101U : 0x202U));
  assert(head->DataLength == 8 && data[0] == 0x55 && data[1] == 0xAA);
  if (h == &hfdcan1)
  {
    assert(irq_mask == 1 && s_can1_latency_armed == 1);
    assert(s_can1_latency_started_us == test_tim2.CNT);
    assert(s_can1_latency_sequence == data[2]);
  }
  if (enqueue_result == HAL_OK)
  {
    uint32_t buffer = h == &hfdcan2 ? (1U << (can2_put_index++ % 3U)) : 1U;
    h->LatestTxFifoQRequest = buffer;
    h->Instance->TXBRP |= buffer;
    h->Instance->TXBTO &= ~buffer;
  }
  return enqueue_result;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *p) { p->BusOff = h == &hfdcan1 ? bus_off : 0; p->LastErrorCode = 7; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *c) { c->TxErrorCnt = c->RxErrorCnt = 0; return HAL_OK; }
uint32_t HAL_FDCAN_IsTxBufferMessagePending(const FDCAN_HandleTypeDef *h, uint32_t mask) { return (h->Instance->TXBRP & mask) != 0; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h) { return free_level; }
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *h) { return h->LatestTxFifoQRequest; }
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h, uint32_t mask) { assert(mask != 0); last_abort_mask = mask; abort_calls++; return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t fifo) { return rx_ready; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo, FDCAN_RxHeaderTypeDef *head, uint8_t *data)
{
  if (!rx_ready) return HAL_ERROR;
  *head = rx_header;
  memcpy(data, rx_data, sizeof(rx_data));
  test_tim2.CNT += rx_read_delay_us;
  rx_ready = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t size) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t size) { return HAL_OK; }
