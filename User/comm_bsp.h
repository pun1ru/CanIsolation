#ifndef COMM_BSP_H
#define COMM_BSP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t tx_completed;
  uint32_t tx_enqueue_failures;
  uint32_t tx_timeouts;
  uint32_t tx_retries;
  uint32_t recovery_attempts;
  uint32_t recovery_completed;
  uint32_t recovery_failures;
  uint32_t bus_off;
  uint32_t last_error_code;
  uint32_t hal_error;
  uint32_t last_hal_status;
  uint32_t error_interrupts;
  uint32_t tx_error_count;
  uint32_t rx_error_count;
} CommBsp_CanStatus;

extern volatile CommBsp_CanStatus g_can1_status;
extern volatile CommBsp_CanStatus g_can2_status;

/* CAN1 enqueue to CAN2 receive callback, including arbitration and IRQ latency. */
typedef struct
{
  uint32_t sample_count;
  uint32_t last_us;
  uint32_t min_us;
  uint32_t max_us;
  uint32_t average_us;
  uint32_t tx_started_us;
  uint32_t rx_received_us;
  uint32_t last_sequence;
} CommBsp_CanLatency;

extern volatile CommBsp_CanLatency g_can1_to_can2_latency;

/* CAN2 first TX SOF to forwarded TX SOF, using hardware Tx Event timestamps. */
typedef struct
{
  uint32_t sample_count;
  uint32_t last_us;
  uint32_t min_us;
  uint32_t max_us;
  uint32_t average_us;
  uint32_t processing_us;
  uint32_t enqueue_interval_us;
  uint32_t first_tx_timestamp;
  uint32_t second_tx_timestamp;
  uint32_t last_sequence;
  uint32_t forwarded_frames;
  uint32_t enqueue_failures;
  uint32_t timeouts;
  uint32_t tx_event_errors;
} CommBsp_CanForwardInterval;

extern volatile CommBsp_CanForwardInterval g_can2_forward_interval;

void CommBsp_Init(void);
void CommBsp_Task(uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* COMM_BSP_H */
