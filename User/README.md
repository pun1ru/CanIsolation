# CAN1 to CAN2 Latency

`g_can1_to_can2_latency` exposes `sample_count`, `last_us`, `min_us`, `max_us`,
`average_us`, `tx_started_us`, `rx_received_us`, and `last_sequence` to Live Watch.
All times are integer microseconds. Statistics reset on `CommBsp_Init()`.

The start timestamp is captured immediately before CAN1's HAL enqueue call.
The receive timestamp is captured in CAN2's FIFO0 callback, before reading the
message. This measures enqueue, arbitration, retransmission, frame transmission,
and receive interrupt servicing, rather than transceiver propagation alone.
Normal example transmissions on both CAN ports remain enabled.

Only standard classic data frames with ID `0x101`, DLC 8, and the expected
`55 AA Seq 01 02 03 04 05` payload match the current CAN1 attempt. Use this ID
exclusively for this board on the test bus. Each attempt contributes at most one
sample; a software retry starts a new measurement using the same sequence.

The BSP reserves TIM2 as a free-running 32-bit 1 MHz counter without interrupts.
Unsigned subtraction supports a counter wrap (about 71.6 minutes); a single
measurement must be shorter than that. Debugger halts and interrupt delays
affect measured latency, so use Live Watch with the target running continuously.
The average is truncated to whole microseconds; a 64-bit sum avoids early
accumulation overflow.

PA15 and PD0 start high, configured in both `Core/Src/main.c` and the CubeMX IOC.

# CAN2 to CAN1 to CAN2 Forwarding Test

CAN2's periodic standard ID `0x202` frame is now sent twice per successful test:
CAN2 sends `55 AA Seq 01 02 03 04 05`, CAN1 receives and processes that frame,
then immediately enqueues the identical ID, DLC, and payload through CAN2 again.
The forwarding request runs in the receive callback; it does not wait for the
100 ms example-transmission scheduler. If the TX FIFO is full, the main loop
retries the saved forwarding request. The second reception does not forward
again, and foreign IDs or mismatched payloads do not trigger forwarding.

`g_can2_forward_interval` exposes the following Live Watch fields:

| Field | Meaning |
| --- | --- |
| `sample_count` | Completed pairs with both hardware TX events |
| `last_us`, `min_us`, `max_us`, `average_us` | CAN2 first TX SOF to second TX SOF |
| `processing_us` | CAN1 FIFO read begins to forwarding HAL enqueue begins |
| `enqueue_interval_us` | Initial HAL enqueue begins to forwarding enqueue begins |
| `first_tx_timestamp`, `second_tx_timestamp` | Raw 16-bit CAN2 hardware timestamps |
| `last_sequence` | Payload sequence of the last completed pair |
| `forwarded_frames` | Confirmed forwarded transmissions |
| `enqueue_failures` | Failed HAL forwarding enqueue attempts |
| `timeouts` | Measurement attempts that expired |
| `tx_event_errors` | Lost TX events or failed event reads |

The primary interval comes from CAN2's Tx Event FIFO, which captures actual
start-of-frame timestamps for successfully transmitted frames. Internal
message markers identify original and forwarded events without changing the
frame on the bus. At this project's configured 1 Mbps nominal bitrate with
timestamp prescaler 1, each hardware timestamp tick equals 1 microsecond.
This conversion must be updated if the CAN nominal bitrate changes.

The 16-bit counter wraps every 65.536 ms. A TIM2-based 50 ms deadline discards
stalled attempts, avoiding ambiguous multiple wraps and stale measurements.
Both timestamps must arrive within that deadline; event loss invalidates the
attempt. CAN Bus-Off recovery continues independently. Forwarded requests have
their own buffer tracking and asynchronous cancellation; new test requests
wait until the previous forwarding request is cleared. All statistics reset
at startup. Run without breakpoints for timing measurements.

## Target Measurement (2026-10-01)

The firmware was downloaded and verified on the connected STM32G0B1 target,
with both CAN ports on the same bus at 1 Mbps. Live RAM snapshots were read
through ST-LINK without resetting the target between snapshots.

| Metric | Recorded value |
| --- | --- |
| Completed pairs | 1525 |
| Last SOF-to-SOF interval | 164 us |
| Minimum / maximum interval | 156 / 165 us |
| Total interval time | 245187 us |
| Mean interval from total / samples | 160.78 us |
| Integer Live Watch `average_us` | 160 us |
| Last receive-processing-to-enqueue time | 29 us |
| Last enqueue-to-enqueue interval | 252 us |
| Forward enqueue failures / timeouts / TX event errors | 0 / 0 / 0 |
| CAN1 / CAN2 transmit error counters | 0 / 0 |
| CAN1 / CAN2 Bus-Off | 0 / 0 |

An earlier snapshot had 539 successful pairs, so the measurement continued
advancing. The second snapshot had 3050 CAN1 receptions, consistent with one
original and one forwarded `0x202` frame per pair. These are measurements of
the connected setup; the SOF interval includes transmission, interrupt handling,
processing, and arbitration. The longer enqueue interval also includes waiting
for the periodic CAN1 example frame before the first CAN2 SOF.
