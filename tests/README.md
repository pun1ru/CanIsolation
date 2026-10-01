# CAN BSP Tests

These host tests compile the actual BSP against a small HAL fake. They exercise
send completion, delayed cancellation, same-frame retries, Bus-Off recovery,
failed recovery backoff, FIFO/enqueue failures, tick rollover, and independent
CAN channels. They also exercise the 1 MHz timer setup, latency statistics,
frame matching, duplicate rejection, failed enqueue, preserved interrupt masks,
microsecond timer rollover, and timestamp renewal after retry. They do not verify
electrical behavior or hardware recovery timing. Forwarding tests additionally
cover an identical one-time retransmission through CAN2, separate TX buffers,
hardware event pairing and stale markers, event callback ordering, queue-full
and failed-enqueue retries, timeout/cancellation, both timestamp wraps, and
TX event loss. The hardware SOF timestamp interpretation still requires target
validation.

From an x64 Native Tools Command Prompt for Visual Studio, at the project root:

```bat
cl /nologo /std:c11 /Itests/fakes /IUser tests/can_recovery_test.c /Fobuild/can_recovery_test.obj /Febuild/can_recovery_test.exe
build\can_recovery_test.exe
```
