# Termite motor-only test

The active main.c uses UART1, PWM1/PWM2, two quadrature encoders and a system
tick. Camera code, UART2 startup and camera acknowledgement waits are removed.

The common controller in `motor_control.h` adjusts BOTH motors every 20 ms,
for forward/backward movement and turns. Keep that header beside main.c.
Cruising PWM is 180 for translation and 170 for turns. Synchronization uses
signed progress in each wheel's commanded direction, with KP 0.15 and trim
limited to +/-50. At translation cruise, motor 1 leading by 100 counts requests
PWM 165 for motor 1 and 195 for motor 2, once the startup ceiling permits it.
There is no fixed master output and no integral term.

The startup ceiling begins at PWM 115 and rises by 200 PWM units per second,
up to 230. Each wheel's base PWM decreases over its final 450 counts toward
115; synchronization correction still applies, bounded by the running floor
and startup ceiling. Each wheel switches off at its own target. Completion
requires both encoders to stay unchanged for 160 ms and end within 40 counts
of their targets. UART logging is queued and does not wait for transmission.

Defaults are trial values, not hardware calibration. Check the minimum running
PWM under the intended load, then tune cruising PWM and slowdown distance
before increasing synchronization gain. Physical overshoot, wheel slip and
unequal wheel sizes are not eliminated by matching encoder counts. Recheck
turn angles because the turn speed and stopping behavior have changed.

Connect the Termite USB-UART adapter TX to P2[6] (Rx_1), RX to P2[5] (Tx_1),
and common ground, according to the generated pin mapping. Use the existing
UART1 baud/parity settings, and terminate commands with CR or LF. Bluetooth
is not required.

Build/program/reset. Expect:

    MOTOR ONLY: Termite UART1; dual encoder control v3

Send `fd1`, then `bk1` after completion. Also supported: `fd 3`, `lt 90`,
`rt 90`, `repeat 2 [fd1 bk1]`. Commands received during motion are discarded;
send the next command after STOP. Existing software
MAKE/IFELSE/WHILE syntax remains, but there is no sensor variable refresh.

Faults stop both outputs and require a board reset: 1 wrong encoder direction,
2 wheel mismatch over 300 counts, 3 an unfinished wheel makes fewer than four
counts of progress in one second, 4 segment exceeds 15 seconds, 5 position
outside tolerance. Faults do not advance the command queue. The old test1/test2
diagnostic commands are not present in this version.

The combined source is backed up as
`versions/main_with_camera_before_motor_only_2026-10-04.c`.
Run `python tests/test_drive_control.py` for mocked command/controller checks.
Tests cannot verify physical movement. If motor 1 stays still at PWM 230,
resolve its drive/encoder path before tuning KP; removing camera code cannot
repair a stationary motor.
