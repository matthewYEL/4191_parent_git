# Motor controller: 4 October 2026

Build `DC_ Motor_test_1/Design01.cydsn/main.c`. The `versions/` file is an
unchanged historical copy. UART1 accepts commands through the existing serial
connection; Bluetooth is not required. `MOTOR_ONLY_TEST` is currently 1:
UART2 is not started, camera processing is disabled, and scan requests release
the next command immediately. Translation still checks encoder settling.
Set `MOTOR_ONLY_TEST` to 0 to restore camera behavior. While motor-only mode is
enabled, conditions use stored MAKE variables without refreshing camera data.
Use `fd1`, `bk1`, `lt 90`, `rt 90`, and `repeat 2 [fd1 bk1]` for motor testing.

UART1 output now uses a 1024-byte software queue. Main transmits at most four
bytes per service call, only when the generated UART hardware FIFO has room.
Formatting uses a bounded, caller-local buffer. A full queue drops entire new
messages and increments `uart_log_dropped` in Watch instead of delaying motor
updates. Commands received during movement are discarded through their line
ending; a rejection message is queued after movement stops. Resend when idle.

## Translation behavior

- Both wheels start at PWM 230, ending the boost when both have moved 12 counts.
  During startup, a wheel leading by 60 counts is held off. If both encoders
  have not reached 12 counts in 250 ms, fault 8 stops the robot.
- Cruise uses PWM 180 plus/minus a PI synchronization correction. KP is 0.15;
  KI is 0.04 PWM per count-second. Integral correction is limited to 20 PWM,
  freezes during slowdown, and does not grow into output saturation. Gains
  are fixed settings, not automatically learned.
- Slowdown begins when the leading wheel is within 450 counts of its endpoint.
  Correction is limited so moving outputs stay at or above PWM 115. The floor
  still needs measurement on the loaded robot; it is not a guaranteed speed.
- Each wheel switches off at its own target. Completion requires both wheels
  to have stopped changing counts for 160 ms and to be within 40 counts of
  the requested target. Excessive coast faults instead of advancing the queue.
  There is no automatic reverse correction or assumed active H-bridge braking.
- Encoder subtraction handles rollover of the configured 16-bit counters.

Turns retain the existing master/slave controller. Translation startup,
PI correction, slowdown and settling checks do not apply to turns.

## First hardware check

For isolation, lift the wheels and send the exact standalone command `test1`.
Motor 1 alone is commanded forward at PWM 230 for 500 ms, then switched off.
Send `test2` after it finishes to test motor 2 alone. These explicit tests bypass
PI/mismatch checks and print both raw encoder deltas. Negative counts are
expected for forward motion with the existing wiring. A stationary motor still
stops after 500 ms; it cannot wait indefinitely for encoder pulses. Normal
`fd`/`bk` fault checks are unchanged. Reset first if a fault was already latched.
The startup banner `MOTOR TEST v2` identifies this diagnostic firmware.

1. Lift the wheels, reset, and send `fd1`. Check that both wheels start and
   P1/P2 increase. Then check `bk1` after the camera scan completes.
2. If motor 1 is still stationary, fault 8 is expected. Check its PWM/enable,
   direction wiring, driver and supply. Increasing KI cannot repair that.
   Raw encoder counts remain visible while faulted; manually turning a wheel
   should change its raw count. Reset is required to clear a latched fault.
3. With the normal load and surface, repeat single-grid moves and measure
   distance and drift. Tune one parameter at a time before trying `fd3`.

## Tuning order

1. Establish that both wheels respond reliably. Startup PWM/time and the
   running PWM floor are trial values. Do not relax mismatch limits to hide
   a stationary wheel.
2. Start with KI set to zero if isolating proportional behavior. Tune KP in
   small steps (for example 0.02), then restore a small KI for persistent
   wheel-count bias. Reduce gains if the correction oscillates. Reducing
   cruise PWM can also help establish a repeatable baseline.
3. Increase SLOWDOWN_PULSES to slow earlier. Reduce MIN_DRIVE_PWM only if
   neither wheel stalls under load. Fault 7 means final position exceeded
   tolerance, not that the movement succeeded.
4. Leave STOP_COMPENSATION_PULSES at zero until consistent coast is measured.
   Positive compensation cuts power early. Keep it within the final position
   tolerance unless the tolerance/checking policy is deliberately redesigned.
5. Calibrate PULSES_PER_GRID and PULSES_PER_DIAGONAL using actual measured
   travel after stopping becomes repeatable. Equal counts cannot detect slip
   or unequal wheel diameter. Recheck turn angles separately.

Faults: 1 wrong direction; 2 mismatch over 300 counts; 3 insufficient progress
for one second; 4 segment timeout (15 seconds, including settling); 5 final
mismatch; 6 unsupported negative step count; 7 final position/overshoot;
8 startup without both encoders responding.

## Verification

`python tests/test_drive_control.py` compiles extracted production controller
functions with mocked PSoC I/O using Visual Studio. It checks startup, missing
feedback, correction signs, PI/output limits, slowdown, individual stopping,
settling/coast faults, reset behavior and counter/timer rollover. It is not
a physical motor simulation or proof of stability on the robot.
The tests also cover a full UART FIFO, transmit work limits, queue rollover,
whole-message dropping on overflow, and bounded text formatting.

Integral limiting follows the conditional-integration approach described in
[MathWorks anti-windup documentation](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html).
