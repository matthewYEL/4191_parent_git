# Two IR sensors for forward grid movement

Before building in PSoC Creator:

1. Add two single-bit **Digital Input Pin** components in `TopDesign`, named
   `LEFT` and `RIGHT`. Use **High Impedance Digital** drive mode and disable
   the hardware connection so the firmware can read the pins directly.
2. In the `.cydwr` Pins tab, assign them to the physical pins wired to the two
   sensors' digital outputs (DO). Connect sensor ground to the controller ground.
3. Check each sensor's output on black. In `main.c`, set
   `IR_LEFT_BLACK_LEVEL` and `IR_RIGHT_BLACK_LEVEL` to that output (0 or 1).
   Both default to 0; this is an assumption until checked on the sensors.
4. Generate the application and build. The generated `LEFT.h` and
   `RIGHT.h` headers are required; do not create placeholder hardware APIs.

Mount the sensors side by side in the positions shown by the green circles,
between the longitudinal black lines. Both must see white between horizontal
lines and each must be able to see the next horizontal black line.

For `fd 1`, both motors start even while both sensors see the starting black
line. After both see white continuously for 5 ms, each sensor stops its own
motor on its first black reading: `LEFT` stops motor 1, `RIGHT` stops motor 2.
The other motor continues forward until its own sensor sees black. Each stop
is latched for the rest of that step, even if the sensor later reads white.
There is no 5 ms delay on arrival; the stop happens on the next main-loop poll.
The step finishes after both motors have stopped, without requiring simultaneous
black readings. Encoders correct wheel speeds while both motors are moving;
the remaining motor uses fixed `MASTER_PWM` once the other has stopped. Forward
distance no longer ends at the old encoder target. `fd N` repeats this sequence
once per grid with the existing camera scan between steps. Backward movement
and turns retain their encoder targets.

Bench check with the wheels raised: send `fd 1`, hold both sensors over black
(motors run), move both over white, put only the left over black (motor 1 stops,
motor 2 keeps running), then move the right over black (motor 2 stops). Repeat
with the right sensor arriving first. Move the first sensor back to white and
confirm its motor stays stopped while the other advances. Check `stop`
during travel before testing on the floor. Actual stopping position depends on
speed, sensor placement, and motor coast; tune on the robot.

Run the host regression with `python tests/test_forward_ir.py`. It compiles
the actual firmware against mocked hardware using MSVC. The older
`test_drive_control.py` targets a different, motor-only firmware version.
