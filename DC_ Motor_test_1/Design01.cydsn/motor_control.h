#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* Trial values: calibrate with the normal battery, load and surface.
 * PWM 180 leaves correction headroom on BOTH motors. */
#define DRIVE_PWM 180
#define TURN_PWM 170
#define MIN_DRIVE_PWM 115
#define MAX_DRIVE_PWM 230
#define SYNC_KP 0.15f
#define MAX_SYNC_CORRECTION 50
#define PWM_RAMP_PER_SECOND 200u
#define SLOWDOWN_PULSES 450
#define MOTOR_STOP_TOLERANCE 40
#define MOTOR_SETTLE_MS 160u
#define MOTOR_STALL_MS 1000u
#define MOTOR_TIMEOUT_MS 15000u
#define MOTOR_MAX_MISMATCH 300
#define MOTOR_REVERSE_TOLERANCE 20

typedef struct {
    unsigned long started, progress_ms[2], quiet_since;
    int checkpoint[2], quiet_count[2];
    unsigned char done[2], settling;
    int pwm[2];
    unsigned char fault;
} MotorControl;

/* Faults: 1 wrong direction, 2 excessive wheel mismatch, 3 stall,
 * 4 segment timeout, 5 final position outside tolerance. */
static int motor_clamp(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static void motor_control_reset(MotorControl *s, unsigned long now)
{
    memset(s, 0, sizeof(*s));
    s->started = s->progress_ms[0] = s->progress_ms[1] = now;
}

/* p1/p2 are signed progress in each wheel's COMMANDED direction.
 * Return 1 only when both wheels have reached and settled at the target.
 * This is travel synchronization, not absolute ground-position sensing. */
static int motor_control_step(MotorControl *s, unsigned long now,
                              int p1, int p2, int target, int cruise)
{
    int p[2] = {p1, p2};
    int trim = motor_clamp((int)(SYNC_KP * (p1 - p2)),
                          -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
    unsigned long elapsed = (uint32)(now - s->started);
    int ramp;
    int i;
    s->pwm[0] = s->pwm[1] = 0;
    if (s->fault) return 0;
    if (p1 < -MOTOR_REVERSE_TOLERANCE || p2 < -MOTOR_REVERSE_TOLERANCE)
        s->fault = 1;
    else if (abs(p1 - p2) > MOTOR_MAX_MISMATCH) s->fault = 2;
    else if (elapsed >= MOTOR_TIMEOUT_MS) s->fault = 4;
    if (s->fault) return 0;

    for (i = 0; i < 2; i++) {
        if (p[i] >= target) s->done[i] = 1;
        if (p[i] > target + MOTOR_STOP_TOLERANCE) s->fault = 5;
    }
    if (s->fault) return 0;
    if (s->done[0] && s->done[1]) {
        if (!s->settling || p1 != s->quiet_count[0] || p2 != s->quiet_count[1]) {
            s->settling = 1;
            s->quiet_since = now;
            s->quiet_count[0] = p1;
            s->quiet_count[1] = p2;
            return 0;
        }
        if ((uint32)(now - s->quiet_since) < MOTOR_SETTLE_MS) return 0;
        if (abs(p1 - target) > MOTOR_STOP_TOLERANCE ||
            abs(p2 - target) > MOTOR_STOP_TOLERANCE) {
            s->fault = 5;
            return 0;
        }
        return 1;
    }
    for (i = 0; i < 2; i++) {
        if (p[i] - s->checkpoint[i] >= 4) {
            s->checkpoint[i] = p[i];
            s->progress_ms[i] = now;
        }
        if (!s->done[i] && (uint32)(now - s->progress_ms[i]) >= MOTOR_STALL_MS)
            s->fault = 3;
    }
    if (s->fault) return 0;
    /* The startup PWM ceiling rises with elapsed time.
     * Slowing or switching off is immediate. */
    ramp = MIN_DRIVE_PWM + (int)(elapsed * PWM_RAMP_PER_SECOND / 1000u);
    ramp = motor_clamp(ramp, MIN_DRIVE_PWM, MAX_DRIVE_PWM);
    for (i = 0; i < 2; i++) {
        int remaining = motor_clamp(target - p[i], 0, SLOWDOWN_PULSES);
        int base = MIN_DRIVE_PWM + (cruise - MIN_DRIVE_PWM) * remaining / SLOWDOWN_PULSES;
        int requested = base + (i == 0 ? -trim : trim);
        if (!s->done[i]) s->pwm[i] = motor_clamp(requested, MIN_DRIVE_PWM, ramp);
    }
    return 0;
}
#endif
