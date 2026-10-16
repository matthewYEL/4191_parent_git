/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
/* Add two Digital Input Pin components with these names in TopDesign. */
#include "LEFT.h"
#include "RIGHT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>

char Rx = '\0';
char string_1[200]; //debug
int start_c1 = 0;
int start_c2 = 0;
volatile int request_reset_start = 0;
char rx_buffer[100];
uint8 rx_index = 0;
int cmd_code = 0;

/* Trial interval: mechanical settling plus five camera frames. Tune on hardware.
 * This is not a camera acknowledgement; slow frames may require a longer wait.
 */
#define CAMERA_SETTLE_MS 100u
#define MOTOR_CONTROL_PERIOD_MS 200u
enum { CAMERA_SETTLING, CAMERA_WAIT_RESULT, CAMERA_READY, CAMERA_MOVING };
static volatile uint8 camera_state = CAMERA_SETTLING;
static volatile uint32 camera_ms = 0u;
static uint32 camera_stop_ms = 0u;
static int movement_steps_done = 0;
static void begin_camera_scan(void);

/* Set each level to the corresponding sensor's digital output on black. */
#define IR_LEFT_BLACK_LEVEL  1u
#define IR_RIGHT_BLACK_LEVEL 1u
#define IR_STABLE_MS         5u
static volatile uint8 forward_ir_active = 0u;
static uint8 forward_ir_armed = 0u;
static uint8 forward_ir_pending = 0u;
static uint32 forward_ir_since_ms = 0u;
static uint8 forward_ir_left_done = 0u;
static uint8 forward_ir_right_done = 0u;

/* Separate reverse state keeps the working forward detection unchanged. */
static volatile uint8 backward_ir_active = 0u;
static uint8 backward_ir_armed = 0u;
static uint8 backward_ir_pending = 0u;
static uint32 backward_ir_since_ms = 0u;
static uint8 backward_ir_left_done = 0u;
static uint8 backward_ir_right_done = 0u;

static void camera_tick(void)
{
    camera_ms++;
}

#define PULSES_PER_GRID  2220
#define PULSES_PER_DIAGONAL  3200
#define PULSES_PER_45DEGREE_LEFT 1300 //right
#define PULSES_PER_45DEGREE_RIGHT  1200 //left
#define PULSES_PER_90DEGREE_LEFT  2400
#define PULSES_PER_135DEGREE_LEFT  3800
#define PULSES_PER_180DEGREE_LEFT  5200
#define PULSES_PER_225DEGREE_LEFT  6470
#define PULSES_PER_270DEGREE_LEFT  7800
#define PULSES_PER_315DEGREE_LEFT  9200
#define PULSES_PER_90DEGREE_RIGHT  2400
#define PULSES_PER_135DEGREE_RIGHT  3800
#define PULSES_PER_180DEGREE_RIGHT  5200
#define PULSES_PER_225DEGREE_RIGHT  6470
#define PULSES_PER_270DEGREE_RIGHT  7800
#define PULSES_PER_315DEGREE_RIGHT  9200

#define MASTER_PWM 200   /* fixed PWM for the master (reference) motor */
#define KP 0.3        /* P-controller gain: corrects the slave motor's
                          * PWM based on encoder-count error vs the master,
                          * so both wheels stay in sync and the robot
                          * drives straight instead of curving */

typedef struct {
    char type;
    int param;          /* f/b: signed grid step count (not pulses). l/r: signed pulse target for the PWM control loop. w: unused (0). */
    int repeat_cnt;      /* unused, kept for compatibility */
    int turn_delta_deg;  /* l/r only: signed heading change in degrees (e.g. -45, +135). Unused by f/b/w. */
} TurtleCommand;

TurtleCommand cmd_queue[30];
volatile int cmd_count = 0;
volatile int current_cmd_idx = 0;

typedef struct {
    char var[20];
    char expected[20];
    char true_cmd[30];
    char false_cmd[30];
} IfelseData;

IfelseData if_data[30];

int angle_to_pulses_rt(int angle)
{
    angle = abs(angle);

    switch (angle)
    {
        case 45:
            return PULSES_PER_45DEGREE_RIGHT;

        case 90:
            return PULSES_PER_90DEGREE_RIGHT;

        case 135:
            return PULSES_PER_135DEGREE_RIGHT;

        case 180:
            return PULSES_PER_180DEGREE_RIGHT;

        case 225:
            return PULSES_PER_225DEGREE_RIGHT;

        case 270:
            return PULSES_PER_270DEGREE_RIGHT;

        case 315:
            return PULSES_PER_315DEGREE_RIGHT;

        default:
            return 0;
    }
}

int angle_to_pulses_lt(int angle)
{
    angle = abs(angle);

    switch (angle)
    {
        case 45:
            return PULSES_PER_45DEGREE_LEFT;

        case 90:
            return PULSES_PER_90DEGREE_LEFT;

        case 135:
            return PULSES_PER_135DEGREE_LEFT;

        case 180:
            return PULSES_PER_180DEGREE_LEFT;

        case 225:
            return PULSES_PER_225DEGREE_LEFT;

        case 270:
            return PULSES_PER_270DEGREE_LEFT;

        case 315:
            return PULSES_PER_315DEGREE_LEFT;

        default:
            return 0;
    }
}

/* Is the given heading (a multiple of 45 degrees) a diagonal grid direction? */
int heading_is_diagonal(float heading_deg)
{
    int h = ((int)heading_deg) % 360;
    if (h < 0)
    {
        h += 360;
    }
    return (h % 90) != 0;
}

/* ---- DFPlayer Mini (via UART_3) ---- */
#define DF_TRACK_A 1
#define DF_TRACK_E 2
#define DF_TRACK_I 3
#define DF_TRACK_O 4
#define DF_TRACK_U 5

static void df_send_command(uint8 cmd, uint8 param_hi, uint8 param_lo)
{
    uint8 frame[10];
    uint16 checksum;

    frame[0] = 0x7E;
    frame[1] = 0xFF;
    frame[2] = 0x06;
    frame[3] = cmd;
    frame[4] = 0x00;  
    frame[5] = param_hi;
    frame[6] = param_lo;

    checksum = (uint16)(0 - (frame[1] + frame[2] + frame[3] + frame[4] + frame[5] + frame[6]));

    frame[7] = (uint8)(checksum >> 8);
    frame[8] = (uint8)(checksum & 0xFF);
    frame[9] = 0xEF;

    for (int i = 0; i < 10; i++)
    {
        UART_3_PutChar(frame[i]);
    }
}

static void df_set_volume(uint8 volume /* 0-30 */)
{
    df_send_command(0x06, 0x00, volume);
}

static void df_play_track(uint8 track_num)
{
    df_send_command(0x03, 0x00, track_num);
}

static void play_vowel_sound(char letter)
{
    switch (letter)
    {
        case 'A': df_play_track(DF_TRACK_A); CyDelay(100); break;
        case 'E': df_play_track(DF_TRACK_E); CyDelay(100); break;
        case 'I': df_play_track(DF_TRACK_I); CyDelay(100); break;
        case 'O': df_play_track(DF_TRACK_O); CyDelay(100); break;
        case 'U': df_play_track(DF_TRACK_U); CyDelay(100); break;
        default: return;
    }
}

/* =========================================================
 * SHARED HELPER FUNCTIONS
 * (extracted to remove duplication that used to be spread
 *  across parse_command_string / parse_repeat_content /
 *  parse_branch / parse_while_body / ISR_Handler_1 / main)
 * ========================================================= */

/* sprintf + UART_1_PutString, combined into one call */
void uart_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsprintf(string_1, fmt, args);
    va_end(args);
    UART_1_PutString(string_1);
}

/* stop both drive motors */
void stop_motors(void)
{
    forward_ir_active = 0u;
    backward_ir_active = 0u;
    Motor_1_IN_1_Write(0);
    Motor_1_IN_2_Write(0);
    Motor_2_IN_3_Write(0);
    Motor_2_IN_4_Write(0);
}

/*
 * Read one whitespace-delimited token starting at p into out
 * (truncated to out_size-1 chars), skip any leading/trailing
 * spaces, and return the position right after the token.
 */
char* read_token(char *p, char *out, int out_size)
{
    while (*p == ' ')
    {
        p++;
    }

    /* Accept fd3/bk3 as fd 3/bk 3. Return the command now and leave
     * its count for the next token read, including inside command bodies.
     * Only split a complete numeric suffix, not names such as fd3abc.
     */
    if (out_size >= 3 &&
        (strncmp(p, "fd", 2) == 0 || strncmp(p, "bk", 2) == 0) &&
        isdigit((unsigned char)p[2]))
    {
        char *end = p + 2;
        while (isdigit((unsigned char)*end)) end++;
        if (*end == ' ' || *end == '\0')
        {
            out[0] = p[0];
            out[1] = p[1];
            out[2] = '\0';
            return p + 2;
        }
    }

    int j = 0;
    while (*p != ' ' && *p != '\0' && j < out_size - 1)
    {
        out[j++] = *p;
        p++;
    }
    out[j] = '\0';

    while (*p == ' ')
    {
        p++;
    }

    return p;
}

/*
 * Turn a "fd/bk/lt/rt <value>" pair into one TurtleCommand and
 * append it to queue[*count]. Returns 1 if a command was added,
 * 0 if the queue was full or cmd_name was not recognized.
 */
int append_simple_command(TurtleCommand *queue, volatile int *count, int max_count,
                           const char *cmd_name, int value)
{
    if (*count >= max_count)
    {
        return 0;
    }

    queue[*count].repeat_cnt = 1;
    queue[*count].turn_delta_deg = 0;

    if (strcmp(cmd_name, "fd") == 0 || strcmp(cmd_name, "bk") == 0)
    {
        /* Store the raw grid-step count (signed for direction).
         * Forward and backward steps both end at the next IR line
         * encountered in the commanded direction. */
        int forward = (cmd_name[0] == 'f');

        queue[*count].type = forward ? 'f' : 'b';
        queue[*count].param = forward ? -value : value;
    }

    else if (strcmp(cmd_name, "lt") == 0 || strcmp(cmd_name, "rt") == 0)
    {
        int pulses;
        pulses = angle_to_pulses_rt(value);

        if (pulses == 0)
        {
            /* angle not in the supported 45-degree table -> skip this command */
            uart_printf("INVALID ANGLE: %d (use 45/90/135/180/225/270/315)\r\n", value);
            return 0;
        }

        int left = (cmd_name[0] == 'l');

        queue[*count].type = left ? 'l' : 'r';
        queue[*count].param = left ? -pulses : pulses;
        queue[*count].turn_delta_deg = left ? -value : value;
    }
    else
    {
        return 0;
    }

    (*count)++;
    return 1;
}

/* function prototype */
char* find_matching_bracket(char *start);

//make
typedef struct {
    char name[20];
    char value[20];
} TurtleVariable;

/* Camera updates :vowel automatically; no initial MAKE is required. */
TurtleVariable variables[10];
int variable_count = 0;

char while_var_name[20];
char while_expected_value[20];
char while_body[100];

int while_condition_valid = 0;
TurtleCommand while_cmd_queue[30];
int while_cmd_count = 0;
int while_cmd_idx = 0;
int executing_while = 0;

#define IF_QUEUE_SIZE 15
TurtleCommand if_cmd_queue[IF_QUEUE_SIZE];
int if_cmd_count = 0;
int if_cmd_idx = 0;
int executing_if = 0;

/*
 * Actual physical heading in degrees (0 = whatever direction the
 * turtle starts facing, increases clockwise). Updated at runtime,
 * only when an l/r command finishes executing - not when it's
 * parsed/queued. Forward and backward grid steps use IR line detection
 * for their stopping positions rather than heading-based pulse targets.
 */
float current_heading_deg = 0.0f;

volatile int waiting_for_uart = 0;
volatile int uart_msg_received = 0;

/* =========================================================
 * EEPROM SAVE / RESUME STATE
 *
 * The command still enters through UART_1 / Bluetooth exactly as before.
 * Software copies the accepted UART command and execution checkpoint into
 * the PSoC 5LP internal EEPROM; EEPROM needs no external wiring.
 *
 * The EEPROM example project supplied with this project targets
 * CY8C5888LTI-LP097, which has 0x800 (2048) bytes of EEPROM.
 * Two 640-byte banks are used so the previous valid checkpoint survives
 * if power disappears while the new bank is being programmed.
 * ========================================================= */
#define EE_MAGIC_0                 0x45u   /* 'E' */
#define EE_MAGIC_1                 0x32u   /* '2' */
#define EE_FORMAT_VERSION          2u
#define EE_ROW_SIZE                16u
#define EE_BANK_ROWS               40u
#define EE_BANK_BYTES              (EE_BANK_ROWS * EE_ROW_SIZE)
#define EE_BANK_A_FIRST_ROW        0u
#define EE_BANK_B_FIRST_ROW        EE_BANK_ROWS
#define EE_HEADER_BYTES            16u
/* Fields after program text: count/index, steps, heading, flags, body indices/counts, variable count. */
#define EE_RUNTIME_BYTES           12u
#define EE_PROGRAM_MAX             (sizeof(rx_buffer) - 1u)

#if defined(CYDEV_EE_SIZE) && (CYDEV_EE_SIZE < (2u * EE_BANK_BYTES))
#error "EEPROM checkpoint layout requires at least 1280 bytes of internal EEPROM"
#endif

/* Header offsets. Header row is written LAST as the commit marker. */
#define EE_H_MAGIC0                0u
#define EE_H_MAGIC1                1u
#define EE_H_VERSION               2u
#define EE_H_ACTIVE                3u
#define EE_H_SEQ_LO                4u
#define EE_H_SEQ_HI                5u
#define EE_H_LEN_LO                6u
#define EE_H_LEN_HI                7u
#define EE_H_CRC_LO                8u
#define EE_H_CRC_HI                9u

#define EE_FLAG_EXEC_IF            0x01u
#define EE_FLAG_EXEC_WHILE         0x02u
#define EE_FLAG_WAIT_WHILE         0x04u
#define EE_FLAG_IF_BRANCH_VALID    0x08u
#define EE_FLAG_IF_BRANCH_TRUE     0x10u

static volatile uint8 eeprom_save_requested = 0u;
static uint8 eeprom_active_bank = 0xFFu;
static uint16 eeprom_sequence = 0u;
static uint8 eeprom_buffer[EE_BANK_BYTES];
static char eeprom_program_text[sizeof(rx_buffer)] = {0};
static uint8 eeprom_if_branch_valid = 0u;
static uint8 eeprom_if_branch_true = 0u;

static void request_eeprom_checkpoint(void);
static void service_eeprom_checkpoint(void);
static uint8 restore_eeprom_checkpoint(void);
static uint8 rebuild_program_from_text(const char *source);
static volatile uint8 eeprom_boot_ready = 0u;


//lookup function
char* get_variable(char *var_name)
{
    for (int i = 0; i < variable_count; i++)
    {
        if (strcmp(variables[i].name, var_name) == 0)
        {
            return variables[i].value;
            
        }
    }
    return NULL;
}

//compare function
int compare_variable(char *var_name, char *expected_value)
{
    char *actual_value = get_variable(var_name);

    if (actual_value == NULL)
    {
        return 0;
    }

    if (expected_value[0] == ':')
    {
        char *other = get_variable(expected_value + 1);

        expected_value = (other != NULL) ? other : (expected_value + 1);
    }

    return strcmp(actual_value, expected_value) == 0;
}

void parse_command_string(char *text)
{
    char *token = strtok(text, " ");

    while (token != NULL && cmd_count < 30)
    {
        char *cmd_name = token;

        char *param_token = strtok(NULL, " ");
        int target_val = (param_token != NULL) ? atoi(param_token) : 1;

        append_simple_command(cmd_queue, &cmd_count, 30, cmd_name, target_val);

        token = strtok(NULL, " ");
    }
}

void parse_repeat_content(char *text)
{
    char *p = text;

    while (*p != '\0' && cmd_count < 30)
    {
        char cmd_name[10];
        char value_str[20];

        p = read_token(p, cmd_name, sizeof(cmd_name));

        if (cmd_name[0] == '\0')
        {
            break;
        }

        p = read_token(p, value_str, sizeof(value_str));
        int value = atoi(value_str);

        append_simple_command(cmd_queue, &cmd_count, 30, cmd_name, value);
    }
}

void parse_into(char *text, TurtleCommand *queue, int *count, int max_count)
{
    char *p = text;

    while (*p != '\0' && *count < max_count)
    {
        while (*p == ' ')
        {
            p++;
        }

        if (*p == '\0')
        {
            break;
        }

        if (strncmp(p, "repeat", 6) == 0)
        {
            p += 6;

            while (*p == ' ')
            {
                p++;
            }

            int rep_count = atoi(p);

            while (*p != ' ' && *p != '\0')
            {
                p++;
            }

            while (*p == ' ')
            {
                p++;
            }

            if (*p == '[')
            {
                char *repeat_start = p;
                char *repeat_end = find_matching_bracket(p);

                if (repeat_end != NULL)
                {
                    char repeat_content[50];
                    int len = repeat_end - (repeat_start + 1);

                    if (len >= sizeof(repeat_content))
                    {
                        len = sizeof(repeat_content) - 1;
                    }

                    strncpy(repeat_content, repeat_start + 1, len);
                    repeat_content[len] = '\0';

                    for (int r = 0; r < rep_count; r++)
                    {
                        char temp_str[50];
                        strcpy(temp_str, repeat_content);
                        parse_into(temp_str, queue, count, max_count);
                    }

                    p = repeat_end + 1;
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            char cmd_name[10];
            char value_str[20];

            p = read_token(p, cmd_name, sizeof(cmd_name));
            p = read_token(p, value_str, sizeof(value_str));
            int value = atoi(value_str);

            append_simple_command(queue, count, max_count, cmd_name, value);
        }
    }
}

char* find_matching_bracket(char *start)
{
    int depth = 0;

    while (*start != '\0')
    {
        if (*start == '[')
        {
            depth++;
        }
        else if (*start == ']')
        {
            depth--;

            if (depth == 0)
            {
                return start;
            }
        }

        start++;
    }

    return NULL;
}

int while_condition_true(void)
{
    return compare_variable(while_var_name, while_expected_value);
}

/*
 * Called whenever a movement command (f/b/l/r) in the main loop
 * finishes: stops the motors, updates the real heading if a turn
 * just completed (heading_delta_deg is 0 for f/b), advances to the
 * next command (either in while_cmd_queue or cmd_queue), and
 * requests an encoder baseline reset.
 */
void advance_command_index(int heading_delta_deg)
{
    begin_camera_scan();

    TurtleCommand *finished = executing_if    ? &if_cmd_queue[if_cmd_idx]
                              : executing_while ? &while_cmd_queue[while_cmd_idx]
                                                 : &cmd_queue[current_cmd_idx];
    if (finished->type == 'f' || finished->type == 'b')
    {
        movement_steps_done++;

        /* EEPROM: one full grid has been completed.  This is a safe
         * checkpoint; the actual EEPROM write is deferred to main(). */
        request_eeprom_checkpoint();

        if (movement_steps_done < abs(finished->param))
        {
            request_reset_start = 1;
            return;
        }
    }
    movement_steps_done = 0;

    current_heading_deg += (float)heading_delta_deg;
    while (current_heading_deg < 0.0f)   current_heading_deg += 360.0f;
    while (current_heading_deg >= 360.0f) current_heading_deg -= 360.0f;

    if (executing_if)
    {
        if_cmd_idx++;
        if (if_cmd_idx >= if_cmd_count)
        {
            executing_if = 0;
            eeprom_if_branch_valid = 0u;
            current_cmd_idx++;
            if (current_cmd_idx >= cmd_count) 
            {
                current_cmd_idx = 0;
                cmd_count = 0;
            }
        }
    }
    else if (executing_while)
    {
        while_cmd_idx++;
    }
    else
    {
        current_cmd_idx++;
        if (current_cmd_idx >= cmd_count) 
        {
            cmd_count = 0;
            current_cmd_idx = 0;
        }
    }

    request_reset_start = 1;

    /* EEPROM: commit completed movement and queue transitions. */
    request_eeprom_checkpoint();
}

/* Ignore the starting line until both sensors have seen white for 5 ms.
 * Then each sensor immediately stops its own motor on the first black reading.
 * Latch each arrival so a stopped motor stays stopped until the next step.
 */
static void check_forward_ir(void)
{
    uint8 interrupt_state = CyEnterCriticalSection();

    if (forward_ir_active)
    {
        uint8 left_black = (LEFT_Read() == IR_LEFT_BLACK_LEVEL);
        uint8 right_black = (RIGHT_Read() == IR_RIGHT_BLACK_LEVEL);
        uint32 now = camera_ms;

        if (!forward_ir_armed)
        {
            if (left_black || right_black)
            {
                forward_ir_pending = 0u;
            }
            else if (!forward_ir_pending)
            {
                forward_ir_pending = 1u;
                forward_ir_since_ms = now;
            }
            else if ((uint32)(now - forward_ir_since_ms) >= IR_STABLE_MS)
            {
                forward_ir_pending = 0u;
                forward_ir_armed = 1u;
            }
        }

        if (forward_ir_armed)
        {
            if (left_black && !forward_ir_left_done)
            {
                forward_ir_left_done = 1u;
                PWM_1_WriteCompare(0);
                Motor_1_IN_1_Write(0);
                Motor_1_IN_2_Write(0);
            }
            if (right_black && !forward_ir_right_done)
            {
                forward_ir_right_done = 1u;
                PWM_2_WriteCompare(0);
                Motor_2_IN_3_Write(0);
                Motor_2_IN_4_Write(0);
            }
            if (forward_ir_left_done && forward_ir_right_done)
            {
                /* Both have arrived, even if their readings no longer match. */
                advance_command_index(0);
            }
        }
    }

    CyExitCriticalSection(interrupt_state);
}

/* With the same fixed sensor positions, bk moves to the previous line.
 * Ignore the starting line until both sensors see white for 5 ms, then
 * stop each reversing motor immediately when its own sensor sees black.
 */
static void check_backward_ir(void)
{
    uint8 interrupt_state = CyEnterCriticalSection();

    if (backward_ir_active)
    {
        uint8 left_black = (LEFT_Read() == IR_LEFT_BLACK_LEVEL);
        uint8 right_black = (RIGHT_Read() == IR_RIGHT_BLACK_LEVEL);
        uint32 now = camera_ms;

        if (!backward_ir_armed)
        {
            if (left_black || right_black)
            {
                backward_ir_pending = 0u;
            }
            else if (!backward_ir_pending)
            {
                backward_ir_pending = 1u;
                backward_ir_since_ms = now;
            }
            else if ((uint32)(now - backward_ir_since_ms) >= IR_STABLE_MS)
            {
                backward_ir_pending = 0u;
                backward_ir_armed = 1u;
            }
        }

        if (backward_ir_armed)
        {
            if (left_black && !backward_ir_left_done)
            {
                backward_ir_left_done = 1u;
                PWM_1_WriteCompare(0);
                Motor_1_IN_1_Write(0);
                Motor_1_IN_2_Write(0);
            }
            if (right_black && !backward_ir_right_done)
            {
                backward_ir_right_done = 1u;
                PWM_2_WriteCompare(0);
                Motor_2_IN_3_Write(0);
                Motor_2_IN_4_Write(0);
            }
            if (backward_ir_left_done && backward_ir_right_done)
            {
                advance_command_index(0);
            }
        }
    }

    CyExitCriticalSection(interrupt_state);
}

/* Proportional correction for the slave motor's PWM, clamped to 0-255 */
int compute_slave_pwm(int error)
{
    int pwm_slave = MASTER_PWM + (int)(error * KP);

    if (pwm_slave > 255)
    {
        pwm_slave = 255;
    }

    if (pwm_slave < 0)
    {
        pwm_slave = 180;
    }

    return pwm_slave;
}

/* Motor 1 is LEFT; motor 2 is RIGHT. Honour arrival latches on every PWM
 * update so the slower control loop cannot restart an arrived wheel.
 */
static void drive_forward(int relative_c1, int relative_c2)
{
    uint8 interrupt_state = CyEnterCriticalSection();

    if (camera_state == CAMERA_MOVING)
    {
        if (!forward_ir_active)
        {
            forward_ir_armed = 0u;
            forward_ir_pending = 0u;
            forward_ir_left_done = 0u;
            forward_ir_right_done = 0u;
            forward_ir_active = 1u;
        }

        /* While both wheels move, negative encoder counts increase in
         * magnitude going forward. slave - master provides negative feedback.
         * Once the right/master wheel stops, use fixed PWM for the remaining
         * left wheel rather than correcting against a stationary reference.
         */
        int pwm_left = forward_ir_right_done ? MASTER_PWM
                          : compute_slave_pwm(relative_c1 - relative_c2);

        PWM_1_WriteCompare(forward_ir_left_done ? 0 : pwm_left);
        PWM_2_WriteCompare(forward_ir_right_done ? 0 : MASTER_PWM);
        Motor_1_IN_1_Write(0);
        Motor_1_IN_2_Write(forward_ir_left_done ? 0 : 1);
        Motor_2_IN_3_Write(0);
        Motor_2_IN_4_Write(forward_ir_right_done ? 0 : 1);
    }

    CyExitCriticalSection(interrupt_state);
}

/* Reverse has positive encoder travel, so master - slave gives the
 * correction sign needed to slow an ahead left wheel and speed up a
 * lagging one. Once either side arrives, honour its stop latch.
 */
static void drive_backward(int relative_c1, int relative_c2)
{
    uint8 interrupt_state = CyEnterCriticalSection();

    if (camera_state == CAMERA_MOVING)
    {
        if (!backward_ir_active)
        {
            backward_ir_armed = 0u;
            backward_ir_pending = 0u;
            backward_ir_left_done = 0u;
            backward_ir_right_done = 0u;
            backward_ir_active = 1u;
        }

        /* Do not correct against the right/master wheel after it stops. */
        int pwm_left = backward_ir_right_done ? MASTER_PWM
                          : compute_slave_pwm(relative_c2 - relative_c1);

        PWM_1_WriteCompare(backward_ir_left_done ? 0 : pwm_left);
        PWM_2_WriteCompare(backward_ir_right_done ? 0 : MASTER_PWM);
        Motor_1_IN_1_Write(backward_ir_left_done ? 0 : 1);
        Motor_1_IN_2_Write(0);
        Motor_2_IN_3_Write(backward_ir_right_done ? 0 : 1);
        Motor_2_IN_4_Write(0);
    }

    CyExitCriticalSection(interrupt_state);
}

void parse_while_body(char *text)
{
    char *p = text;

    while_cmd_count = 0;

    while (*p != '\0' && while_cmd_count < 30)
    {
        char cmd_name[10];
        char value_str[20];

        p = read_token(p, cmd_name, sizeof(cmd_name));

        if (cmd_name[0] == '\0')
        {
            break;
        }

        p = read_token(p, value_str, sizeof(value_str));
        int value = atoi(value_str);

        append_simple_command(while_cmd_queue, &while_cmd_count, 30, cmd_name, value);
    }
}

/* -------------------------------------------------------------------------
 * Rebuild the parser structures from the original UART program after reboot.
 * This mirrors the existing UART parser but deliberately DOES NOT execute
 * MAKE or LOOKUP side effects.  Variable values are restored from EEPROM
 * after the structural parse is complete.
 * ------------------------------------------------------------------------- */
static uint8 rebuild_program_from_text(const char *source)
{
    char work[sizeof(rx_buffer)];
    char *p;

    if (source == NULL)
    {
        return 0u;
    }

    strncpy(work, source, sizeof(work) - 1u);
    work[sizeof(work) - 1u] = '\0';
    p = work;

    cmd_count = 0;
    current_cmd_idx = 0;
    while_cmd_count = 0;
    while_cmd_idx = 0;
    if_cmd_count = 0;
    if_cmd_idx = 0;
    executing_while = 0;
    executing_if = 0;
    waiting_for_uart = 0;
    while_condition_valid = 0;
    memset(if_data, 0, sizeof(if_data));
    while_var_name[0] = '\0';
    while_expected_value[0] = '\0';
    while_body[0] = '\0';

    while (*p != '\0' && cmd_count < 30)
    {
        while (*p == ' ') p++;
        if (*p == '\0') break;

        if (strncmp(p, "repeat", 6) == 0)
        {
            int rep_count;
            p += 6;
            while (*p == ' ') p++;
            rep_count = atoi(p);
            while (*p != ' ' && *p != '\0') p++;
            while (*p == ' ') p++;

            if (*p == '[')
            {
                char *end;
                p++;
                end = strchr(p, ']');
                if (end != NULL)
                {
                    char inner_cmd_str[50];
                    int len = (int)(end - p);
                    int r;
                    if (len >= (int)sizeof(inner_cmd_str))
                        len = (int)sizeof(inner_cmd_str) - 1;
                    strncpy(inner_cmd_str, p, (size_t)len);
                    inner_cmd_str[len] = '\0';
                    for (r = 0; r < rep_count && cmd_count < 30; r++)
                    {
                        char temp_str[50];
                        strcpy(temp_str, inner_cmd_str);
                        parse_repeat_content(temp_str);
                    }
                    p = end + 1;
                }
                else
                {
                    return 0u;
                }
            }
        }
        else if (strncmp(p, "make", 4) == 0)
        {
            /* Skip the MAKE name and value.  EEPROM restores the actual
             * variable table after this structural parser finishes. */
            char dummy[20];
            p += 4;
            while (*p == ' ') p++;
            if (*p == '"') p++;
            p = read_token(p, dummy, sizeof(dummy));
            if (*p == '"') p++;
            p = read_token(p, dummy, sizeof(dummy));
        }
        else if (*p == ':')
        {
            /* LOOKUP changes no execution state; skip the variable token. */
            char dummy[20];
            p++;
            p = read_token(p, dummy, sizeof(dummy));
        }
        else if (strncmp(p, "ifelse", 6) == 0)
        {
            p += 6;
            while (*p == ' ') p++;

            if (*p == ':')
            {
                char var_name[20];
                char expected_value[20];
                int j = 0;
                p++;
                while (*p != ' ' && *p != '\0' && j < 19)
                    var_name[j++] = *p++;
                var_name[j] = '\0';

                while (*p == ' ') p++;
                if (*p == '=') p++;
                while (*p == ' ') p++;
                if (*p == '"') p++;

                j = 0;
                while (*p != ' ' && *p != '\0' && j < 19)
                    expected_value[j++] = *p++;
                expected_value[j] = '\0';
                while (*p == ' ') p++;

                if (*p == '[')
                {
                    char *true_start = p + 1;
                    char *true_end = find_matching_bracket(p);
                    if (true_end != NULL)
                    {
                        char true_cmd[30];
                        int true_len = (int)(true_end - true_start);
                        if (true_len >= (int)sizeof(true_cmd))
                            true_len = (int)sizeof(true_cmd) - 1;
                        strncpy(true_cmd, true_start, (size_t)true_len);
                        true_cmd[true_len] = '\0';
                        p = true_end + 1;
                        while (*p == ' ') p++;

                        if (*p == '[')
                        {
                            char *false_start = p + 1;
                            char *false_end = find_matching_bracket(p);
                            if (false_end != NULL)
                            {
                                char false_cmd[30];
                                int false_len = (int)(false_end - false_start);
                                if (false_len >= (int)sizeof(false_cmd))
                                    false_len = (int)sizeof(false_cmd) - 1;
                                strncpy(false_cmd, false_start, (size_t)false_len);
                                false_cmd[false_len] = '\0';
                                p = false_end + 1;

                                if (cmd_count < 30)
                                {
                                    cmd_queue[cmd_count].type = 'i';
                                    cmd_queue[cmd_count].param = 0;
                                    cmd_queue[cmd_count].repeat_cnt = 1;
                                    cmd_queue[cmd_count].turn_delta_deg = 0;
                                    strcpy(if_data[cmd_count].var, var_name);
                                    strcpy(if_data[cmd_count].expected, expected_value);
                                    strcpy(if_data[cmd_count].true_cmd, true_cmd);
                                    strcpy(if_data[cmd_count].false_cmd, false_cmd);
                                    cmd_count++;
                                }
                            }
                            else return 0u;
                        }
                    }
                    else return 0u;
                }
            }
        }
        else if (strncmp(p, "while", 5) == 0)
        {
            p += 5;
            while (*p == ' ') p++;

            if (*p == ':')
            {
                int j = 0;
                p++;
                while (*p != ' ' && *p != '\0' && j < 19)
                    while_var_name[j++] = *p++;
                while_var_name[j] = '\0';
            }

            while (*p == ' ') p++;
            if (*p == '=') p++;
            while (*p == ' ') p++;
            if (*p == '"') p++;

            {
                int j = 0;
                while (*p != ' ' && *p != '\0' && j < 19)
                    while_expected_value[j++] = *p++;
                while_expected_value[j] = '\0';
            }

            while (*p == ' ') p++;
            if (*p == '[')
            {
                char *body_start = p + 1;
                char *body_end = find_matching_bracket(p);
                if (body_end != NULL)
                {
                    int len = (int)(body_end - body_start);
                    if (len >= (int)sizeof(while_body))
                        len = (int)sizeof(while_body) - 1;
                    strncpy(while_body, body_start, (size_t)len);
                    while_body[len] = '\0';
                    while_condition_valid = 1;

                    if (cmd_count < 30)
                    {
                        cmd_queue[cmd_count].type = 'w';
                        cmd_queue[cmd_count].param = 0;
                        cmd_queue[cmd_count].repeat_cnt = 1;
                        cmd_queue[cmd_count].turn_delta_deg = 0;
                        cmd_count++;
                    }
                    parse_while_body(while_body);
                    p = body_end + 1;
                }
                else return 0u;
            }
        }
        else
        {
            char cmd_name[10];
            char value_str[20];
            int value = 1;
            p = read_token(p, cmd_name, sizeof(cmd_name));
            if (*p != '\0')
            {
                p = read_token(p, value_str, sizeof(value_str));
                value = atoi(value_str);
            }
            append_simple_command(cmd_queue, &cmd_count, 30, cmd_name, value);
        }
    }

    return 1u;
}

static uint16 ee_get_u16(const uint8 *p)
{
    return (uint16)p[0] | ((uint16)p[1] << 8);
}

static void ee_put_u16(uint8 *p, uint16 value)
{
    p[0] = (uint8)(value & 0xFFu);
    p[1] = (uint8)((value >> 8) & 0xFFu);
}

static uint16 ee_crc16(const uint8 *data, uint16 length)
{
    uint16 crc = 0xFFFFu;
    uint16 i;
    uint8 bit;
    for (i = 0u; i < length; i++)
    {
        crc ^= ((uint16)data[i] << 8);
        for (bit = 0u; bit < 8u; bit++)
            crc = (crc & 0x8000u) ? (uint16)((crc << 1) ^ 0x1021u)
                                  : (uint16)(crc << 1);
    }
    return crc;
}

static uint8 ee_bounded_strlen(const char *s, uint8 max_len)
{
    uint8 n = 0u;
    while (n < max_len && s[n] != '\0') n++;
    return n;
}

static void request_eeprom_checkpoint(void)
{
    eeprom_save_requested = 1u;
}

static void eeprom_set_program_text(const char *text)
{
    if (text == NULL) return;
    strncpy(eeprom_program_text, text, sizeof(eeprom_program_text) - 1u);
    eeprom_program_text[sizeof(eeprom_program_text) - 1u] = '\0';
}

/* Build a compact snapshot.  Variables are length-prefixed, so normal use
 * (for example only :vowel) writes far fewer rows than the worst case. */
static uint16 eeprom_build_snapshot(uint8 *data, uint16 sequence)
{
    uint16 offset = EE_HEADER_BYTES;
    uint16 crc;
    uint8 program_len;
    uint8 runtime_flags = 0u;
    uint8 active;
    uint8 i;
    int heading;

    memset(data, 0, EE_BANK_BYTES);

    if (cmd_count < 0 || cmd_count > 30 || current_cmd_idx < 0 || current_cmd_idx > 30 ||
        movement_steps_done < 0 || movement_steps_done > 65535 ||
        variable_count < 0 || variable_count > 10 ||
        if_cmd_idx < 0 || if_cmd_idx > 255 || while_cmd_idx < 0 || while_cmd_idx > 255 ||
        if_cmd_count < 0 || if_cmd_count > 255 || while_cmd_count < 0 || while_cmd_count > 255)
    {
        return 0u;
    }

    active = (cmd_count > 0 && current_cmd_idx < cmd_count) ? 1u : 0u;
    program_len = ee_bounded_strlen(eeprom_program_text, (uint8)EE_PROGRAM_MAX);

    data[EE_H_MAGIC0] = EE_MAGIC_0;
    data[EE_H_MAGIC1] = EE_MAGIC_1;
    data[EE_H_VERSION] = EE_FORMAT_VERSION;
    data[EE_H_ACTIVE] = active;
    ee_put_u16(&data[EE_H_SEQ_LO], sequence);

    /* Payload fixed fields. */
    if ((uint16)(offset + 1u + program_len + EE_RUNTIME_BYTES) > EE_BANK_BYTES) return 0u;
    data[offset++] = program_len;
    memcpy(&data[offset], eeprom_program_text, program_len);
    offset = (uint16)(offset + program_len);

    data[offset++] = (uint8)((cmd_count < 0) ? 0 : cmd_count);
    data[offset++] = (uint8)((current_cmd_idx < 0) ? 0 : current_cmd_idx);
    ee_put_u16(&data[offset], (uint16)((movement_steps_done < 0) ? 0 : movement_steps_done));
    offset += 2u;

    heading = (int)(current_heading_deg + 0.5f);
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    ee_put_u16(&data[offset], (uint16)heading);
    offset += 2u;

    if (executing_if) runtime_flags |= EE_FLAG_EXEC_IF;
    if (executing_while) runtime_flags |= EE_FLAG_EXEC_WHILE;
    if (waiting_for_uart) runtime_flags |= EE_FLAG_WAIT_WHILE;
    if (eeprom_if_branch_valid) runtime_flags |= EE_FLAG_IF_BRANCH_VALID;
    if (eeprom_if_branch_true) runtime_flags |= EE_FLAG_IF_BRANCH_TRUE;
    data[offset++] = runtime_flags;
    data[offset++] = (uint8)((if_cmd_idx < 0) ? 0 : if_cmd_idx);
    data[offset++] = (uint8)((while_cmd_idx < 0) ? 0 : while_cmd_idx);
    data[offset++] = (uint8)((if_cmd_count < 0) ? 0 : if_cmd_count);
    data[offset++] = (uint8)((while_cmd_count < 0) ? 0 : while_cmd_count);
    data[offset++] = (uint8)((variable_count < 0) ? 0 : variable_count);

    /* Persist the complete current variable table. */
    for (i = 0u; i < (uint8)variable_count && i < 10u; i++)
    {
        uint8 name_len = ee_bounded_strlen(variables[i].name, 19u);
        uint8 value_len = ee_bounded_strlen(variables[i].value, 19u);
        uint16 needed = (uint16)(2u + name_len + value_len);
        if ((uint16)(offset + needed) > EE_BANK_BYTES) return 0u;
        data[offset++] = name_len;
        data[offset++] = value_len;
        memcpy(&data[offset], variables[i].name, name_len);
        offset = (uint16)(offset + name_len);
        memcpy(&data[offset], variables[i].value, value_len);
        offset = (uint16)(offset + value_len);
    }

    ee_put_u16(&data[EE_H_LEN_LO], offset);
    data[EE_H_CRC_LO] = 0u;
    data[EE_H_CRC_HI] = 0u;
    crc = ee_crc16(data, offset);
    ee_put_u16(&data[EE_H_CRC_LO], crc);
    return offset;
}

static uint8 eeprom_load_bank(uint8 first_row, uint8 *dest, uint16 *length_out, uint16 *seq_out)
{
    uint16 base = (uint16)first_row * EE_ROW_SIZE;
    uint16 total_len;
    uint16 stored_crc;
    uint16 calc_crc;
    uint16 i;

    memset(dest, 0, EE_BANK_BYTES);
    for (i = 0u; i < EE_HEADER_BYTES; i++)
        dest[i] = EEPROM_ReadByte((uint16)(base + i));

    if (dest[EE_H_MAGIC0] != EE_MAGIC_0 || dest[EE_H_MAGIC1] != EE_MAGIC_1 ||
        dest[EE_H_VERSION] != EE_FORMAT_VERSION || dest[EE_H_ACTIVE] > 1u)
        return 0u;

    total_len = ee_get_u16(&dest[EE_H_LEN_LO]);
    if (total_len < EE_HEADER_BYTES || total_len > EE_BANK_BYTES)
        return 0u;

    for (i = EE_HEADER_BYTES; i < total_len; i++)
        dest[i] = EEPROM_ReadByte((uint16)(base + i));

    stored_crc = ee_get_u16(&dest[EE_H_CRC_LO]);
    dest[EE_H_CRC_LO] = 0u;
    dest[EE_H_CRC_HI] = 0u;
    calc_crc = ee_crc16(dest, total_len);
    ee_put_u16(&dest[EE_H_CRC_LO], stored_crc);

    if (stored_crc != calc_crc) return 0u;
    *length_out = total_len;
    *seq_out = ee_get_u16(&dest[EE_H_SEQ_LO]);
    return 1u;
}

static uint8 eeprom_sequence_is_newer(uint16 a, uint16 b)
{
    return ((int16)(a - b) > 0) ? 1u : 0u;
}

static cystatus eeprom_write_bank(uint8 first_row, const uint8 *data, uint16 total_len)
{
    uint8 invalid_header[EE_ROW_SIZE] = {0u};
    uint8 rows_to_write = (uint8)((total_len + EE_ROW_SIZE - 1u) / EE_ROW_SIZE);
    uint8 row;
    cystatus status;

    status = EEPROM_Write(invalid_header, first_row);
    if (status != CYRET_SUCCESS) return status;

    for (row = 1u; row < rows_to_write; row++)
    {
        status = EEPROM_Write(&data[(uint16)row * EE_ROW_SIZE], (uint8)(first_row + row));
        if (status != CYRET_SUCCESS) return status;
    }

    /* Header/commit marker is written last. */
    return EEPROM_Write(data, first_row);
}

static void service_eeprom_checkpoint(void)
{
    uint8 interrupt_state;
    uint16 total_len;
    uint16 new_sequence;
    uint8 target_bank;
    uint8 first_row;
    uint8 temp_status;
    cystatus status;

    if (eeprom_save_requested == 0u) return;

    interrupt_state = CyEnterCriticalSection();
    eeprom_save_requested = 0u;
    new_sequence = (uint16)(eeprom_sequence + 1u);
    total_len = eeprom_build_snapshot(eeprom_buffer, new_sequence);
    CyExitCriticalSection(interrupt_state);

    if (total_len == 0u)
    {
        UART_1_PutString("EEPROM SAVE ERROR: snapshot too large\r\n");
        return;
    }

    temp_status = EEPROM_UpdateTemperature();
    if (temp_status != 0u)
    {
        uart_printf("EEPROM ERROR: temperature update status=%u\r\n", (unsigned int)temp_status);
        return;
    }

    target_bank = (eeprom_active_bank == 0u) ? 1u : 0u;
    first_row = (target_bank == 0u) ? EE_BANK_A_FIRST_ROW : EE_BANK_B_FIRST_ROW;
    status = eeprom_write_bank(first_row, eeprom_buffer, total_len);

    if (status == CYRET_SUCCESS)
    {
        eeprom_active_bank = target_bank;
        eeprom_sequence = new_sequence;
        uart_printf("EEPROM SAVE OK: seq=%u bank=%c active=%u idx=%d/%d step=%d bytes=%u\r\n",
                    (unsigned int)eeprom_sequence,
                    target_bank == 0u ? 'A' : 'B',
                    (unsigned int)eeprom_buffer[EE_H_ACTIVE],
                    current_cmd_idx, cmd_count, movement_steps_done,
                    (unsigned int)total_len);
    }
    else
    {
        uart_printf("EEPROM WRITE FAILED: status=%u; previous bank kept\r\n", (unsigned int)status);
    }
}

static uint8 eeprom_restore_payload(const uint8 *data, uint16 total_len)
{
    uint16 offset = EE_HEADER_BYTES;
    uint8 program_len;
    uint8 saved_cmd_count;
    uint8 saved_cmd_idx;
    uint16 saved_steps;
    uint16 saved_heading;
    uint8 flags;
    uint8 saved_if_idx;
    uint8 saved_while_idx;
    uint8 saved_if_count;
    uint8 saved_while_count;
    uint8 saved_var_count;
    uint8 i;
    uint8 active = data[EE_H_ACTIVE];

    if (offset >= total_len) return 0u;
    program_len = data[offset++];
    if (program_len > EE_PROGRAM_MAX || (uint16)(offset + program_len + EE_RUNTIME_BYTES) > total_len) return 0u;
    memcpy(eeprom_program_text, &data[offset], program_len);
    eeprom_program_text[program_len] = '\0';
    offset = (uint16)(offset + program_len);

    saved_cmd_count = data[offset++];
    saved_cmd_idx = data[offset++];
    saved_steps = ee_get_u16(&data[offset]); offset += 2u;
    saved_heading = ee_get_u16(&data[offset]); offset += 2u;
    flags = data[offset++];
    saved_if_idx = data[offset++];
    saved_while_idx = data[offset++];
    saved_if_count = data[offset++];
    saved_while_count = data[offset++];
    saved_var_count = data[offset++];

    if (saved_var_count > 10u) return 0u;

    /* Reconstruct parser-owned queues/data from the exact original UART line. */
    if (active != 0u)
    {
        if (program_len == 0u || rebuild_program_from_text(eeprom_program_text) == 0u)
            return 0u;
        if (cmd_count != (int)saved_cmd_count || saved_cmd_idx >= saved_cmd_count)
            return 0u;
    }
    else
    {
        cmd_count = 0;
        current_cmd_idx = 0;
        movement_steps_done = 0;
        executing_if = 0;
        executing_while = 0;
        waiting_for_uart = 0;
    }

    /* Restore variables exactly as they were at the checkpoint. */
    variable_count = 0;
    for (i = 0u; i < saved_var_count; i++)
    {
        uint8 name_len;
        uint8 value_len;
        if ((uint16)(offset + 2u) > total_len) return 0u;
        name_len = data[offset++];
        value_len = data[offset++];
        if (name_len > 19u || value_len > 19u ||
            (uint16)(offset + name_len + value_len) > total_len) return 0u;
        memcpy(variables[i].name, &data[offset], name_len);
        variables[i].name[name_len] = '\0';
        offset = (uint16)(offset + name_len);
        memcpy(variables[i].value, &data[offset], value_len);
        variables[i].value[value_len] = '\0';
        offset = (uint16)(offset + value_len);
        variable_count++;
    }

    current_heading_deg = (float)(saved_heading % 360u);

    if (active == 0u)
    {
        eeprom_if_branch_valid = 0u;
        eeprom_if_branch_true = 0u;
        uart_printf("EEPROM RESTORE: IDLE heading=%u vars=%u\r\n",
                    (unsigned int)saved_heading, (unsigned int)saved_var_count);
        return 1u;
    }

    current_cmd_idx = saved_cmd_idx;
    movement_steps_done = (int)saved_steps;
    executing_if = 0;
    executing_while = 0;
    waiting_for_uart = 0;
    uart_msg_received = 0;
    eeprom_if_branch_valid = (flags & EE_FLAG_IF_BRANCH_VALID) ? 1u : 0u;
    eeprom_if_branch_true = (flags & EE_FLAG_IF_BRANCH_TRUE) ? 1u : 0u;

    if ((flags & EE_FLAG_EXEC_IF) != 0u)
    {
        IfelseData *d;
        char chosen_copy[30];
        if ((flags & (EE_FLAG_EXEC_WHILE | EE_FLAG_WAIT_WHILE)) != 0u ||
            eeprom_if_branch_valid == 0u || cmd_queue[current_cmd_idx].type != 'i') return 0u;
        d = &if_data[current_cmd_idx];
        strncpy(chosen_copy, eeprom_if_branch_true ? d->true_cmd : d->false_cmd,
                sizeof(chosen_copy) - 1u);
        chosen_copy[sizeof(chosen_copy) - 1u] = '\0';
        if_cmd_count = 0;
        parse_into(chosen_copy, if_cmd_queue, &if_cmd_count, IF_QUEUE_SIZE);
        if (if_cmd_count != (int)saved_if_count || saved_if_idx >= saved_if_count) return 0u;
        if_cmd_idx = saved_if_idx;
        executing_if = 1;
    }
    else if ((flags & EE_FLAG_EXEC_WHILE) != 0u)
    {
        if ((flags & EE_FLAG_WAIT_WHILE) != 0u || cmd_queue[current_cmd_idx].type != 'w') return 0u;
        if (while_cmd_count != (int)saved_while_count || saved_while_idx > saved_while_count) return 0u;
        while_cmd_idx = saved_while_idx;
        executing_while = 1;
    }
    else if ((flags & EE_FLAG_WAIT_WHILE) != 0u)
    {
        if (cmd_queue[current_cmd_idx].type != 'w') return 0u;
        waiting_for_uart = 1;
        executing_while = 0;
        while_cmd_idx = saved_while_idx;
    }
    else
    {
        char t = cmd_queue[current_cmd_idx].type;
        if ((t == 'f' || t == 'b') && movement_steps_done > abs(cmd_queue[current_cmd_idx].param))
            return 0u;
        if (t != 'f' && t != 'b') movement_steps_done = 0;
    }

    request_reset_start = 1;
    uart_printf("EEPROM RESUME: idx=%d/%d type=%c step=%d heading=%u IF=%d WHILE=%d WAIT=%d\r\n",
                current_cmd_idx, cmd_count, cmd_queue[current_cmd_idx].type,
                movement_steps_done, (unsigned int)saved_heading,
                executing_if, executing_while, waiting_for_uart);
    return 1u;
}

static uint8 restore_eeprom_checkpoint(void)
{
    uint8 valid_a, valid_b;
    uint16 len_a = 0u, len_b = 0u;
    uint16 seq_a = 0u, seq_b = 0u;
    uint8 chosen_bank;
    uint8 first_row;
    uint16 chosen_len, chosen_seq;

    valid_a = eeprom_load_bank(EE_BANK_A_FIRST_ROW, eeprom_buffer, &len_a, &seq_a);
    valid_b = eeprom_load_bank(EE_BANK_B_FIRST_ROW, eeprom_buffer, &len_b, &seq_b);

    if (valid_a == 0u && valid_b == 0u)
    {
        eeprom_active_bank = 0xFFu;
        eeprom_sequence = 0u;
        UART_1_PutString("EEPROM: no valid checkpoint (fresh start)\r\n");
        return 0u;
    }

    if (valid_a != 0u && valid_b == 0u) chosen_bank = 0u;
    else if (valid_a == 0u && valid_b != 0u) chosen_bank = 1u;
    else chosen_bank = eeprom_sequence_is_newer(seq_b, seq_a) ? 1u : 0u;

    first_row = (chosen_bank == 0u) ? EE_BANK_A_FIRST_ROW : EE_BANK_B_FIRST_ROW;
    if (eeprom_load_bank(first_row, eeprom_buffer, &chosen_len, &chosen_seq) == 0u)
        return 0u;

    eeprom_active_bank = chosen_bank;
    eeprom_sequence = chosen_seq;

    if (eeprom_restore_payload(eeprom_buffer, chosen_len) == 0u)
    {
        cmd_count = 0;
        current_cmd_idx = 0;
        movement_steps_done = 0;
        executing_if = 0;
        executing_while = 0;
        waiting_for_uart = 0;
        request_reset_start = 0;
        eeprom_if_branch_valid = 0u;
        eeprom_if_branch_true = 0u;
        UART_1_PutString("EEPROM RESTORE REJECTED: corrupt/inconsistent state\r\n");
        return 0u;
    }

    uart_printf("EEPROM BANK %c seq=%u restored\r\n",
                chosen_bank == 0u ? 'A' : 'B', (unsigned int)eeprom_sequence);
    return eeprom_buffer[EE_H_ACTIVE] ? 1u : 0u;
}

CY_ISR(ISR_Handler_1)
{
    Rx = UART_1_GetChar();
    /* Discard startup input while EEPROM owns the execution state. */
    if (!eeprom_boot_ready) return;
    
    if (Rx != '\0')
    { 
        UART_1_PutChar(Rx);
        if (Rx == '\r' || Rx == '\n')
        {
            if (rx_index == 0)
            {
                return;
            }
            UART_1_PutString("\n");
            rx_buffer[rx_index] = '\0';

            char *p = rx_buffer;
            int is_lookup_command = (p[0] == ':');
            
            /* =========================
               EMERGENCY STOP
               (bypasses the BUSY gate on purpose - must work even
               while moving/scanning/executing while or ifelse)
               ========================= */
            if (strncmp(p, "stop", 4) == 0)
            {
                stop_motors();
                df_send_command(0x16, 0x00, 0x00);   /* DFPlayer: stop playback */

                cmd_count = 0;
                current_cmd_idx = 0;

                /* Emergency stop is intentionally independent of EEPROM.
                 * Cancel a not-yet-written checkpoint request so this STOP
                 * command does not modify the last committed power-loss save. */
                eeprom_save_requested = 0u;

                while_cmd_count = 0;
                while_cmd_idx = 0;
                executing_while = 0;

                if_cmd_count = 0;
                if_cmd_idx = 0;
                executing_if = 0;

                waiting_for_uart = 0;
                uart_msg_received = 0;
                movement_steps_done = 0;

                request_reset_start = 0;   /* not moving, nothing to reset a baseline for */
                camera_state = CAMERA_READY;   /* ready to accept the next command immediately */

                UART_1_PutString("EMERGENCY STOP\r\n");

                rx_index = 0;
                return;
            }

            /* Check whether this line is MAKE */
            int is_make_command = (strncmp(p, "make", 4) == 0);

            /* The parser chooses IFELSE branches here, in the UART1 ISR.
             * Only accept a new command at a confirmed, stationary position.
             * Do not let a command overwrite an active multi-grid sequence.
             */
            if (camera_state != CAMERA_READY || waiting_for_uart ||
                executing_while || current_cmd_idx < cmd_count)
            {
                uart_printf("BUSY: cam=%d(%s) wait=%d while=%d idx=%d cnt=%d ms=%lu\r\n",
                camera_state,
                camera_state == CAMERA_SETTLING    ? "SETTLING"  :
                camera_state == CAMERA_WAIT_RESULT ? "WAIT_RESULT" :
                camera_state == CAMERA_READY       ? "READY"     : "MOVING",
                waiting_for_uart, executing_while,
                current_cmd_idx, cmd_count,
                (unsigned long)camera_ms);

                //UART_1_PutString("BUSY: moving/scanning; retry command when stopped\r\n");
                rx_index = 0;
                return;
            }

            /* New movement/program command starts a new queue */
            if (!is_make_command)
            {
                cmd_count = 0;
                current_cmd_idx = 0;
            }

            while (*p != '\0' && cmd_count < 30)
            {
                /* Skip spaces */
                while (*p == ' ')
                {
                    p++;
                }

                if (*p == '\0')
                {
                    break;
                }

                /* =========================
                   REPEAT
                   ========================= */
                if (strncmp(p, "repeat", 6) == 0)
                {
                    p += 6;

                    /* Skip spaces */
                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* Get repeat number */
                    int rep_count = atoi(p);

                    while (*p != ' ' && *p != '\0')
                    {
                        p++;
                    }

                    /* Skip spaces */
                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* Expect '[' */
                    if (*p == '[')
                    {
                        p++;

                        /* Find end of repeat block */
                        char *end = strchr(p, ']');

                        if (end != NULL)
                        {
                            /* Copy contents inside [ ] */
                            char inner_cmd_str[50];

                            int len = end - p;

                            if (len >= sizeof(inner_cmd_str))
                            {
                                len = sizeof(inner_cmd_str) - 1;
                            }

                            strncpy(inner_cmd_str, p, len);
                            inner_cmd_str[len] = '\0';

                            /* =========================
                               Parse commands inside [ ]
                               ========================= */

                            for (int r = 0; r < rep_count; r++)
                            {
                                char temp_str[50];
                                strcpy(temp_str, inner_cmd_str);
                                parse_repeat_content(temp_str);
                                
                            }

                            /* Jump to character after ']' */
                            p = end + 1;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                
                /* =========================
                   MAKE COMMAND
                   ========================= */
                else if (strncmp(p, "make", 4) == 0)
                {
                    p += 4;

                    /* Skip spaces */
                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* Skip " before variable name */
                    if (*p == '"')
                    {
                        p++;
                    }

                    /* Read variable name */
                    char var_name[20];
                    int j = 0;

                    while (*p != ' ' && *p != '\0' && j < 19)
                    {
                        var_name[j++] = *p;
                        p++;
                    }

                    var_name[j] = '\0';

                    /* Skip spaces */
                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* Skip " before value */
                    if (*p == '"')
                    {
                        p++;
                    }

                    /* Read value */
                    char value[20];
                    j = 0;

                    while (*p != ' ' && *p != '\0' && j < 19)
                    {
                        value[j++] = *p;
                        p++;
                    }

                    value[j] = '\0';

                    /* Store variable */
                    int found = 0;

                    /* Check whether variable already exists */
                    for (int i = 0; i < variable_count; i++)
                    {
                        if (strcmp(variables[i].name, var_name) == 0)
                        {
                            /* Variable exists -> update value */
                            strcpy(variables[i].value, value);
                            found = 1;

                            uart_printf("MAKE UPDATE: %s = %s\r\n", var_name, value);
                            uart_msg_received = 1;

                            break;
                        }
                    }

                    /* Variable does not exist -> create new variable */
                    if (!found && variable_count < 10)
                    {
                        strcpy(variables[variable_count].name, var_name);
                        strcpy(variables[variable_count].value, value);

                        variable_count++;

                        uart_printf("MAKE NEW: %s = %s\r\n", var_name, value);
                        uart_msg_received = 1;
                    }
                }
                
                /* =========================
                   LOOKUP COMMAND
                   ========================= */
                else if (*p == ':')
                {
                    p++;  

                    char var_name[20];
                    int j = 0;

                    while (*p != ' ' && *p != '\0' && j < 19)
                    {
                        var_name[j++] = *p;
                        p++;
                    }

                    var_name[j] = '\0';

                    char *value = get_variable(var_name);

                    if (value != NULL)
                    {
                        uart_printf("VARIABLE: %s = %s\r\n", var_name, value);
                    }
                    else
                    {
                        uart_printf("VARIABLE NOT FOUND: %s\r\n", var_name);
                    }
                }
                
                /* =========================
                   IFELSE COMMAND
                   ========================= */
                                else if (strncmp(p, "ifelse", 6) == 0)
                {
                    p += 6;

                    while (*p == ' ')
                    {
                        p++;
                    }

                    if (*p == ':')
                    {
                        p++;

                        char var_name[20];
                        int j = 0;

                        while (*p != ' ' && *p != '\0' && j < 19)
                        {
                            var_name[j++] = *p;
                            p++;
                        }

                        var_name[j] = '\0';

                        while (*p == ' ')
                        {
                            p++;
                        }

                        if (*p == '=')
                        {
                            p++;
                        }

                        while (*p == ' ')
                        {
                            p++;
                        }

                        if (*p == '"')
                        {
                            p++;
                        }

                        char expected_value[20];
                        j = 0;

                        while (*p != ' ' && *p != '\0' && j < 19)
                        {
                            expected_value[j++] = *p;
                            p++;
                        }

                        expected_value[j] = '\0';

                        /* Skip spaces */
                        while (*p == ' ')
                        {
                            p++;
                        }

                        /* First [ ... ] */
                        if (*p == '[')
                        {
                            char *true_start = p + 1;
                            char *true_end = find_matching_bracket(p);

                            if (true_end != NULL)
                            {
                                int true_len = true_end - true_start;

                                char true_cmd[30];

                                strncpy(true_cmd, true_start, true_len);
                                true_cmd[true_len] = '\0';

                                p = true_end + 1;

                                /* Skip spaces */
                                while (*p == ' ')
                                {
                                    p++;
                                }

                                /* Second [ ... ] */
                                if (*p == '[')
                                {
                                    char *false_start = p + 1;
                                    char *false_end = find_matching_bracket(p);

                                    if (false_end != NULL)
                                    {
                                        int false_len = false_end - false_start;

                                        char false_cmd[30];

                                        strncpy(false_cmd,
                                                false_start,
                                                false_len);

                                        false_cmd[false_len] = '\0';

                                        p = false_end + 1;

                                        if (cmd_count < 30)
                                        {
                                            cmd_queue[cmd_count].type = 'i';
                                            cmd_queue[cmd_count].param = 0;
                                            cmd_queue[cmd_count].repeat_cnt = 1;
                                            cmd_queue[cmd_count].turn_delta_deg = 0;

                                            strcpy(if_data[cmd_count].var, var_name);
                                            strcpy(if_data[cmd_count].expected, expected_value);
                                            strcpy(if_data[cmd_count].true_cmd, true_cmd);
                                            strcpy(if_data[cmd_count].false_cmd, false_cmd);

                                            uart_printf("IFELSE QUEUED: %s = %s [%s] [%s]\r\n",
                                                        var_name, expected_value, true_cmd, false_cmd);

                                            cmd_count++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                /* =========================
                   WHILE COMMAND
                   ========================= */
                else if (strncmp(p, "while", 5) == 0)
                {
                    p += 5;

                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* =========================
                       Get variable name
                       ========================= */

                    if (*p == ':')
                    {
                        p++;

                        int j = 0;

                        while (*p != ' ' &&
                               *p != '\0' &&
                               j < 19)
                        {
                            while_var_name[j++] = *p;
                            p++;
                        }

                        while_var_name[j] = '\0';
                    }

                    /* =========================
                       Get =
                       ========================= */

                    while (*p == ' ')
                    {
                        p++;
                    }

                    if (*p == '=')
                    {
                        p++;
                    }

                    while (*p == ' ')
                    {
                        p++;
                    }

                    /* =========================
                       Get expected value
                       ========================= */

                    if (*p == '"')
                    {
                        p++;
                    }

                    int j = 0;

                    while (*p != ' ' &&
                           *p != '\0' &&
                           j < 19)
                    {
                        while_expected_value[j++] = *p;
                        p++;
                    }

                    while_expected_value[j] = '\0';

                    /* =========================
                       Get [ ... ] body
                       ========================= */

                    while (*p == ' ')
                    {
                        p++;
                    }

                    if (*p == '[')
                    {
                        char *body_start = p + 1;

                        char *body_end = find_matching_bracket(p);

                        if (body_end != NULL)
                        {
                            int len = body_end - body_start;

                            if (len >= sizeof(while_body))
                            {
                                len = sizeof(while_body) - 1;
                            }

                            strncpy(while_body,
                                    body_start,
                                    len);

                            while_body[len] = '\0';

                            while_condition_valid = 1;
                            cmd_queue[cmd_count].type = 'w';
                            cmd_queue[cmd_count].param = 0;
                            cmd_queue[cmd_count].repeat_cnt = 1;
                            cmd_queue[cmd_count].turn_delta_deg = 0;

                            cmd_count++;
                            parse_while_body(while_body);

                            uart_printf("WHILE CMD COUNT=%d\r\n", while_cmd_count);
                            uart_printf("WHILE: %s == %s\r\n",
                                        while_var_name, while_expected_value);
                            uart_printf("WHILE BODY=[%s]\r\n", while_body);

                            /*
                             * Move p to the command AFTER
                             * the while body.
                             */
                            p = body_end + 1;
                        }
                    }
                }

                /* =========================
                   NORMAL COMMAND
                   ========================= */
                else
                {
                    char cmd_name[10];
                    p = read_token(p, cmd_name, sizeof(cmd_name));

                    int target_val = 1;

                    if (*p != '\0')
                    {
                        char value_str[20];
                        p = read_token(p, value_str, sizeof(value_str));
                        target_val = atoi(value_str);
                    }

                    append_simple_command(cmd_queue, &cmd_count, 30, cmd_name, target_val);
                }
            }
            if (cmd_count > 0)
            {
                
                uart_printf("ACCEPTED: cam=%d ms=%lu\r\n", camera_state, (unsigned long)camera_ms);
                uart_printf("DEBUG: count=%d\r\n", cmd_count);

                for (int j = 0; j < cmd_count; j++)
                {
                    uart_printf("QUEUE[%d]: type=%c param=%d repeat=%d\r\n",
                                j,
                                cmd_queue[j].type,
                                cmd_queue[j].param,
                                cmd_queue[j].repeat_cnt);
                }

                /* EEPROM: preserve the exact UART1/Bluetooth program text.
                 * The parser structures can therefore be reconstructed after
                 * a complete power loss, including REPEAT/IFELSE/WHILE syntax. */
                eeprom_set_program_text(rx_buffer);
                eeprom_if_branch_valid = 0u;
                eeprom_if_branch_true = 0u;
                request_eeprom_checkpoint();

                request_reset_start = 1;
            }
            else if (is_make_command || is_lookup_command)
            {
                /* MAKE and LOOKUP have no motor queue, but the exact UART line
                 * can still be retained. MAKE also persists the variable table;
                 * LOOKUP has no execution state to resume after reboot. */
                eeprom_set_program_text(rx_buffer);
                request_eeprom_checkpoint();
            }
            rx_index = 0;
        }
        else
        {
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = Rx;
            }
        }
    
    }
        
} 

/* Start the stopped-only settling interval and discard in-flight readings. */
static void begin_camera_scan(void)
{
    camera_state = CAMERA_SETTLING;
    stop_motors();
    camera_stop_ms = camera_ms;
    uart_msg_received = 0;
    UART_2_ClearRxBuffer();
}

/* Publish atomically: the UART1 command ISR also reads this variable table. */
static void update_camera_vowel(uint8 camera_rx)
{
    uint8 interrupt_state = CyEnterCriticalSection();

    char *value = get_variable("vowel");

    if (value != NULL)
    {
        if (camera_rx == 'N')
        {
            strcpy(value, "NONE");
        }
        else
        {
            value[0] = (char)(camera_rx - 'A' + 'a');
            value[1] = '\0';
        }
    }
    

    uart_msg_received = 1;
    camera_state = CAMERA_READY;

    CyExitCriticalSection(interrupt_state);
}

/* Print every raw byte waiting in UART_2's buffer, then discard it.
 * Used anywhere we'd otherwise call UART_2_ClearRxBuffer() silently. */
static void drain_and_print_camera_bytes(void)
{
    while (UART_2_GetRxBufferSize() > 0u)
    {
        uint8 b = UART_2_GetChar();

        if (b >= 0x20 && b <= 0x7E)
        {
            uart_printf("CAM RAW: 0x%02X '%c'\r\n", b, b);
        }
        else
        {
            uart_printf("CAM RAW: 0x%02X\r\n", b);
        }
    }
}

/* Camera protocol: one A/E/I/O/U or N byte per detection (either case).
 * Camera TX -> UART_2 RX (P15[5]); UART_2 TX (P12[7]) -> USB-UART RX.
 * Updates :vowel for existing conditions and reports detections to Termite.
 * Drain/discard during motion and settling. After the settling interval,
 * flush once more and require a newly received valid byte before proceeding.
 */
static void check_camera_uart(void)
{
    uint8 camera_rx;
    static uint8 last_reported = 0u;
    uint8 report_result;
    char camera_message[] = "CAMERA DETECTED: A\r\n";
    if (camera_state == CAMERA_READY)
    {
        UART_2_ClearRxBuffer();
        return;
    }

    if (camera_state == CAMERA_MOVING || camera_state == CAMERA_SETTLING)
    {
        UART_2_ClearRxBuffer();
        if (camera_state == CAMERA_SETTLING &&
            (uint32)(camera_ms - camera_stop_ms) >= CAMERA_SETTLE_MS)
        {
            camera_state = CAMERA_WAIT_RESULT;
        }
        return;
    }
    
    if (camera_state == CAMERA_WAIT_RESULT &&
    (uint32)(camera_ms - camera_stop_ms) >= CAMERA_SETTLE_MS + 2000u)
    {
        uint8 s = CyEnterCriticalSection();
        char *v = get_variable("vowel");
        if (v != NULL) strcpy(v, "NONE");
        uart_msg_received = 1;
        camera_state = CAMERA_READY;
        camera_stop_ms = camera_ms;
        CyExitCriticalSection(s);
        UART_1_PutString("CAMERA TIMEOUT -> vowel=NONE\r\n");
    }

    /* Bound service time even if the camera transmits continuously. */
    if (UART_2_GetRxBufferSize() > 0u)
    {
        camera_rx = UART_2_GetChar();
        //uart_printf("CAM RX: 0x%02X state=%d\r\n", camera_rx, camera_state);
        
        if (camera_rx >= 'a' && camera_rx <= 'z')
        {
            camera_rx = (uint8)(camera_rx - 'a' + 'A');
        }
        report_result = (camera_state == CAMERA_WAIT_RESULT || camera_rx != last_reported);
        switch (camera_rx)
        {
            case 'A':
            case 'E':
            case 'I':
            case 'O':
            case 'U':
                update_camera_vowel(camera_rx);
                camera_message[sizeof("CAMERA DETECTED: ") - 1u] = (char)camera_rx;
                if (report_result) 
                {
                    UART_2_PutString(camera_message);
                    //play_vowel_sound(camera_rx);
                }
                last_reported = camera_rx;
                char *v_now = get_variable("vowel");
                uart_printf("VOWEL NOW: %s\r\n", v_now != NULL ? v_now : "(not made yet)");
                if (v_now != NULL && report_result)  
                {
                    play_vowel_sound(camera_rx);
                }
                break;

            case 'N':
                update_camera_vowel(camera_rx);
                if (report_result) UART_2_PutString("NO VOWELS DETECTED\r\n");
                last_reported = camera_rx;
                char *none_now = get_variable("vowel");
                uart_printf("VOWEL NOW: %s\r\n", none_now != NULL ? none_now : "(not made yet)");
                break;

            default:
                /* Ignore separators and unsupported bytes. */
                break;
        }
    }
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_1_Start();
    UART_2_Start();
    UART_3_Start();

    /* Internal EEPROM initialization; original motor/camera/audio settings are retained. */
    EEPROM_Start();
    CyDelayUs(10u);
    if (EEPROM_UpdateTemperature() != 0u)
    {
        UART_1_PutString("EEPROM WARNING: temperature update failed\r\n");
    }

    df_set_volume(30);
    UART_2_PutString("UART2 READY\r\n");
    isr_1_StartEx(ISR_Handler_1);
    UART_1_PutString("START...........");
    UART_1_PutString("\n");
    int counter1 = 0;
    int counter2 = 0;
    //df_play_track(DF_TRACK_A);

    // initial setup
    PWM_1_Start();
    QuadDec_1_Start();
    PWM_2_Start();
    QuadDec_2_Start();
    
    PWM_1_WriteCompare(230);
    PWM_2_WriteCompare(230);
    
    int start_c1 = QuadDec_1_GetCounter();
    int start_c2 = QuadDec_2_GetCounter();
    int relative_c1 = 0;
    int relative_c2 = 0;

    /* CySysTick uses a 1 ms period at the configured CPU clock. */
    CySysTickStart();
    CySysTickSetCallback(0u, camera_tick);
    begin_camera_scan();

    /* EEPROM restore completes before command processing is enabled. */
    (void)restore_eeprom_checkpoint();
    eeprom_boot_ready = 1u;

    uint32 last_motor_update_ms = camera_ms;
    
    for(;;)
    {  
        /* Perform deferred EEPROM writes outside UART ISR / motor code. */
        service_eeprom_checkpoint();

        /* Place your application code here. */
        check_forward_ir();
        check_backward_ir();
        check_camera_uart();
        if (camera_state == CAMERA_SETTLING || camera_state == CAMERA_WAIT_RESULT)
        {
            /* Motors are already stopped; keep servicing UART while waiting.
             * No valid camera result means remain stopped, never drive blind.
             */
            continue;
        }
        if (waiting_for_uart)
        {
            if (!uart_msg_received)
            {
                continue;
            }

            waiting_for_uart = 0;
            uart_msg_received = 0;

            if (while_condition_true())
            {
                UART_1_PutString("WHILE LOOP AGAIN\r\n");

                while_cmd_idx = 0;
                executing_while = 1;
                request_reset_start = 1;
                request_eeprom_checkpoint();
            }
            else
            {
                UART_1_PutString("WHILE END\r\n");

                current_cmd_idx++;
                if (current_cmd_idx >= cmd_count)
                {
                    current_cmd_idx = 0;
                    cmd_count = 0;
                }
                request_reset_start = 1;

                uart_printf("AFTER WHILE: idx=%d count=%d\r\n",
                            current_cmd_idx, cmd_count);
                request_eeprom_checkpoint();
            }

            continue;
        }

        /* Preserve the nominal 100 ms motor update period without blocking
         * UART servicing. Sample encoders when the update is actually due.
         */
        if ((uint32)(camera_ms - last_motor_update_ms) < MOTOR_CONTROL_PERIOD_MS)
        {
            continue;
        }
        last_motor_update_ms = camera_ms;

        if (request_reset_start)
        {
            request_reset_start = 0;
            start_c1 = QuadDec_1_GetCounter();
            start_c2 = QuadDec_2_GetCounter();
        }
        counter1 = QuadDec_1_GetCounter(); 
        counter2 = QuadDec_2_GetCounter();
        
        relative_c1 = counter1 - start_c1;
        relative_c2 = counter2 - start_c2;

        /* =========================
           WHILE BODY FINISHED
           ========================= */

        if (executing_while && while_cmd_idx >= while_cmd_count)
        {
            stop_motors();

            executing_while = 0;
            waiting_for_uart = 1;

            /* Keep the result from the final grid's scan for the next check.
             * An empty body has no final movement scan, so request one here.
             */
            if (!uart_msg_received) begin_camera_scan();

            UART_1_PutString("WAITING FOR UART...\r\n");
            request_eeprom_checkpoint();

            continue;
        }


        /* =========================
           NO MORE NORMAL COMMAND
           ========================= */

        if (current_cmd_idx >= cmd_count)
        {
            stop_motors();
            continue;
        }


        /* =========================
           GET CURRENT COMMAND
           ========================= */

        TurtleCommand *current_cmd;

        if (executing_if)
        {
            current_cmd = &if_cmd_queue[if_cmd_idx];
        }
        else if (executing_while)
        {
            current_cmd = &while_cmd_queue[while_cmd_idx];
        }
        else
        {
            current_cmd = &cmd_queue[current_cmd_idx];
        }

        if (current_cmd->type == 'f' || current_cmd->type == 'b' ||
            current_cmd->type == 'l' || current_cmd->type == 'r')
        {
            if (current_cmd->param == 0)
            {
                advance_command_index(0);
                continue;
            }
            camera_state = CAMERA_MOVING;
        }

        switch (current_cmd->type)
        {
            case 'i': //ifelse
            {
                static int if_scanned = 0;

                if (!if_scanned)
                {
                    if_scanned = 1;
                    begin_camera_scan();
                    break;
                }
                if_scanned = 0;

                IfelseData *d = &if_data[current_cmd_idx];

                int condition = compare_variable(d->var, d->expected);

                uart_printf("IFELSE: %s == %s ? %d\r\n", d->var, d->expected, condition);
                UART_1_PutString(condition ? "IF TRUE\r\n" : "IF FALSE\r\n");

                char *chosen = condition ? d->true_cmd : d->false_cmd;

                if_cmd_count = 0;
                parse_into(chosen, if_cmd_queue, &if_cmd_count, IF_QUEUE_SIZE);

                if (if_cmd_count > 0)
                {
                    executing_if = 1;
                    if_cmd_idx = 0;
                    eeprom_if_branch_valid = 1u;
                    eeprom_if_branch_true = condition ? 1u : 0u;
                    request_reset_start = 1;
                    request_eeprom_checkpoint();
                }
                else
                {
                    current_cmd_idx++;
                    if (current_cmd_idx >= cmd_count) 
                    {
                        current_cmd_idx = 0;
                        cmd_count = 0;
                    }
                    request_reset_start = 1;
                    eeprom_if_branch_valid = 0u;
                    request_eeprom_checkpoint();
                }
            }
            break;
            case 'w': //while
            {
                static int w_scanned = 0;

                if (!w_scanned)
                {
                    w_scanned = 1;
                    begin_camera_scan();
                    break;
                }
                w_scanned = 0;

                if (while_condition_true())
                {
                    UART_1_PutString("WHILE TRUE\r\n");

                    while_cmd_idx = 0;
                    executing_while = 1;
                    request_reset_start = 1;
                    request_eeprom_checkpoint();
                }
                else
                {
                    UART_1_PutString("WHILE FALSE\r\n");

                    executing_while = 0;

                    current_cmd_idx++;
                    if (current_cmd_idx >= cmd_count)
                    {
                        current_cmd_idx = 0;
                        cmd_count = 0;
                    }
                    request_eeprom_checkpoint();
                }
            }
            break;
            break;
            case 'f'://forward
            {
                /* Avoid blocking debug prints while polling for each arrival. */
                drive_forward(relative_c1, relative_c2);
            }
            break;
            case 'l'://turn left
            {
                int target = current_cmd->param; 

                /*
                 * Left turn:
                 * Motor 2 (Master) -> forward  (+)
                 * Motor 1 (Slave)  -> backward (-)
                 *
                 * Compare the magnitudes of the two movements.
                 */
                int error =  -relative_c2 - relative_c1;
                int pwm_slave = compute_slave_pwm(error);

                PWM_2_WriteCompare(MASTER_PWM);
                PWM_1_WriteCompare(pwm_slave);

                //uart_printf("LT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                //            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 >= target)
                {
                    Motor_1_IN_1_Write(1);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(1);
                }
                else
                {
                    uart_printf("lt");
                    advance_command_index(current_cmd->turn_delta_deg);
                }
                break;
            }
            case 'b'://backward
            {
                /* Each reverse grid ends using the fixed LEFT/RIGHT sensors. */
                drive_backward(relative_c1, relative_c2);
                break;
            }
            case 'r'://turn right
            {
                int target = current_cmd->param;

                /*
                 * Right turn:
                 * Motor 2 (Master) -> backward (-)
                 * Motor 1 (Slave)  -> forward (+)
                 *
                 * Compare the magnitudes of the two movements.
                 */
                int error = (relative_c2 + relative_c1);
                int pwm_slave = compute_slave_pwm(error);

                PWM_2_WriteCompare(MASTER_PWM);
                PWM_1_WriteCompare(pwm_slave);

                //uart_printf("RT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                //            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 <= target)
                {
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(1);
                    Motor_2_IN_3_Write(1);
                    Motor_2_IN_4_Write(0);
                }
                else
                {
                    uart_printf("rt");
                    advance_command_index(current_cmd->turn_delta_deg);
                }
                break;
            }      
            default:
                stop_motors();
                break;                
        }
    }
}

/* [] END OF FILE */