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
#define CAMERA_SETTLE_MS 600u
#define MOTOR_CONTROL_PERIOD_MS 100u
enum { CAMERA_SETTLING, CAMERA_WAIT_RESULT, CAMERA_READY, CAMERA_MOVING };
static volatile uint8 camera_state = CAMERA_SETTLING;
static volatile uint32 camera_ms = 0u;
static uint32 camera_stop_ms = 0u;
static int movement_steps_done = 0;
static void begin_camera_scan(void);

static void camera_tick(void)
{
    camera_ms++;
}

#define PULSES_PER_GRID  1900
#define PULSES_PER_DIAGONAL  2700
#define PULSES_PER_45DEGREE  1300
#define PULSES_PER_90DEGREE  2400
#define PULSES_PER_135DEGREE  3800
#define PULSES_PER_180DEGREE  5200
#define PULSES_PER_225DEGREE  6470
#define PULSES_PER_270DEGREE  7800
#define PULSES_PER_315DEGREE  9200
#define MASTER_PWM 230   /* fixed PWM for the master (reference) motor */
#define KP 0.15          /* P-controller gain: corrects the slave motor's
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

int angle_to_pulses(int angle)
{
    angle = abs(angle);

    switch (angle)
    {
        case 45:
            return PULSES_PER_45DEGREE;

        case 90:
            return PULSES_PER_90DEGREE;

        case 135:
            return PULSES_PER_135DEGREE;

        case 180:
            return PULSES_PER_180DEGREE;

        case 225:
            return PULSES_PER_225DEGREE;

        case 270:
            return PULSES_PER_270DEGREE;

        case 315:
            return PULSES_PER_315DEGREE;

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
        /* Store the raw grid-step count (signed for direction). The
         * actual pulse target depends on the heading at the moment
         * this command is executed, so it's computed at runtime in
         * main() instead of here. */
        int forward = (cmd_name[0] == 'f');

        queue[*count].type = forward ? 'f' : 'b';
        queue[*count].param = forward ? -value : value;
    }
    else if (strcmp(cmd_name, "lt") == 0 || strcmp(cmd_name, "rt") == 0)
    {
        int pulses = angle_to_pulses(value);

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
TurtleVariable variables[10] = {{"vowel", "blank"}};
int variable_count = 1;

char while_var_name[20];
char while_expected_value[20];
char while_body[100];

int while_condition_valid = 0;
TurtleCommand while_cmd_queue[30];
int while_cmd_count = 0;
int while_cmd_idx = 0;
int executing_while = 0;

/*
 * Actual physical heading in degrees (0 = whatever direction the
 * turtle starts facing, increases clockwise). Updated at runtime,
 * only when an l/r command finishes executing - not when it's
 * parsed/queued. This is what fd/bk look at (at the moment they
 * execute) to decide PULSES_PER_GRID vs PULSES_PER_DIAGONAL, so it
 * stays correct even inside a while-loop body that turns by
 * different amounts (45/90/135/...) on every pass.
 */
float current_heading_deg = 0.0f;

volatile int waiting_for_uart = 0;
volatile int uart_msg_received = 0;

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

    if (strcmp(actual_value, expected_value) == 0)
    {
        return 1;
    }

    return 0;
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

void parse_branch(char *text)
{
    char *p = text;

    while (*p != '\0' && cmd_count < 30)
    {
        while (*p == ' ')
        {
            p++;
        }

        if (*p == '\0')
        {
            break;
        }

        /* =========================
           repeat
           ========================= */
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

                    strncpy(repeat_content,
                            repeat_start + 1,
                            len);

                    repeat_content[len] = '\0';

                    for (int r = 0; r < rep_count; r++)
                    {
                        char temp_str[50];

                        strcpy(temp_str, repeat_content);

                        parse_branch(temp_str);
                    }

                    p = repeat_end + 1;
                }
                else
                {
                    break;
                }
            }
        }

        /* =========================
           normal command
           ========================= */
        else
        {
            char cmd_name[10];
            char value_str[20];

            p = read_token(p, cmd_name, sizeof(cmd_name));
            p = read_token(p, value_str, sizeof(value_str));
            int value = atoi(value_str);

            append_simple_command(cmd_queue, &cmd_count, 30, cmd_name, value);
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
    char *actual_value = get_variable(while_var_name);

    if (actual_value == NULL)
    {
        return 0;
    }

    if (strcmp(actual_value, while_expected_value) == 0)
    {
        return 1;
    }

    return 0;
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

    /* Keep the original command intact so REPEAT/WHILE can execute it again.
     * Each completed f/b segment is one grid, including diagonal headings.
     */
    TurtleCommand *finished = executing_while
        ? &while_cmd_queue[while_cmd_idx] : &cmd_queue[current_cmd_idx];
    if (finished->type == 'f' || finished->type == 'b')
    {
        movement_steps_done++;
        if (movement_steps_done < abs(finished->param))
        {
            request_reset_start = 1;
            return;
        }
    }
    movement_steps_done = 0;

    current_heading_deg += (float)heading_delta_deg;

    while (current_heading_deg < 0.0f)
    {
        current_heading_deg += 360.0f;
    }
    while (current_heading_deg >= 360.0f)
    {
        current_heading_deg -= 360.0f;
    }

    if (executing_while)
    {
        while_cmd_idx++;
    }
    else
    {
        current_cmd_idx++;

        if (current_cmd_idx >= cmd_count)
        {
            cmd_count = 0;
        }
    }

    request_reset_start = 1;
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
        pwm_slave = 0;
    }

    return pwm_slave;
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

CY_ISR(ISR_Handler_1)
{
    Rx = UART_1_GetChar();
    
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

            /* Check whether this line is MAKE */
            int is_make_command = (strncmp(p, "make", 4) == 0);

            /* The parser chooses IFELSE branches here, in the UART1 ISR.
             * Only accept a new command at a confirmed, stationary position.
             * Do not let a command overwrite an active multi-grid sequence.
             */
            if (camera_state != CAMERA_READY || waiting_for_uart ||
                executing_while || current_cmd_idx < cmd_count)
            {
                UART_1_PutString("BUSY: moving/scanning; retry command when stopped\r\n");
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

                    /* =========================
                       Read variable name
                       Example: :vowel
                       ========================= */

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

                        /* Skip spaces */
                        while (*p == ' ')
                        {
                            p++;
                        }

                        /* =========================
                           Read =
                           ========================= */

                        if (*p == '=')
                        {
                            p++;
                        }

                        while (*p == ' ')
                        {
                            p++;
                        }

                        /* =========================
                           Read expected value
                           Example: "a
                           ========================= */

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

                        /* =========================
                           Compare
                           ========================= */

                        int condition = compare_variable(var_name, expected_value);

                        uart_printf("IFELSE: %s == %s ? %d\r\n",
                                    var_name, expected_value, condition);
                        
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

                                        if (condition)
                                        {
                                            UART_1_PutString("IF TRUE\r\n");
                                            parse_branch(true_cmd);
                                        }
                                        else
                                        {
                                            UART_1_PutString("IF FALSE\r\n");
                                            parse_branch(false_cmd);
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
            if (!is_make_command && cmd_count > 0)
            {
                uart_printf("DEBUG: count=%d\r\n", cmd_count);

                for (int j = 0; j < cmd_count; j++)
                {
                    uart_printf("QUEUE[%d]: type=%c param=%d repeat=%d\r\n",
                                j,
                                cmd_queue[j].type,
                                cmd_queue[j].param,
                                cmd_queue[j].repeat_cnt);
                }

                request_reset_start = 1;
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

        /* Wake the existing WHILE wait, including repeated detections. */
        uart_msg_received = 1;
        camera_state = CAMERA_READY;
    }
    CyExitCriticalSection(interrupt_state);
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

    /* Bound service time even if the camera transmits continuously. */
    if (UART_2_GetRxBufferSize() > 0u)
    {
        camera_rx = UART_2_GetChar();
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
                if (report_result) UART_2_PutString(camera_message);
                last_reported = camera_rx;
                break;

            case 'N':
                update_camera_vowel(camera_rx);
                if (report_result) UART_2_PutString("NO VOWELS DETECTED\r\n");
                last_reported = camera_rx;
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
    UART_2_PutString("UART2 READY\r\n");
    isr_1_StartEx(ISR_Handler_1);
    UART_1_PutString("START...........");
    UART_1_PutString("\n");
    int counter1 = 0;
    int counter2 = 0;

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
    uint32 last_motor_update_ms = camera_ms;
    
    for(;;)
    { 
        /* Place your application code here. */
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
            }
            else
            {
                UART_1_PutString("WHILE END\r\n");

                current_cmd_idx++;
                request_reset_start = 1;

                uart_printf("AFTER WHILE: idx=%d count=%d\r\n",
                            current_cmd_idx, cmd_count);
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

        if (executing_while)
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
            case 'w': //while
            {
                if (while_condition_true())
                {
                    UART_1_PutString("WHILE TRUE\r\n");

                    while_cmd_idx = 0;
                    executing_while = 1;
                    request_reset_start = 1;
                }
                else
                {
                    UART_1_PutString("WHILE FALSE\r\n");
                    
                    executing_while = 0;

                    current_cmd_idx++;
                }
            }
            break;
            case 'f'://forward
            {
                /* param holds a signed grid-step count; pick the pulse
                 * count per step from the *current* heading so this
                 * stays correct across repeated while-loop passes. */
                int pulses_per_step = heading_is_diagonal(current_heading_deg)
                                           ? PULSES_PER_DIAGONAL
                                           : PULSES_PER_GRID;
                /* One grid per segment; advance_command_index tracks the rest. */
                int target = (current_cmd->param < 0 ? -1 : 1) * pulses_per_step;

                /*
                 * P-control (KP) keeps the two wheels' encoder counts
                 * in sync so the robot drives straight: master motor
                 * runs at a fixed MASTER_PWM, slave motor's PWM is
                 * nudged up/down by KP * error.
                 *
                 * Forward motion drives relative_c2 (and relative_c1)
                 * DOWN toward a negative target, so "ahead" means
                 * "more negative". error = slave - master: if slave
                 * has gone more negative than master (slave ahead),
                 * error < 0 -> slave PWM decreases (slow down); if
                 * slave lags (less negative than master), error > 0
                 * -> slave PWM increases (speed up). Negative feedback.
                 */
                int error = relative_c1 - relative_c2;
                int pwm_slave = compute_slave_pwm(error);

                PWM_2_WriteCompare(MASTER_PWM);
                PWM_1_WriteCompare(pwm_slave);

                uart_printf("M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 >= target)
                {
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(1);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(1);
                }
                else
                {
                    advance_command_index(0);
                }
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

                uart_printf("LT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 >= target)
                {
                    Motor_1_IN_1_Write(1);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(1);
                }
                else
                {
                    advance_command_index(current_cmd->turn_delta_deg);
                }
                break;
            }
            case 'b'://backward
            {
                /* param holds a signed grid-step count; pick the pulse
                 * count per step from the *current* heading, same as
                 * the 'f' case above. */
                int pulses_per_step = heading_is_diagonal(current_heading_deg)
                                           ? PULSES_PER_DIAGONAL
                                           : PULSES_PER_GRID;
                /* One grid per segment; advance_command_index tracks the rest. */
                int target = (current_cmd->param < 0 ? -1 : 1) * pulses_per_step;

                int error = relative_c2 - relative_c1;
                int pwm_slave = compute_slave_pwm(error);

                PWM_2_WriteCompare(MASTER_PWM);
                PWM_1_WriteCompare(pwm_slave);

                uart_printf("M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 <= target)
                {
                    Motor_1_IN_1_Write(1);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(1);
                    Motor_2_IN_4_Write(0);
                }
                else
                {
                    advance_command_index(0);
                }
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

                uart_printf("RT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                            relative_c2, relative_c1, error, pwm_slave, MASTER_PWM);

                if (relative_c2 <= target)
                {
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(1);
                    Motor_2_IN_3_Write(1);
                    Motor_2_IN_4_Write(0);
                }
                else
                {
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