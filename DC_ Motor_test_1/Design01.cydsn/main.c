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
// lol thisis a test

char Rx = '\0';
char string_1[200]; //debug
char debug_frame[300];
static void (*uart1_raw_put_string)(const char *) = UART_1_PutString;

void send_debug_message(const char *text)
{
    /*
     * UART1 is kept for the existing motor/Bluetooth/debug path only.
     * It is NOT used to send ESP32-S3 camera data to Termite.
     */
    snprintf(debug_frame, sizeof(debug_frame), "T:%s", text);
    uart1_raw_put_string(debug_frame);
    UART_1_PutChar((uint8)0x00);
}

void send_terminal_format(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    vsnprintf(string_1, sizeof(string_1), format, args);
    va_end(args);

    send_debug_message(string_1);
}

// #define UART_1_PutString send_debug_message

int start_c1 = 0;
int start_c2 = 0;
volatile int request_reset_start = 0;
char rx_buffer[256];
uint8 rx_index = 0;
int cmd_code = 0;

#define PULSES_PER_GRID  1900
#define PULSES_PER_DIAGONAL  2700
#define PULSES_PER_45DEGREE  1300
#define PULSES_PER_90DEGREE  2400
#define PULSES_PER_135DEGREE  3800
#define PULSES_PER_180DEGREE  5200
#define PULSES_PER_225DEGREE  6470
#define PULSES_PER_270DEGREE  7800
#define PULSES_PER_315DEGREE  9200
#define MASTER_PWM 230
#define KP 0.15

/* grid / obstacle model (the old Xiao ESP32C3 LOGO interpreter is
   retired; its grid/position/heading tracking and bounds/obstacle/vowel
   checks now live here, applied to the fd/bk/lt/rt commands parsed
   straight off UART_1) */
#define GRID_SIZE      20
#define CELL_EMPTY     0
#define CELL_VISITED   1
#define CELL_OBSTACLE  2
#define CELL_SOURCE    3
#define CELL_DEST      4
#define CELL_VOWEL_A   10
#define CELL_VOWEL_E   11
#define CELL_VOWEL_I   12
#define CELL_VOWEL_O   13
#define CELL_VOWEL_U   14

typedef struct {
    char type;      
    int param;     
    int repeat_cnt;
    int grid_steps;
} TurtleCommand;

typedef struct {
    char variable[20];
    char expected[20];
    char true_body[100];
    char false_body[100];
} IfElseCommand;

typedef struct {
    char name[20];
    char value[20];
} AssignmentCommand;

typedef struct {
    char variable[20];
    char expected[20];
    char body[100];
} WhileDefinition;

typedef struct {
    TurtleCommand queue[100];
    int count;
    int index;
    char variable[20];
    char expected[20];
} WhileContext;

TurtleCommand cmd_queue[30];
volatile int cmd_count = 0;
volatile int current_cmd_idx = 0;
IfElseCommand ifelse_queue[30];
int ifelse_count = 0;
AssignmentCommand assignment_queue[30];
int assignment_count = 0;
WhileDefinition while_definitions[30];
int while_definition_count = 0;
WhileContext while_context_stack[4];
int while_context_depth = 0;

/* function prototype */
char* find_matching_bracket(char *start);
char* queue_while_command(char *text,
                          TurtleCommand *queue,
                          volatile int *count);
void send_vowel_message(char *vowel);

typedef struct {
    char name[20];
    char value[20];
} TurtleVariable;

TurtleVariable variables[10];
int variable_count = 0;

char while_var_name[20];
char while_expected_value[20];
char while_body[100];
char if_var_name[20];
char if_expected_value[20];
char if_true_body[100];
char if_false_body[100];
int while_condition_valid = 0;
TurtleCommand while_cmd_queue[30];
int while_cmd_count = 0;
int while_cmd_idx = 0;
int executing_while = 0;

volatile int uart_msg_received = 0;
volatile int waiting_for_uart = 0;
volatile unsigned long camera_update_count = 0;
unsigned long camera_updates_consumed = 0;

/* vowel value received from the camera module over UART_2 */
char current_vowel[8] = "blank";
char rx_buffer_2[40];
uint8 rx_index_2 = 0;

/* grid/turtle state */
uint8 grid[GRID_SIZE][GRID_SIZE];
uint8 planning_grid[GRID_SIZE][GRID_SIZE];
int posX = GRID_SIZE / 2;
int posY = GRID_SIZE / 2;
float heading_deg = 0.0f; /* 0 = +Y, grows clockwise */
int planning_posX = GRID_SIZE / 2;
int planning_posY = GRID_SIZE / 2;
float planning_heading_deg = 0.0f;

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

char* resolve_expression(char *expression)
{
    char *current = expression;
    int max_depth = 5; /* Limit resolution depth to prevent infinite loops */

    while (current != NULL && current[0] == ':' && max_depth > 0)
    {
        current = get_variable(current + 1);
        max_depth--;
    }

    return current;
}

int expression_int(char *expression)
{
    if (expression == NULL)
    {
        return 0;
    }

    char *value = resolve_expression(expression);
    if (value == NULL)
    {
        return 0;
    }

    return atoi(value);
}

void assign_variable(char *name, char *value)
{
    char *resolved_value = resolve_expression(value);
    if (resolved_value == NULL)
    {
        resolved_value = value;
    }

    for (int i = 0; i < variable_count; i++)
    {
        if (strcmp(variables[i].name, name) == 0)
        {
            strncpy(variables[i].value,
                    resolved_value,
                    sizeof(variables[i].value) - 1);
            variables[i].value[sizeof(variables[i].value) - 1] = '\0';
            return;
        }
    }

    if (variable_count < 10)
    {
        strncpy(variables[variable_count].name,
                name,
                sizeof(variables[variable_count].name) - 1);
        variables[variable_count].name[sizeof(variables[variable_count].name) - 1] = '\0';
        strncpy(variables[variable_count].value,
                resolved_value,
                sizeof(variables[variable_count].value) - 1);
        variables[variable_count].value[sizeof(variables[variable_count].value) - 1] = '\0';
        variable_count++;
    }
}

void queue_assignment(TurtleCommand *queue,
                      volatile int *count,
                      int max_count,
                      char *name,
                      char *value)
{
    if (*count >= max_count || assignment_count >= 30)
    {
        return;
    }

    strncpy(assignment_queue[assignment_count].name,
            name,
            sizeof(assignment_queue[assignment_count].name) - 1);
    assignment_queue[assignment_count].name[sizeof(assignment_queue[assignment_count].name) - 1] = '\0';
    strncpy(assignment_queue[assignment_count].value,
            value,
            sizeof(assignment_queue[assignment_count].value) - 1);
    assignment_queue[assignment_count].value[sizeof(assignment_queue[assignment_count].value) - 1] = '\0';

    queue[*count].type = 'm';
    queue[*count].param = assignment_count++;
    queue[*count].repeat_cnt = 1;
    (*count)++;
}

//compare function
int compare_variable(char *var_name, char *expected_value)
{
    char *actual_value = get_variable(var_name);
    char *expected = expected_value;

    if (expected_value[0] == ':')
    {
        expected = get_variable(expected_value + 1);
    }

    if (actual_value == NULL || expected == NULL)
    {
        return 0;
    }

    if (strcmp(actual_value, expected) == 0)
    {
        return 1;
    }

    return 0;
}

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

/* ---- grid / obstacle checking ---- */

int in_bounds(int x, int y)
{
    return x >= 1 && x <= GRID_SIZE && y >= 1 && y <= GRID_SIZE;
}


void normalize_heading(void)
{
    while (heading_deg < 0.0f)    heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
}

int is_vowel_cell(uint8 cellValue)
{
    return cellValue >= CELL_VOWEL_A && cellValue <= CELL_VOWEL_U;
}

uint8 vowel_cell(char *letter)
{
    if (strcmp(letter, "a") == 0) return CELL_VOWEL_A;
    if (strcmp(letter, "e") == 0) return CELL_VOWEL_E;
    if (strcmp(letter, "i") == 0) return CELL_VOWEL_I;
    if (strcmp(letter, "o") == 0) return CELL_VOWEL_O;
    if (strcmp(letter, "u") == 0) return CELL_VOWEL_U;
    return CELL_EMPTY;
}

void reset_grid(void)
{
    memset(grid, CELL_EMPTY, sizeof(grid));
    
    /* 1-based indexing: Center at (10, 10) */
    posX = GRID_SIZE / 2;       // 10
    posY = GRID_SIZE / 2;       // 10
    heading_deg = 0.0f;
    
    /* Map (1..20) down to array indices [0..19] */
    grid[posY - 1][posX - 1] = CELL_VISITED;
    
    memcpy(planning_grid, grid, sizeof(grid));
    planning_posX = posX;
    planning_posY = posY;
    planning_heading_deg = heading_deg;
}

void sync_planning_state(void)
{
    memcpy(planning_grid, grid, sizeof(grid));
    planning_posX = posX;
    planning_posY = posY;
    planning_heading_deg = heading_deg;
}

/* attempts to advance the turtle by one grid cell along the current
   heading (forward) or its reverse (backward); on success updates
   posX/posY and returns 1. On refusal returns 0 and sets *reason to
   "edge"/"obstacle"/"vowel". */
int try_step_grid(int forward, const char **reason)
{
    float rad = planning_heading_deg * 3.14159265f / 180.0f;
    int dx = (int)lroundf(sinf(rad));
    
    /* North (0 deg) moves UP toward Row 1 (decreasing Y) */
    int dy = -(int)lroundf(cosf(rad)); 

    if (!forward)
    {
        dx = -dx;
        dy = -dy;
    }

    int targetX = planning_posX + dx;
    int targetY = planning_posY + dy;

    if (!in_bounds(targetX, targetY))
    {
        *reason = "edge";
        return 0;
    }

    /* Map 1-based coordinates directly to array indices [0..19] */
    if (planning_grid[targetY - 1][targetX - 1] == CELL_OBSTACLE)
    {
        *reason = "obstacle";
        return 0;
    }


    int isDiagonalStep = (dx != 0) && (dy != 0);

    if (isDiagonalStep)
    {
        /* Prevent squeezing diagonally between two adjacent obstacles */
        if (planning_grid[planning_posY - 1][targetX - 1] == CELL_OBSTACLE ||
            planning_grid[targetY - 1][planning_posX - 1] == CELL_OBSTACLE)
        {
            *reason = "obstacle";
            return 0;
        }
    }

    if (isDiagonalStep && is_vowel_cell(planning_grid[targetY - 1][targetX - 1]))
    {
        *reason = "vowel";
        return 0;
    }

    planning_posX = targetX;
    planning_posY = targetY;

    if (planning_grid[planning_posY - 1][planning_posX - 1] == CELL_EMPTY)
    {
        planning_grid[planning_posY - 1][planning_posX - 1] = CELL_VISITED;
    }

    return 1;
}

/* dumps the grid over UART_1 (Bluetooth/debug) */
void print_grid_uart1(void)
{
    char row[GRID_SIZE + 3];

    /* y_index = 0 is Row 1 (Top), y_index = 19 is Row 20 (Bottom) */
    for (int y_index = 0; y_index < GRID_SIZE; y_index++)
    {
        int y = y_index + 1; // 1-based Y coordinate (1 to 20)

        for (int x_index = 0; x_index < GRID_SIZE; x_index++)
        {
            int x = x_index + 1; // 1-based X coordinate (1 to 20)
            char c;

            if (x == posX && y == posY)
            {
                c = 'T';
            }
            else
            {
                switch (grid[y_index][x_index])
                {
                    case CELL_EMPTY:    c = '.'; break;
                    case CELL_VISITED:  c = '*'; break;
                    case CELL_OBSTACLE: c = '#'; break;
                    case CELL_SOURCE:   c = 'S'; break;
                    case CELL_DEST:     c = 'D'; break;
                    case CELL_VOWEL_A:  c = 'a'; break;
                    case CELL_VOWEL_E:  c = 'e'; break;
                    case CELL_VOWEL_I:  c = 'i'; break;
                    case CELL_VOWEL_O:  c = 'o'; break;
                    case CELL_VOWEL_U:  c = 'u'; break;
                    default:            c = '?'; break;
                }
            }

            row[x_index] = c;
        }

        row[GRID_SIZE] = '\r';
        row[GRID_SIZE + 1] = '\n';
        row[GRID_SIZE + 2] = '\0';
        UART_1_PutString(row);
    }

    sprintf(string_1, "pos=(%d,%d) heading=%d\r\n", posX, posY, (int)heading_deg);
    UART_1_PutString(string_1);
}

/* Turns one "fd/bk/lt/rt <n>" into a queue entry, validating fd/bk
   against the grid first (bounds/obstacle/diagonal-vowel) and only
   queuing however many cells actually passed -- shared by every place
   that builds a movement queue (top-level commands, repeat content,
   ifelse branches, while body) so the grid check is written once.
   lt/rt always succeed (no bounds check for turning) and update
   heading_deg immediately. Reports a BLOCKED reason over UART_1 if
   fd/bk couldn't fully complete. */
void enqueue_movement(TurtleCommand *queue, volatile int *count, int max_count, char cmd_type, int target_val)
{
    if (*count >= max_count)
    {
        return;
    }

    if (cmd_type == 'f' || cmd_type == 'b')
    {
        int forward = (cmd_type == 'f');
        int achieved = 0;
        const char *reason = "";

        for (int i = 0; i < target_val; i++)
        {
            if (!try_step_grid(forward, &reason))
            {
                break;
            }
            achieved++;
        }

        if (achieved != target_val)
        {
            sprintf(string_1, "BLOCKED %d %s\r\n", achieved, reason);
            UART_1_PutString(string_1);
        }

        if (achieved > 0)
        {
            float rad =
                planning_heading_deg * 3.14159265f / 180.0f;

            int dx = (int)lroundf(sinf(rad));
            int dy = (int)lroundf(cosf(rad));

            int pulse_count;

            if ((dx != 0) && (dy != 0))
            {
                pulse_count = PULSES_PER_DIAGONAL;
            }
            else
            {
                pulse_count = PULSES_PER_GRID;
            }

            for (int i = 0; i < achieved && *count < max_count; i++)
            {
                queue[*count].type = cmd_type;

                queue[*count].param =
                    (cmd_type == 'f') ? -pulse_count : pulse_count;

                queue[*count].repeat_cnt = 1;
                queue[*count].grid_steps = 1;

                (*count)++;
            }
        }
    }
    else if (cmd_type == 'l' || cmd_type == 'r')
    {
        queue[*count].type = cmd_type;
        int pulse_count = angle_to_pulses(target_val);
        queue[*count].param =
            (cmd_type == 'l') ? pulse_count : -pulse_count;
        queue[*count].repeat_cnt = 1;
        queue[*count].grid_steps = 0;
        (*count)++;

        planning_heading_deg += (cmd_type == 'l') ? -(float)target_val : (float)target_val;
        while (planning_heading_deg < 0.0f) planning_heading_deg += 360.0f;
        while (planning_heading_deg >= 360.0f) planning_heading_deg -= 360.0f;
    }
}

char* queue_ifelse_command(char *text,
                           TurtleCommand *queue,
                           volatile int *count,
                           int max_count)
{
    char *p = text + 6;
    char *true_start;
    char *true_end;
    char *false_start;
    char *false_end;
    IfElseCommand *conditional;
    int length;

    while (*p == ' ') p++;
    if (*p != ':') return NULL;
    p++;

    if (ifelse_count >= 30 || *count >= max_count) return NULL;
    conditional = &ifelse_queue[ifelse_count];

    length = 0;
    while (*p != ' ' && *p != '\0' && length < sizeof(conditional->variable) - 1)
    {
        conditional->variable[length++] = *p++;
    }
    conditional->variable[length] = '\0';

    while (*p == ' ') p++;
    if (*p == '=') p++;
    while (*p == ' ') p++;
    if (*p == '"') p++;

    length = 0;
    while (*p != ' ' && *p != '\0' && length < sizeof(conditional->expected) - 1)
    {
        conditional->expected[length++] = *p++;
    }
    conditional->expected[length] = '\0';

    while (*p == ' ') p++;
    if (*p != '[') return NULL;
    true_start = p + 1;
    true_end = find_matching_bracket(p);
    if (true_end == NULL) return NULL;

    p = true_end + 1;
    while (*p == ' ') p++;
    if (*p != '[') return NULL;
    false_start = p + 1;
    false_end = find_matching_bracket(p);
    if (false_end == NULL) return NULL;

    length = true_end - true_start;
    if (length >= sizeof(conditional->true_body))
    {
        length = sizeof(conditional->true_body) - 1;
    }
    strncpy(conditional->true_body, true_start, length);
    conditional->true_body[length] = '\0';

    length = false_end - false_start;
    if (length >= sizeof(conditional->false_body))
    {
        length = sizeof(conditional->false_body) - 1;
    }
    strncpy(conditional->false_body, false_start, length);
    conditional->false_body[length] = '\0';

    queue[*count].type = 'i';
    queue[*count].param = ifelse_count++;
    queue[*count].repeat_cnt = 1;
    (*count)++;

    return false_end + 1;
}

void parse_branch_queue(char *text,
                        TurtleCommand *queue,
                        volatile int *count)
{
    char *p = text;

    while (*p != '\0' && *count < 30)
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
              nested while
              ========================= */
          if (strncmp(p, "while", 5) == 0)
          {
                char *next = queue_while_command(p, queue, count);
                if (next == NULL)
                {
                     break;
                }
                p = next;
          }
          /* =========================
              nested ifelse
              ========================= */
          else if (strncmp(p, "ifelse", 6) == 0)
          {
                char *next = queue_ifelse_command(p, queue, count, 30);
                if (next == NULL)
                {
                     break;
                }
                p = next;
          }
          /* =========================
              make
              ========================= */
          else if (strncmp(p, "make", 4) == 0)
        {
            char name[20];
            char value[20];
            int length = 0;

            p += 4;
            while (*p == ' ') p++;
            if (*p == '"') p++;

            while (*p != ' ' && *p != '\0' && length < sizeof(name) - 1)
            {
                name[length++] = *p++;
            }
            name[length] = '\0';

            while (*p == ' ') p++;
            if (*p == '"') p++;

            length = 0;
            while (*p != ' ' && *p != '\0' && length < sizeof(value) - 1)
            {
                value[length++] = *p++;
            }
            value[length] = '\0';
            queue_assignment(queue, count, 30, name, value);
        }
        /* =========================
           repeat
           ========================= */
        else if (strncmp(p, "repeat", 6) == 0)
        {
            p += 6;

            while (*p == ' ')
            {
                p++;
            }

            int rep_count = expression_int(p);

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

                        parse_branch_queue(temp_str, queue, count);
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
            int i = 0;

            /* get command name */
            while (*p != ' ' &&
                   *p != '\0' &&
                   i < 9)
            {
                cmd_name[i++] = *p;
                p++;
            }

            cmd_name[i] = '\0';

            while (*p == ' ')
            {
                p++;
            }

            /* get parameter */
            int value = expression_int(p);

            while (*p != ' ' && *p != '\0')
            {
                p++;
            }

            /* add command */
            if (strcmp(cmd_name, "fd") == 0)
            {
                enqueue_movement(queue, count, 30, 'f', value);
            }
            else if (strcmp(cmd_name, "bk") == 0)
            {
                enqueue_movement(queue, count, 30, 'b', value);
            }
            else if (strcmp(cmd_name, "lt") == 0)
            {
                enqueue_movement(queue, count, 30, 'l', value);
            }
            else if (strcmp(cmd_name, "rt") == 0)
            {
                enqueue_movement(queue, count, 30, 'r', value);
            }
        }
    }
}

void parse_branch(char *text)
{
    parse_branch_queue(text, cmd_queue, &cmd_count);
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

/* reports the current vowel over Bluetooth (UART_1) */
void send_vowel_message(char *vowel)
{
    sprintf(string_1, "V:%s", vowel);
    uart1_raw_put_string(string_1);
    UART_1_PutChar((uint8)0x00);
}

void debug_camera_uart_message(char *message)
{
    sprintf(string_1,
            "CAMERA UART2 RX P15[5]: %s\r\n",
            message);
    UART_1_PutString(string_1);
}

int condition_true(char *variable, char *expected_value)
{
    char *actual_value = get_variable(variable);
    char *expected = expected_value;

    if (expected_value[0] == ':')
    {
        expected = get_variable(expected_value + 1);
    }

    if (actual_value == NULL || expected == NULL)
    {
        return 0;
    }

    if (strcmp(actual_value, expected) == 0)
    {
        return 1;
    }

    return 0;
}

int while_condition_true(void)
{
    return condition_true(while_var_name, while_expected_value);
}

void update_vowel_variable(char *vowel)
{
    assign_variable("vowel", vowel);
    camera_update_count++;
}

void commit_completed_movement(TurtleCommand *command)
{
    if (command->type == 'l' || command->type == 'r')
    {
        int turn_angle = angle_to_pulses(command->param);

        heading_deg += (command->type == 'l') ?
                               -(float)turn_angle :
                               (float)turn_angle;

        normalize_heading();
        return;
    }

    if ((command->type == 'f' || command->type == 'b') && command->grid_steps > 0)
    {
        int forward = (command->type == 'f');

        for (int step = 0; step < command->grid_steps; step++)
        {
            float rad = heading_deg * 3.14159265f / 180.0f;
            int dx = (int)lroundf(sinf(rad));
            int dy = -(int)lroundf(cosf(rad));

            if (!forward)
            {
                dx = -dx;
                dy = -dy;
            }

           /* Inside the (type == 'f' || type == 'b') movement loop: */
            posX += dx;
            posY += dy;

            if (in_bounds(posX, posY) && grid[posY - 1][posX - 1] == CELL_EMPTY)
            {
                grid[posY - 1][posX - 1] = CELL_VISITED;
            }
        }
    }
}

void handle_camera_byte(uint8 received)
{
    char vowel[2];
    char camera_message[40];

    if (received == 'N')
    {
        strcpy(current_vowel, "NONE");
        update_vowel_variable(current_vowel);

        /* Keep existing UART1/Bluetooth vowel report separate. */
        send_vowel_message(current_vowel);

        /* Termite output goes through UART2. */
        UART_2_PutString("NO VOWELS DETECTED\r\n");
        return;
    }

    if (received >= 'a' && received <= 'z')
    {
        received = (uint8)(received - 'a' + 'A');
    }

    if (received != 'A' && received != 'E' && received != 'I' &&
        received != 'O' && received != 'U')
    {
        return;
    }

    vowel[0] = (char)(received - 'A' + 'a');
    vowel[1] = '\0';

    strcpy(current_vowel, vowel);
    update_vowel_variable(current_vowel);

    /* Existing UART1/Bluetooth message. */
    send_vowel_message(current_vowel);

    /* Display the confirmed camera result on Termite using UART2. */
    sprintf(camera_message, "CAMERA DETECTED: %c\r\n", (char)received);
    UART_2_PutString(camera_message);
}

/* Kept for compatibility with the original line-oriented camera path. */
void update_vowel_variable_legacy(char *vowel)
{
    for (int i = 0; i < variable_count; i++)
    {
        if (strcmp(variables[i].name, "vowel") == 0)
        {
            strncpy(variables[i].value, vowel, sizeof(variables[i].value) - 1);
            variables[i].value[sizeof(variables[i].value) - 1] = '\0';
            uart_msg_received = 1;
            return;
        }
    }
}

char* queue_while_command(char *text,
                          TurtleCommand *queue,
                          volatile int *count)
{
    char *p = text + 5;
    char *body_start;
    char *body_end;
    WhileDefinition *definition;
    int length;

    if (while_definition_count >= 30 || *count >= 30)
    {
        return NULL;
    }

    while (*p == ' ') p++;
    if (*p != ':') return NULL;
    p++;

    definition = &while_definitions[while_definition_count];
    length = 0;
    while (*p != ' ' && *p != '\0' && length < sizeof(definition->variable) - 1)
    {
        definition->variable[length++] = *p++;
    }
    definition->variable[length] = '\0';

    while (*p == ' ') p++;
    if (*p == '=') p++;
    while (*p == ' ') p++;
    if (*p == '"') p++;

    length = 0;
    while (*p != ' ' && *p != '\0' && length < sizeof(definition->expected) - 1)
    {
        definition->expected[length++] = *p++;
    }
    definition->expected[length] = '\0';

    while (*p == ' ') p++;
    if (*p != '[') return NULL;
    body_start = p + 1;
    body_end = find_matching_bracket(p);
    if (body_end == NULL) return NULL;

    length = body_end - body_start;
    if (length >= sizeof(definition->body))
    {
        length = sizeof(definition->body) - 1;
    }
    strncpy(definition->body, body_start, length);
    definition->body[length] = '\0';

    queue[*count].type = 'w';
    queue[*count].param = while_definition_count++;
    queue[*count].repeat_cnt = 1;
    (*count)++;

    return body_end + 1;
}

void parse_while_queue_body(char *text,
                            TurtleCommand *queue,
                            volatile int *count)
{
    char *p = text;

    while (*p != '\0' && *count < 30)
    {
        while (*p == ' ') p++;
        if (*p == '\0') break;

        if (strncmp(p, "while", 5) == 0)
        {
            char *next = queue_while_command(p, queue, count);
            if (next == NULL)
            {
                break;
            }
            p = next;
            continue;
        }

        if (strncmp(p, "ifelse", 6) == 0)
        {
            char *next = queue_ifelse_command(p, queue, count, 30);
            if (next == NULL)
            {
                break;
            }
            p = next;
            continue;
        }

        if (strncmp(p, "repeat", 6) == 0)
        {
            int repeat_count;
            char *repeat_start;
            char *repeat_end;

            p += 6;
            while (*p == ' ') p++;
            repeat_count = expression_int(p);
            while (*p != ' ' && *p != '\0') p++;
            while (*p == ' ') p++;

            if (*p != '[')
            {
                break;
            }

            repeat_start = p + 1;
            repeat_end = find_matching_bracket(p);
            if (repeat_end == NULL)
            {
                break;
            }

            *repeat_end = '\0';
            for (int repeat_index = 0;
                 repeat_index < repeat_count && *count < 30;
                 repeat_index++)
            {
                parse_while_queue_body(repeat_start, queue, count);
            }
            *repeat_end = ']';
            p = repeat_end + 1;
            continue;
        }

        if (strncmp(p, "make", 4) == 0)
        {
            char name[20];
            char value[20];
            int length = 0;

            p += 4;
            while (*p == ' ') p++;
            if (*p == '"') p++;
            while (*p != ' ' && *p != '\0' && length < sizeof(name) - 1)
            {
                name[length++] = *p++;
            }
            name[length] = '\0';
            while (*p == ' ') p++;
            if (*p == '"') p++;
            length = 0;
            while (*p != ' ' && *p != '\0' && length < sizeof(value) - 1)
            {
                value[length++] = *p++;
            }
            value[length] = '\0';
            queue_assignment(queue, count, 30, name, value);
            continue;
        }

        char cmd_name[10];
        int i = 0;
        while (*p != ' ' && *p != '\0' && i < 9)
        {
            cmd_name[i++] = *p++;
        }
        cmd_name[i] = '\0';
        while (*p == ' ') p++;

        int value = expression_int(p);
        while (*p != ' ' && *p != '\0') p++;

        if (strcmp(cmd_name, "fd") == 0)
        {
            enqueue_movement(queue, count, 30, 'f', value);
        }
        else if (strcmp(cmd_name, "bk") == 0)
        {
            enqueue_movement(queue, count, 30, 'b', value);
        }
        else if (strcmp(cmd_name, "lt") == 0)
        {
            enqueue_movement(queue, count, 30, 'l', value);
        }
        else if (strcmp(cmd_name, "rt") == 0)
        {
            enqueue_movement(queue, count, 30, 'r', value);
        }
    }
}

void parse_while_body(char *text)
{
    while_cmd_count = 0;
    parse_while_queue_body(text, while_cmd_queue, &while_cmd_count);
}

int parse_x_coord(const char *str)
{
    if (str == NULL || *str == '\0') return -1;

    /* Handle single character letter inputs (e.g., 'A'..'T' or 'a'..'t') */
    if (isalpha((unsigned char)str[0]) && (str[1] == ' ' || str[1] == '\0' || str[1] == '\r' || str[1] == '\n'))
    {
        char c = toupper((unsigned char)str[0]);
        if (c >= 'A' && c <= 'T')
        {
            return (c - 'A') + 1; // 'A' -> 1, 'B' -> 2 ... 'T' -> 20
        }
    }

    /* Fallback to standard integer parsing for numeric inputs "1".."20" */
    return atoi(str);
}

CY_ISR(ISR_Handler_1)
{
    Rx = UART_1_GetChar();

    if (Rx == '\0')
    {
        if (rx_index == 0)
        {
            return;
        }
    }

    if (Rx == '\r' || Rx == '\n' || Rx == '\0')
    {
        if (rx_index == 0)
        {
            return;
        }

        if (Rx != '\0')
        {
            UART_1_PutString("\n");
        }
        rx_buffer[rx_index] = '\0';
        //current_cmd_idx = 0;
        
        //cmd_count = 0;
        //current_cmd_idx = 0;

        char *p = rx_buffer;
        char message_flag = '\0';

        if (rx_index >= 2 && p[1] == ':' &&
            (p[0] == 'S' || p[0] == 'T' || p[0] == 'A' || p[0] == 'V'))
        {
            message_flag = p[0];
            p += 2;

            for (char *q = p; *q != '\0'; q++)
            {
                if (*q == '\r' || *q == '\n')
                {
                    *q = ' ';
                }
            }
        }

        if (message_flag == 'A' || message_flag == 'V')
        {
            rx_index = 0;
            return;
        }

        /* Check whether this line is MAKE */
        int is_make_command = (strncmp(p, "make", 4) == 0);

            /* New movement/program command starts a new queue */
            if (!is_make_command || message_flag == 'S')
            {
                cmd_count = 0;
                current_cmd_idx = 0;
                ifelse_count = 0;
                assignment_count = 0;
                while_definition_count = 0;
                while_context_depth = 0;
                sync_planning_state();
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
                    int rep_count = expression_int(p);

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
                        char *end = find_matching_bracket(p - 1);

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
                                parse_branch(temp_str);
                                
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

                    queue_assignment(cmd_queue,
                                     &cmd_count,
                                     30,
                                     var_name,
                                     value);
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
                        sprintf(string_1,
                                "VARIABLE: %s = %s\r\n",
                                var_name,
                                value);
                        UART_1_PutString(string_1);
                    }
                    else
                    {
                        sprintf(string_1,
                                "VARIABLE NOT FOUND: %s\r\n",
                                var_name);
                        UART_1_PutString(string_1);
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

                                        if (true_len >= sizeof(if_true_body))
                                        {
                                            true_len = sizeof(if_true_body) - 1;
                                        }
                                        if (false_len >= sizeof(if_false_body))
                                        {
                                            false_len = sizeof(if_false_body) - 1;
                                        }

                                        if (ifelse_count >= 30)
                                        {
                                            break;
                                        }

                                        strncpy(ifelse_queue[ifelse_count].variable,
                                            var_name,
                                            sizeof(ifelse_queue[ifelse_count].variable) - 1);
                                        ifelse_queue[ifelse_count].variable[sizeof(ifelse_queue[ifelse_count].variable) - 1] = '\0';
                                        strncpy(ifelse_queue[ifelse_count].expected,
                                            expected_value,
                                            sizeof(ifelse_queue[ifelse_count].expected) - 1);
                                        ifelse_queue[ifelse_count].expected[sizeof(ifelse_queue[ifelse_count].expected) - 1] = '\0';
                                        strncpy(ifelse_queue[ifelse_count].true_body, true_start, true_len);
                                        ifelse_queue[ifelse_count].true_body[true_len] = '\0';
                                        strncpy(ifelse_queue[ifelse_count].false_body, false_start, false_len);
                                        ifelse_queue[ifelse_count].false_body[false_len] = '\0';

                                        p = false_end + 1;
                                        cmd_queue[cmd_count].type = 'i';
                                        cmd_queue[cmd_count].param = ifelse_count++;
                                        cmd_queue[cmd_count].repeat_cnt = 1;
                                        cmd_count++;
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
                            cmd_queue[cmd_count].param = -1;
                            cmd_queue[cmd_count].repeat_cnt = 1;

                            cmd_count++;
                            parse_while_body(while_body);
                            sprintf(string_1, "WHILE CMD COUNT=%d\r\n", while_cmd_count);
                            UART_1_PutString(string_1);
                            
                            sprintf(string_1,
                                    "WHILE: %s == %s\r\n",
                                    while_var_name,
                                    while_expected_value);

                            UART_1_PutString(string_1);

                            sprintf(string_1,
                                    "WHILE BODY=[%s]\r\n",
                                    while_body);

                            UART_1_PutString(string_1);

                            /*
                             * Move p to the command AFTER
                             * the while body.
                             */
                            p = body_end + 1;
                        }
                    }
                }

                /* =========================
                   GRID / OBSTACLE CONFIG
                   ========================= */
                else if (strncmp(p, "RESET", 5) == 0)
                {
                    p += 5;
                    reset_grid();
                    UART_1_PutString("GRID RESET\r\n");
                }
                /* =========================
                SET STARTING HEADING
                Command format: HEADING <deg> (e.g., HEADING 90)
                ========================= */
                else if (strncmp(p, "HEADING", 7) == 0)
                {
                    p += 7;
                    while (*p == ' ') p++;
                    float angle = (float)atof(p);
                    
                    // Normalize angle to [0, 360)
                    while (angle < 0.0f) angle += 360.0f;
                    while (angle >= 360.0f) angle -= 360.0f;

                    heading_deg = angle;
                    planning_heading_deg = angle;

                    sprintf(string_1, "HEADING SET TO %.1f DEG\r\n", heading_deg);
                    UART_1_PutString(string_1);
                }

                /* =========================
                SET STARTING POSITION
                Command format: POS <x> <y> (e.g., POS 5 5)
                ========================= */
                else if (strncmp(p, "POS", 3) == 0)
                {
                    p += 3;
                    while (*p == ' ') p++;
                    
                    int gx = parse_x_coord(p); /* Accepts 'A'..'T' or 1..20 */
                    
                    while (*p != ' ' && *p != '\0') p++;
                    while (*p == ' ') p++;
                    int gy = atoi(p);          /* Y remains numeric 1..20 */

                    if (!in_bounds(gx, gy))
                    {
                        UART_1_PutString("POSITION OUT OF BOUNDS\r\n");
                    }
                    else
                    {
                        posX = gx;
                        posY = gy;
                        planning_posX = gx;
                        planning_posY = gy;
                        grid[posY - 1][posX - 1] = CELL_VISITED;
                        planning_grid[posY - 1][posX - 1] = CELL_VISITED;

                        sprintf(string_1, "POSITION SET TO (%d, %d)\r\n", posX, posY);
                        UART_1_PutString(string_1);
                    }
                }
                else if (strncmp(p, "PRINT", 5) == 0)
                {
                    p += 5;
                    print_grid_uart1();
                }
                else if (strncmp(p, "OBSTACLE", 8) == 0 ||
                strncmp(p, "SOURCE", 6) == 0 ||
                strncmp(p, "DEST", 4) == 0)
                {
                    char cmd_name[16];
                    sscanf(p, "%s", cmd_name);

                    p += strlen(cmd_name);
                    while (*p == ' ') p++;

                    int gx = parse_x_coord(p); /* Accepts 'A'..'T' or 1..20 */

                    while (*p != ' ' && *p != '\0') p++;
                    while (*p == ' ') p++;
                    int gy = atoi(p);

                    if (!in_bounds(gx, gy))
                    {
                        UART_1_PutString("COORDINATE OUT OF BOUNDS\r\n");
                    }
                    else
                    {
                        if (strcmp(cmd_name, "OBSTACLE") == 0)
                        {
                            grid[gy - 1][gx - 1] = CELL_OBSTACLE;
                            planning_grid[gy - 1][gx - 1] = CELL_OBSTACLE;
                        }
                        else if (strcmp(cmd_name, "SOURCE") == 0)
                        {
                            posX = gx;
                            posY = gy;
                            planning_posX = gx;
                            planning_posY = gy;
                            grid[gy - 1][gx - 1] = CELL_SOURCE;
                            planning_grid[gy - 1][gx - 1] = CELL_SOURCE;
                        }
                        else if (strcmp(cmd_name, "DEST") == 0)
                        {
                            grid[gy - 1][gx - 1] = CELL_DEST;
                            planning_grid[gy - 1][gx - 1] = CELL_DEST;
                        }
                        
                        sprintf(string_1, "%s SET AT (%d, %d)\r\n", cmd_name, gx, gy);
                        UART_1_PutString(string_1);
                    }
                }
                else if (strncmp(p, "VOWEL", 5) == 0)
                {
                    p += 5;
                    while (*p == ' ') p++;
                    
                    char v = *p;
                    p++;
                    while (*p == ' ') p++;

                    int gx = parse_x_coord(p); /* Accepts 'A'..'T' or 1..20 */

                    while (*p != ' ' && *p != '\0') p++;
                    while (*p == ' ') p++;
                    int gy = atoi(p);

                    uint8_t cell = CELL_EMPTY;
                    switch (tolower((unsigned char)v))
                    {
                        case 'a': cell = CELL_VOWEL_A; break;
                        case 'e': cell = CELL_VOWEL_E; break;
                        case 'i': cell = CELL_VOWEL_I; break;
                        case 'o': cell = CELL_VOWEL_O; break;
                        case 'u': cell = CELL_VOWEL_U; break;
                    }

                    if (!in_bounds(gx, gy) || cell == CELL_EMPTY)
                    {
                        UART_1_PutString("INVALID VOWEL OR COORDINATE\r\n");
                    }
                    else
                    {
                        grid[gy - 1][gx - 1] = cell;
                        planning_grid[gy - 1][gx - 1] = cell;

                        sprintf(string_1, "VOWEL %c SET AT (%d, %d)\r\n", v, gx, gy);
                        UART_1_PutString(string_1);
                    }
                }

                /* TEST_VOWEL Command (e.g. "T: Test_Vowel A" or "Test_Vowel e") */
                else if (strncmp(p, "TEST_VOWEL", 10) == 0)
                {
                    /* Move pointer past "Test_Vowel" */
                    p += 10;
                    while (*p == ' ') p++;

                    char v = *p;
                    char lower_v = tolower((unsigned char)v);

                    /* Validate if the argument is a valid vowel */
                    if (lower_v == 'a' || lower_v == 'e' || lower_v == 'i' || lower_v == 'o' || lower_v == 'u')
                    {
                        char vowel_str[2]; 
                        vowel_str[0] = toupper((unsigned char)v);
                        vowel_str[1] = '\0'; // Properly terminate the string

                        send_vowel_message(vowel_str);
                    }
                    else
                    {
                        UART_1_PutString("ERROR: INVALID VOWEL FOR TEST\r\n");
                    }
                }

                /* =========================
                   NORMAL COMMAND
                   ========================= */
                else
                {
                    char cmd_name[10];

                    int j = 0;

                    while (*p != ' ' && *p != '\0' && j < sizeof(cmd_name) - 1)
                    {
                        cmd_name[j++] = *p;
                        p++;
                    }

                    cmd_name[j] = '\0';

                    /* Skip spaces */
                    while (*p == ' ')
                    {
                        p++;
                    }

                    int target_val = 1;

                    if (*p != '\0' && *p != ' ')
                    {
                        target_val = expression_int(p);

                        while (*p != ' ' && *p != '\0')
                        {
                            p++;
                        }
                    }

                    if (strcmp(cmd_name, "fd") == 0)
                    {
                        enqueue_movement(cmd_queue, &cmd_count, 30, 'f', target_val);
                    }
                    else if (strcmp(cmd_name, "lt") == 0)
                    {
                        enqueue_movement(cmd_queue, &cmd_count, 30, 'l', target_val);
                    }
                    else if (strcmp(cmd_name, "bk") == 0)
                    {
                        enqueue_movement(cmd_queue, &cmd_count, 30, 'b', target_val);
                    }
                    else if (strcmp(cmd_name, "rt") == 0)
                    {
                        enqueue_movement(cmd_queue, &cmd_count, 30, 'r', target_val);
                    }
                }
            }
            if (!is_make_command && cmd_count > 0)
            {
                sprintf(string_1, "DEBUG: count=%d\r\n", cmd_count);
                UART_1_PutString(string_1);

                for (int j = 0; j < cmd_count; j++)
                {
                    sprintf(string_1,
                            "QUEUE[%d]: type=%c param=%d repeat=%d\r\n",
                            j,
                            cmd_queue[j].type,
                            cmd_queue[j].param,
                            cmd_queue[j].repeat_cnt);

                    UART_1_PutString(string_1);
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




/* ============================================================
 * ESP32-S3 CAMERA - UART2 POLLING
 *
 * This follows the same method as the working motor_camera ZIP:
 *   ESP32-S3 TX -> P15[5] -> UART2 RX
 *   UART2 RX is polled in the main loop.
 *   Detected vowels are processed by handle_camera_byte().
 *   handle_camera_byte() sends the Termite message using UART2 TX.
 *
 * ============================================================ */
void check_camera_uart(void)
{
    uint8 camera_rx;

    while (UART_2_GetRxBufferSize() > 0u)
    {
        camera_rx = UART_2_GetChar();

        switch (camera_rx)
        {
            case 'A':
            case 'E':
            case 'I':
            case 'O':
            case 'U':
            case 'a':
            case 'e':
            case 'i':
            case 'o':
            case 'u':
            case 'N':
            case 'n':
                handle_camera_byte(camera_rx);
                break;

            default:
                /* Ignore CR, LF, spaces, debug text, etc. */
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
    isr_1_StartEx(ISR_Handler_1);

    /*
     * Camera UART:
     * RX = P15[5] from ESP32-S3.
     * TX = P12[7] to onboard KitProg / COM port / Termite.
     */
    UART_2_PutString("UART2 READY\r\n");
    assign_variable("vowel", "NONE");
    reset_grid();
    UART_1_PutString("START...........");
    UART_1_PutString("\n");
    int counter1 = 0;
    int counter2 = 0;

    // initial setup
    PWM_1_Start();
    QuadDec_1_Start();
    PWM_2_Start();
    QuadDec_2_Start();
    
    //PWM_1_WriteCompare(230);
    //PWM_2_WriteCompare(230);
    
    int start_c1 = QuadDec_1_GetCounter();
    int start_c2 = QuadDec_2_GetCounter();
    int relative_c1 = 0;
    int relative_c2 = 0;
    
    //UART_1_PutString("AT+NAME=<aaaaaaaaaaaaa>");
    //UART_1_PutCRLF('\0');
    for(;;)
    {
        /*
         * Always service the ESP32-S3 camera first.
         * This is the same UART2 polling method used by the working ZIP.
         */
        check_camera_uart();

        /* Place your application code here. */
        if (request_reset_start)
        {
            //UART_1_PutString("reset");
            request_reset_start = 0;
            start_c1 = QuadDec_1_GetCounter();
            start_c2 = QuadDec_2_GetCounter();
        }
        counter1 = QuadDec_1_GetCounter(); 
        counter2 = QuadDec_2_GetCounter();
        
        relative_c1 = counter1 - start_c1;
        relative_c2 = counter2 - start_c2;
        
        //sprintf(string_1, "%d %d\r\n", counter1, counter2);
        //UART_1_PutString(string_1);

        CyDelay(50);

        if (waiting_for_uart)
        {
            if (camera_update_count <= camera_updates_consumed)
            {
                continue;
            }

            camera_updates_consumed++;
            waiting_for_uart = 0;

            if (while_context_depth > 0)
            {
                if (while_condition_true())
                {
                    while_cmd_idx = 0;
                    executing_while = 1;
                    request_reset_start = 1;
                }
                else
                {
                    WhileContext *parent = &while_context_stack[while_context_depth - 1];

                    memcpy(while_cmd_queue, parent->queue, sizeof(parent->queue));
                    while_cmd_count = parent->count;
                    while_cmd_idx = parent->index + 1;
                    strcpy(while_var_name, parent->variable);
                    strcpy(while_expected_value, parent->expected);
                    while_context_depth--;
                    executing_while = 1;
                    request_reset_start = 1;
                }
            }
            else if (while_condition_true())
            {
                while_cmd_idx = 0;
                executing_while = 1;
                request_reset_start = 1;
            }
            else
            {
                current_cmd_idx++;
                request_reset_start = 1;
            }

            continue;
        }

        /* =========================
           WHILE BODY FINISHED
           ========================= */

        if (executing_while && while_cmd_idx >= while_cmd_count)
        {
            // Stop motors
            Motor_1_IN_1_Write(0);
            Motor_1_IN_2_Write(0);
            Motor_2_IN_3_Write(0);
            Motor_2_IN_4_Write(0);

            executing_while = 0;
            waiting_for_uart = 1;
            uart_msg_received = 0;

            continue;
        }


        /* =========================
           NO MORE NORMAL COMMAND
           ========================= */

        if (current_cmd_idx >= cmd_count)
        {
            Motor_1_IN_1_Write(0);
            Motor_1_IN_2_Write(0);
            Motor_2_IN_3_Write(0);
            Motor_2_IN_4_Write(0);

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
            //UART_1_PutString("get command");
            current_cmd = &cmd_queue[current_cmd_idx];
        }

        
        switch (current_cmd->type)
        {
            case 'm': //make assignment
            {
                AssignmentCommand *assignment = &assignment_queue[current_cmd->param];
                assign_variable(assignment->name, assignment->value);

                if (executing_while)
                {
                    while_cmd_idx++;
                }
                else
                {
                    current_cmd_idx++;
                }
                request_reset_start = 1;
            }
            break;
            case 'w': //while
            {
                int condition;

                if (executing_while)
                {
                    WhileDefinition *definition = &while_definitions[current_cmd->param];
                    condition = condition_true(definition->variable,
                                               definition->expected);

                    if (condition && while_context_depth < 4)
                    {
                        WhileContext *saved = &while_context_stack[while_context_depth];

                        memcpy(saved->queue,
                               while_cmd_queue,
                               sizeof(saved->queue));
                        saved->count = while_cmd_count;
                        saved->index = while_cmd_idx;
                        strcpy(saved->variable, while_var_name);
                        strcpy(saved->expected, while_expected_value);

                        while_context_depth++;
                        strcpy(while_var_name, definition->variable);
                        strcpy(while_expected_value, definition->expected);
                        while_cmd_count = 0;
                        parse_while_queue_body(definition->body,
                                               while_cmd_queue,
                                               &while_cmd_count);
                        while_cmd_idx = 0;
                        executing_while = 1;
                        request_reset_start = 1;
                    }
                    else if (!condition || while_context_depth >= 4)
                    {
                        while_cmd_idx++;
                    }
                }
                else if (current_cmd->param >= 0)
                {
                    WhileDefinition *definition = &while_definitions[current_cmd->param];

                    if (condition_true(definition->variable, definition->expected))
                    {
                        strcpy(while_var_name, definition->variable);
                        strcpy(while_expected_value, definition->expected);
                        while_cmd_count = 0;
                        parse_while_queue_body(definition->body,
                                               while_cmd_queue,
                                               &while_cmd_count);
                        while_cmd_idx = 0;
                        executing_while = 1;
                        request_reset_start = 1;
                    }
                    else
                    {
                        current_cmd_idx++;
                    }
                }
                else if (while_condition_true())
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
            case 'i': //deferred ifelse
            {
                IfElseCommand *conditional = &ifelse_queue[current_cmd->param];
                int condition = compare_variable(conditional->variable,
                                                 conditional->expected);

                sprintf(string_1,
                        "IFELSE: %s == %s ? %d\r\n",
                        conditional->variable,
                        conditional->expected,
                        condition);
                UART_1_PutString(string_1);

                if (condition)
                {
                    UART_1_PutString("IF TRUE\r\n");
                    if (executing_while)
                    {
                        parse_branch_queue(conditional->true_body,
                                           while_cmd_queue,
                                           &while_cmd_count);
                    }
                    else
                    {
                        parse_branch(conditional->true_body);
                    }
                }
                else
                {
                    UART_1_PutString("IF FALSE\r\n");
                    if (executing_while)
                    {
                        parse_branch_queue(conditional->false_body,
                                           while_cmd_queue,
                                           &while_cmd_count);
                    }
                    else
                    {
                        parse_branch(conditional->false_body);
                    }
                }

                if (executing_while)
                {
                    while_cmd_idx++;
                }
                else
                {
                    current_cmd_idx++;
                }
                request_reset_start = 1;
            }
            break;
            case 'f'://forward
            {
                //UART_1_PutString("f");
                int target = current_cmd->param;
                
                int count_master = QuadDec_2_GetCounter();
                int count_slave = QuadDec_1_GetCounter();
                
                int error = relative_c1 - relative_c2;

                int pwm_master = MASTER_PWM;
                int pwm_slave = pwm_master + (error * KP);

                if (pwm_slave > 255)
                    pwm_slave = 255;

                if (pwm_slave < 0)
                    pwm_slave = 0;

                PWM_2_WriteCompare(pwm_master);
                PWM_1_WriteCompare(pwm_slave);
                
                
                sprintf(string_1,
                        "FD DEBUG: relative=%d target=%d\r\n",
                        relative_c2,
                        target);
                //UART_1_PutString(string_1);
                if (relative_c2 >= target)
                {
                    sprintf(string_1,
                        "2=%d 1=%d\r\n",
                        relative_c2, relative_c1);

                    UART_1_PutString(string_1);
                    
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(1);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(1);
                }
                else
                {
                    sprintf(string_1,
                        "M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                        relative_c2,
                        relative_c1,
                        error,
                        pwm_slave,MASTER_PWM);

                    UART_1_PutString(string_1);
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(0);
                    commit_completed_movement(current_cmd);
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
            }
            break;
            case 'r'://turn right
            {
                int target = current_cmd->param; 
                
                int count_master = QuadDec_2_GetCounter();
                int count_slave = QuadDec_1_GetCounter();

                /*
                 * Left turn:
                 * Motor 2 (Master) -> forward  (+)
                 * Motor 1 (Slave)  -> backward (-)
                 *
                 * Compare the magnitudes of the two movements.
                 */
                int error =  -relative_c2 - relative_c1;

                int pwm_master = MASTER_PWM;
                int pwm_slave = pwm_master + (error * KP);

                if (pwm_slave > 255)
                    pwm_slave = 255;

                if (pwm_slave < 0)
                    pwm_slave = 0;

                /* Motor 2 = Master */
                PWM_2_WriteCompare(pwm_master);

                /* Motor 1 = Slave */
                PWM_1_WriteCompare(pwm_slave);

                
                if (relative_c2 > target)
                {
                    Motor_1_IN_1_Write(1);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(1);
                }
                else
                {
                    sprintf(string_1,
                        "LT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                        relative_c2,
                        relative_c1,
                        error,
                        pwm_slave,
                        pwm_master);

                    UART_1_PutString(string_1);
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(0); 
                    commit_completed_movement(current_cmd);
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
                break;
            }
            case 'b'://backward
            {
                int target = current_cmd->param;
                int count_master = QuadDec_2_GetCounter();
                int count_slave = QuadDec_1_GetCounter();
                
                int error = relative_c2 - relative_c1;

                int pwm_master = MASTER_PWM;
                int pwm_slave = pwm_master + (error * KP);

                if (pwm_slave > 255)
                    pwm_slave = 255;

                if (pwm_slave < 0)
                    pwm_slave = 0;

                PWM_2_WriteCompare(pwm_master);
                PWM_1_WriteCompare(pwm_slave);
                
                if (relative_c2 <= target)
                {
                    sprintf(string_1,
                        "2=%d 1=%d\r\n",
                        relative_c2, relative_c1);

                    //UART_1_PutString(string_1);
                    Motor_1_IN_1_Write(1);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(1);
                    Motor_2_IN_4_Write(0);
                }
                else
                {
                    sprintf(string_1,
                        "M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                        relative_c2,
                        relative_c1,
                        error,
                        pwm_slave,MASTER_PWM);

                    UART_1_PutString(string_1);
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(0);
                    commit_completed_movement(current_cmd);
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
                break;
            }
            case 'l'://turn left
            {
                int target = current_cmd->param;
                int count_master = QuadDec_2_GetCounter();
                int count_slave = QuadDec_1_GetCounter();

                /*
                 * Right turn:
                 * Motor 2 (Master) -> backward (-)
                 * Motor 1 (Slave)  -> forward (+)
                 *
                 * Compare the magnitudes of the two movements.
                 */
                int error = (relative_c2 + relative_c1);

                int pwm_master = MASTER_PWM;
                int pwm_slave = pwm_master + (error * KP);

                if (pwm_slave > 255)
                    pwm_slave = 255;

                if (pwm_slave < 0)
                    pwm_slave = 0;

                /* Motor 2 = Master */
                PWM_2_WriteCompare(pwm_master);

                /* Motor 1 = Slave */
                PWM_1_WriteCompare(pwm_slave);


                if (relative_c2 < target)
                {
                    sprintf(string_1,
                        "2=%d 1=%d\r\n",
                        relative_c2, relative_c1);
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(1);
                    Motor_2_IN_3_Write(1);
                    Motor_2_IN_4_Write(0);
                }
                else
                {
                    sprintf(string_1,
                        "RT M=%d S=%d E=%d PWM_s=%d PWM_m=%d\r\n",
                        relative_c2,
                        relative_c1,
                        error,
                        pwm_slave,
                        pwm_master);

                    UART_1_PutString(string_1);
                    Motor_1_IN_1_Write(0);
                    Motor_1_IN_2_Write(0);
                    Motor_2_IN_3_Write(0);
                    Motor_2_IN_4_Write(0);
                    commit_completed_movement(current_cmd);
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
                break;
            }      
            default:
                Motor_1_IN_1_Write(0);
                Motor_1_IN_2_Write(0);
                Motor_2_IN_3_Write(0);
                Motor_2_IN_4_Write(0);
                break;                
        }
    }
}

/* [] END OF FILE 
case 'f':
{
    int target = -current_cmd.param; 
    
    int count_master = QuadDec_1_GetCounter();
    int count_slave = QuadDec_2_GetCounter();
   
    int error = count_master - count_slave;
    int Kp = 2; 
    int base_pwm = 200;
    int pwm_slave = base_pwm + (error * Kp);
    
    if (pwm_slave > 255) pwm_slave = 255;
    if (pwm_slave < 0) pwm_slave = 0;
    
    PWM_1_WriteCompare(base_pwm);
    PWM_2_WriteCompare(pwm_slave);
    
    if (relative_c2 >= target)
    {
        Motor_1_IN_1_Write(0); Motor_1_IN_2_Write(1);
        Motor_2_IN_3_Write(0); Motor_2_IN_4_Write(1);
    }
    else
    {
        Motor_1_IN_1_Write(0); Motor_1_IN_2_Write(0);
        Motor_2_IN_3_Write(0); Motor_2_IN_4_Write(0);
        current_cmd.type = '0';
    }
    break;
}*/
