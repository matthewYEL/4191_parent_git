"""Exercise the actual firmware IR stop logic with mocked hardware (MSVC)."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "DC_ Motor_test_1/Design01.cydsn/main.c").read_text()
vcvars = os.environ.get(
    "DRIVE_TEST_VCVARS",
    r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat",
)
header = r'''
#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
#define CY_ISR(name) void name(void)
#define CyGlobalIntEnable ((void)0)
static char input_byte;
static int pins[4], pwm[2], ir_left, ir_right;
static uint8 CyEnterCriticalSection(void) { return 0; }
static void CyExitCriticalSection(uint8 s) { (void)s; }
static void CyDelay(uint32 ms) { (void)ms; }
static void UART_1_Start(void) {}
static char UART_1_GetChar(void) { return input_byte; }
static void UART_1_PutString(const char *s) { (void)s; }
static void UART_1_PutChar(char c) { (void)c; }
static void UART_2_Start(void) {}
static uint8 UART_2_GetChar(void) { return 0; }
static uint8 UART_2_GetRxBufferSize(void) { return 0; }
static void UART_2_ClearRxBuffer(void) {}
static void UART_2_PutString(const char *s) { (void)s; }
static void UART_3_Start(void) {}
static void UART_3_PutChar(uint8 c) { (void)c; }
static void Motor_1_IN_1_Write(int v) { pins[0]=v; }
static void Motor_1_IN_2_Write(int v) { pins[1]=v; }
static void Motor_2_IN_3_Write(int v) { pins[2]=v; }
static void Motor_2_IN_4_Write(int v) { pins[3]=v; }
static void PWM_1_Start(void) {}
static void PWM_2_Start(void) {}
static void PWM_1_WriteCompare(int v) { pwm[0]=v; }
static void PWM_2_WriteCompare(int v) { pwm[1]=v; }
static void QuadDec_1_Start(void) {}
static void QuadDec_2_Start(void) {}
static int QuadDec_1_GetCounter(void) { return 0; }
static int QuadDec_2_GetCounter(void) { return 0; }
static void CySysTickStart(void) {}
static void CySysTickSetCallback(int n, void (*cb)(void)) { (void)n; (void)cb; }
static void isr_1_StartEx(void (*cb)(void)) { (void)cb; }
static uint8 LEFT_Read(void) { return (uint8)ir_left; }
static uint8 RIGHT_Read(void) { return (uint8)ir_right; }
'''
cases = r'''
#undef main
static void sample(uint32 ms, int left, int right) {
    camera_ms=ms; ir_left=left; ir_right=right; check_forward_ir();
}
static void start_step(void) {
    camera_state=CAMERA_MOVING;
    drive_forward(0,0);
    assert(forward_ir_active && !forward_ir_armed);
    assert(!forward_ir_left_done && !forward_ir_right_done);
    assert(pins[1] && pins[3] && pwm[0]>0 && pwm[1]>0);
}
static void send_line(const char *s) {
    while (*s) { input_byte=*s++; ISR_Handler_1(); }
    input_byte='\r'; ISR_Handler_1();
}
int main(void) {
    /* Ignore starting black and mixed readings; debounce departure only. */
    cmd_count=1; cmd_queue[0].type='f'; cmd_queue[0].param=-1;
    start_step();
    /* Encoders alone correct straight travel; IR has not armed yet. */
    drive_forward(-100,-100);
    assert(pwm[0]==MASTER_PWM && pwm[1]==MASTER_PWM);
    drive_forward(-200,-100); /* left wheel ahead: slow it down */
    assert(pwm[0]<pwm[1] && pwm[1]==MASTER_PWM && pins[1] && pins[3]);
    drive_forward(-100,-200); /* left wheel behind: speed it up */
    assert(pwm[0]>pwm[1] && pwm[1]==MASTER_PWM && pins[1] && pins[3]);
    assert(!forward_ir_left_done && !forward_ir_right_done);
    sample(0,0,0); sample(100,0,0);
    assert(!forward_ir_armed && cmd_count==1 && pins[1] && pins[3]);
    sample(101,1,0); sample(120,0,1); assert(!forward_ir_armed);
    sample(121,1,1); sample(125,1,1); assert(!forward_ir_armed);
    sample(126,0,0); /* brief white must not arm */
    sample(130,1,1); sample(135,1,1); assert(forward_ir_armed);
    /* Left arrives first: stop motor 1 on this very sample. */
    sample(140,0,1);
    assert(cmd_count==1 && forward_ir_left_done && !forward_ir_right_done);
    assert(!pins[0] && !pins[1] && pwm[0]==0 && pins[3] && pwm[1]>0);
    drive_forward(-200,-150); /* later PWM update must honour the stop */
    assert(!pins[1] && pwm[0]==0 && pins[3] && pwm[1]==MASTER_PWM);
    sample(150,1,1); /* even if left leaves black, it stays stopped */
    drive_forward(-200,-300);
    assert(!pins[1] && pwm[0]==0 && pins[3]);
    sample(151,1,0); /* right arrives while left reads white */
    assert(cmd_count==0 && current_cmd_idx==0 && !forward_ir_active);
    assert(!pins[0] && !pins[1] && !pins[2] && !pins[3]);
    assert(pwm[0]==0 && pwm[1]==0);
    assert(camera_state==CAMERA_SETTLING && request_reset_start);
    drive_forward(-200,-300); /* no restart during camera scan */
    assert(!forward_ir_active && !pins[1] && !pins[3]);
    sample(200,0,0); assert(cmd_count==0); /* only one advancement */

    /* fd 2 must ignore the line again on the second segment. */
    cmd_count=1; cmd_queue[0].param=-2;
    start_step(); sample(201,1,1); sample(206,1,1);
    sample(210,0,0); sample(215,0,0);
    assert(cmd_count==1 && movement_steps_done==1 && !forward_ir_active);
    start_step(); sample(220,0,0); sample(240,0,0);
    assert(cmd_count==1 && !forward_ir_armed);
    sample(241,1,1); sample(246,1,1); sample(250,0,0); sample(255,0,0);
    assert(cmd_count==0 && movement_steps_done==0);

    /* The user's fd 3: reset departure/arrival on all three grid steps. */
    camera_state=CAMERA_READY;
    send_line("fd 3");
    assert(cmd_count==1 && cmd_queue[0].type=='f' && cmd_queue[0].param==-3);
    for (int step=0; step<3; step++) {
        uint32 time=1000u+(uint32)step*100u;
        start_step(); sample(time,0,0); sample(time+10,0,0);
        assert(!forward_ir_armed && pins[1] && pins[3]);
        sample(time+11,1,1); sample(time+16,1,1);
        sample(time+17,step==1 ? 1 : 0,step==1 ? 0 : 1);
        assert(cmd_count==1 && movement_steps_done==step);
        drive_forward(-100,-100);
        assert(step==1 ? (!pins[3] && pins[1]) : (!pins[1] && pins[3]));
        sample(time+18,0,0);
        assert(!pins[1] && !pins[3] && pwm[0]==0 && pwm[1]==0);
        assert(camera_state==CAMERA_SETTLING && !forward_ir_active);
        assert(cmd_count==(step==2 ? 0 : 1));
        assert(movement_steps_done==(step==2 ? 0 : step+1));
    }

    /* Right arrives first: left runs at fixed PWM, without correction
     * against the right wheel which is now stationary. */
    cmd_count=1; cmd_queue[0].param=-1;
    start_step(); sample(260,1,1); sample(265,1,1);
    sample(266,1,0);
    assert(cmd_count==1 && !forward_ir_left_done && forward_ir_right_done);
    assert(pins[1] && pwm[0]>0 && !pins[3] && pwm[1]==0);
    drive_forward(-10000,-100);
    assert(pins[1] && pwm[0]==MASTER_PWM && !pins[3] && pwm[1]==0);
    sample(267,1,1); drive_forward(-11000,-100);
    assert(pwm[0]==MASTER_PWM && pwm[1]==0);
    sample(268,0,1); assert(cmd_count==0 && pwm[0]==0 && pwm[1]==0);

    /* Counter wrap during departure; simultaneous arrival stops immediately. */
    cmd_count=1; cmd_queue[0].param=-1;
    start_step(); sample(UINT32_MAX-2,1,1); sample(2,1,1);
    assert(forward_ir_armed);
    sample(3,0,0); assert(cmd_count==0 && pwm[0]==0 && pwm[1]==0);

    /* Emergency stop clears partial multi-grid progress and IR state. */
    cmd_count=1; cmd_queue[0].param=-2; movement_steps_done=1;
    start_step(); sample(10,1,1); sample(15,1,1); sample(16,0,1);
    assert(forward_ir_left_done && pins[3]); send_line("stop");
    assert(cmd_count==0 && !forward_ir_active && movement_steps_done==0);
    drive_forward(0,0); assert(!pins[1] && !pins[3] && !forward_ir_active);
    sample(20,0,0); sample(25,0,0); assert(cmd_count==0);
    send_line("fd 1"); assert(cmd_count==1 && cmd_queue[0].param==-1);
    start_step(); sample(30,0,0); sample(40,0,0); assert(cmd_count==1);
    sample(41,1,1); sample(46,1,1); sample(50,0,0); sample(55,0,0);
    assert(cmd_count==0);

    /* IR input cannot complete a backward command or a turn. */
    cmd_count=1; cmd_queue[0].type='b'; cmd_queue[0].param=1;
    sample(60,1,1); sample(65,1,1); sample(70,0,0); sample(75,0,0);
    assert(cmd_count==1);
    cmd_queue[0].type='l'; sample(80,0,0); assert(cmd_count==1);

    /* Completion uses the existing branch queue bookkeeping. */
    executing_if=1; if_cmd_count=1; if_cmd_idx=0;
    if_cmd_queue[0].type='f'; if_cmd_queue[0].param=-1;
    start_step(); sample(90,1,1); sample(95,1,1); sample(100,0,0); sample(105,0,0);
    assert(!executing_if && cmd_count==0);
    cmd_count=1; executing_while=1; while_cmd_count=1; while_cmd_idx=0;
    while_cmd_queue[0].type='f'; while_cmd_queue[0].param=-1;
    start_step(); sample(110,1,1); sample(115,1,1); sample(120,0,0); sample(125,0,0);
    assert(while_cmd_idx==1 && cmd_count==1 && !forward_ir_active);
    puts("PASS: encoder P correction, initial line, independent IR stops, latched outputs, fd 3, rollover, emergency stop, branch queues");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="forward_ir_test_") as directory:
    temp = Path(directory)
    (temp / "project.h").write_text(header)
    (temp / "LEFT.h").write_text("/* Mock API supplied by project.h. */\n")
    (temp / "RIGHT.h").write_text("/* Mock API supplied by project.h. */\n")
    (temp / "ir_test.c").write_text("#define main firmware_main\n" + source + cases)
    (temp / "build.cmd").write_text(
        f'@echo off\ncall "{vcvars}" >nul\n'
        'if errorlevel 1 exit /b 1\n'
        'cl /nologo /std:c11 /W3 /D_CRT_SECURE_NO_WARNINGS ir_test.c /Fe:ir_test.exe\n'
        'exit /b %errorlevel%\n'
    )
    subprocess.run(["cmd", "/c", str(temp / "build.cmd")], cwd=temp, check=True)
    subprocess.run([str(temp / "ir_test.exe")], cwd=temp, check=True)
