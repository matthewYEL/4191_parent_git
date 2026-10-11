"""Compile the actual motor-only source with mocked hardware; requires MSVC."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "DC_ Motor_test_1/Design01.cydsn/main.c").read_text()
assert "UART_2" not in source and "camera" not in source.lower()
vcvars = os.environ.get("DRIVE_TEST_VCVARS", r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat")
header = r'''
#include <stdint.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef uint16_t uint16;
typedef int16_t int16;
#define CY_ISR(name) void name(void)
#define CyGlobalIntEnable ((void)0)
static char input_byte;
static int pins[4];
static int pwm[2], fifo_room, tx_count;
#define UART_1_TX_STS_FIFO_FULL 4u
static uint8 CyEnterCriticalSection(void) { return 0; }
static void CyExitCriticalSection(uint8 s) { (void)s; }
static uint8 UART_1_ReadTxStatus(void) { return fifo_room ? 0 : 4; }
static void UART_1_WriteTxData(uint8 c) { (void)c; assert(fifo_room>0); fifo_room--; tx_count++; }
static void UART_1_Start(void) {}
static char UART_1_GetChar(void) { return input_byte; }
static void UART_1_PutString(const char *s) { (void)s; }
static void UART_1_PutChar(char c) { (void)c; }
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
'''
cases = r'''
#undef main
#include <assert.h>
static void send_line(const char *s) {
    while (*s) { input_byte=*s++; ISR_Handler_1(); }
    input_byte='\r'; ISR_Handler_1();
}
int main(void) {
    finish_motor_segment();
    assert(!pins[0] && !pins[1] && !pins[2] && !pins[3]);
    send_line("fd1");
    assert(cmd_count==1 && cmd_queue[0].type=='f' && cmd_queue[0].param==-1);
    send_line("bk1"); /* busy command cannot overwrite fd1 */
    assert(cmd_count==1 && cmd_queue[0].type=='f');
    advance_command_index(0); assert(cmd_count==0);
    send_line("fd 3");
    advance_command_index(0); assert(cmd_count==1 && movement_steps_done==1);
    advance_command_index(0); assert(cmd_count==1 && movement_steps_done==2);
    advance_command_index(0); assert(cmd_count==0 && movement_steps_done==0);
    send_line("bk1"); assert(cmd_queue[0].type=='b' && cmd_queue[0].param==1);
    advance_command_index(0);
    send_line("repeat 2 [fd1 bk1]");
    assert(cmd_count==4 && cmd_queue[0].type=='f' && cmd_queue[3].type=='b');
    for(int i=0;i<4;i++) advance_command_index(0);
    assert(cmd_count==0);
    send_line("lt 90 rt 45");
    assert(cmd_count==2 && cmd_queue[0].param==-2400 && cmd_queue[1].param==1300);
    advance_command_index(-90); assert(current_heading_deg==270);
    advance_command_index(45); assert(current_heading_deg==315 && cmd_count==0);
    assert(heading_is_diagonal(current_heading_deg));
    send_line("fd -1"); assert(cmd_count==0);
    send_line("lt -90"); assert(cmd_count==0);
    current_heading_deg=0;
    send_line("fd1"); motor_ms=0; update_movement(&cmd_queue[0],0,0);
    assert(pwm[0]==115 && pwm[1]==115 && pins[1] && pins[3]);
    motor_ms=400; update_movement(&cmd_queue[0],-100,-50);
    assert(pwm[0]<180 && pwm[1]>180 && pwm[0]<pwm[1]);
    motor_ms=600; update_movement(&cmd_queue[0],-1700,-1700);
    assert(pwm[0]<180 && pwm[1]<180);
    motor_ms=800; update_movement(&cmd_queue[0],-1900,-1800);
    assert(pwm[0]==0 && pwm[1]>0 && cmd_count==1);
    motor_ms=900; update_movement(&cmd_queue[0],-1905,-1900);
    assert(!pwm[0] && !pwm[1] && cmd_count==1);
    motor_ms=1040; update_movement(&cmd_queue[0],-1910,-1900);
    motor_ms=1060; update_movement(&cmd_queue[0],-1910,-1900);
    assert(cmd_count==1); /* coast restarted the settling timer */
    motor_ms=1200; update_movement(&cmd_queue[0],-1910,-1900);
    assert(cmd_count==0 && !motor_fault);
    send_line("bk1"); motor_ms=1300; update_movement(&cmd_queue[0],0,0);
    assert(pins[0] && !pins[1] && pins[2] && !pins[3]);
    motor_ms=1700; update_movement(&cmd_queue[0],100,50);
    assert(pwm[0]<pwm[1]);
    advance_command_index(0);
    send_line("lt 90"); motor_ms=1800; update_movement(&cmd_queue[0],0,0);
    assert(pins[0] && !pins[1] && !pins[2] && pins[3]);
    motor_ms=2200; update_movement(&cmd_queue[0],100,-50);
    assert(pwm[0]<pwm[1]);
    advance_command_index(-90);
    send_line("rt 90"); motor_ms=2300; update_movement(&cmd_queue[0],0,0);
    assert(!pins[0] && pins[1] && pins[2] && !pins[3]);
    motor_ms=2700; update_movement(&cmd_queue[0],-100,50);
    assert(pwm[0]<pwm[1]);
    advance_command_index(90);
    {
        MotorControl s;
        motor_control_reset(&s,0);
        motor_control_step(&s,20,-21,0,1900,180); assert(s.fault==1 && !s.pwm[0] && !s.pwm[1]);
        motor_control_reset(&s,0);
        motor_control_step(&s,400,0,301,1900,180); assert(s.fault==2);
        motor_control_reset(&s,0);
        motor_control_step(&s,1000,0,0,1900,180); assert(s.fault==3);
        motor_control_reset(&s,0);
        motor_control_step(&s,15000,100,100,1900,180); assert(s.fault==4);
        motor_control_reset(&s,0);
        motor_control_step(&s,400,1941,1900,1900,180); assert(s.fault==5);
        motor_control_reset(&s,UINT32_MAX-10);
        motor_control_step(&s,9,0,0,1900,180); assert(!s.fault && s.pwm[0]==119);
        for(int e=-250;e<=250;e+=10) {
            motor_control_reset(&s,0);
            motor_control_step(&s,500,300+e,300,1900,180);
            assert(!s.fault && s.pwm[0]>=115 && s.pwm[0]<=230 && s.pwm[1]>=115 && s.pwm[1]<=230);
        }
    }
    uart_head=uart_tail=0; uart_puts("abcdef");
    fifo_room=0; service_uart_tx(); assert(tx_count==0);
    fifo_room=20; service_uart_tx(); assert(tx_count==4);
    service_uart_tx(); assert(tx_count==6);
    motor_ms=0; motor_tick(); assert(motor_ms==1);
    puts("PASS: commands, forward/backward/turn polarity, dual correction, ramp, slowdown, independent stops, settling, faults, rollover, nonblocking UART");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="motor_only_test_") as directory:
    temp = Path(directory)
    (temp / "project.h").write_text('#include <assert.h>\n' + header)
    (temp / "motor_control.h").write_text((ROOT / "DC_ Motor_test_1/Design01.cydsn/motor_control.h").read_text())
    (temp / "motor_test.c").write_text("#define main firmware_main\n" + source + cases)
    (temp / "build.cmd").write_text(
        f'@echo off\ncall "{vcvars}" >nul\n'
        'if errorlevel 1 exit /b 1\n'
        'cl /nologo /std:c11 /W3 /D_CRT_SECURE_NO_WARNINGS motor_test.c /Fe:motor_test.exe\n'
        'exit /b %errorlevel%\n')
    subprocess.run(["cmd", "/c", str(temp / "build.cmd")], cwd=temp, check=True)
    subprocess.run([str(temp / "motor_test.exe")], cwd=temp, check=True)
