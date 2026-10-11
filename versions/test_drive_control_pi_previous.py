"""Execute the actual main.c translation controller with mocked PSoC I/O.

Run: python tests/test_drive_control.py
Requires the installed Visual Studio C compiler; no hardware is driven.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "DC_ Motor_test_1/Design01.cydsn/main.c"
VCVARS = Path(os.environ.get(
    "DRIVE_TEST_VCVARS",
    r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat",
))

source = SOURCE.read_text()
settings = source[source.index("#define CAMERA_SETTLE_MS"):source.index("#define PULSES_PER_GRID")]
controller = source[source.index("static int encoder_relative("):source.index("void parse_while_body(")]
serial = source[source.index("#define UART_LOG_SIZE"):source.index("/* stop both drive motors */")]
scan = source[source.index("static void begin_camera_scan(void)\n{"):source.index("#if !MOTOR_ONLY_TEST\n/* Publish atomically")]
stub = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint32_t uint32;
typedef uint16_t uint16;
typedef int16_t int16;
static int out1, out2, pins[4], advanced;
static int uart_msg_received;
static void PWM_1_WriteCompare(int v) { out1=v; }
static void PWM_2_WriteCompare(int v) { out2=v; }
static void Motor_1_IN_1_Write(int v) { pins[0]=v; }
static void Motor_1_IN_2_Write(int v) { pins[1]=v; }
static void Motor_2_IN_3_Write(int v) { pins[2]=v; }
static void Motor_2_IN_4_Write(int v) { pins[3]=v; }
static void stop_motors(void) { for(int i=0;i<4;i++) pins[i]=0; }
static int fifo_room, tx_count;
static char transmitted[4096];
#define UART_1_TX_STS_FIFO_NOT_FULL 1u
static uint8 CyEnterCriticalSection(void) { return 0; }
static void CyExitCriticalSection(uint8 saved) { (void)saved; }
static uint8 UART_1_ReadTxStatus(void) { return fifo_room>0 ? 1 : 0; }
static void UART_1_WriteTxData(uint8 c) {
    assert(fifo_room>0); fifo_room--; transmitted[tx_count++]=(char)c;
}
'''
advance = r'''
static void advance_command_index(int delta) {
    (void)delta; advanced++; drive_active=0;
}
'''
cases = r'''
static void reset_test(void) {
    drive_active=0; drive_fault=0; camera_ms=0;
    advanced=out1=out2=0; stop_motors();
}
static void step(uint32 ms, int p1, int p2) {
    camera_ms=ms; update_drive(1, -p1, -p2, 1900);
}
int main(void) {
    int a,b;
    camera_state=CAMERA_MOVING;
    pins[0]=pins[1]=pins[2]=pins[3]=1;
    begin_camera_scan();
    assert(camera_state==CAMERA_READY && uart_msg_received==1);
    assert(!pins[0] && !pins[1] && !pins[2] && !pins[3]);
    uart_puts("abcdef");
    fifo_room=0; service_uart_tx(); assert(tx_count==0 && uart_log_tail==0);
    fifo_room=20; service_uart_tx(); assert(tx_count==4 && uart_log_tail==4);
    service_uart_tx(); assert(tx_count==6 && !memcmp(transmitted,"abcdef",6));
    uart_log_head=uart_log_tail=UART_LOG_SIZE-2;
    uart_puts("wrap"); fifo_room=4; service_uart_tx();
    assert(tx_count==10 && !memcmp(transmitted+6,"wrap",4));
    uart_log_head=UART_LOG_SIZE-2; uart_log_tail=0;
    uart_puts("no room");
    assert(uart_log_dropped==1 && uart_log_head==UART_LOG_SIZE-2);
    uart_log_head=uart_log_tail=0;
    { char long_text[400]; memset(long_text,'x',399); long_text[399]=0;
      uart_printf("%s",long_text); assert(uart_log_head==199); }
    uart_log_head=uart_log_tail=0;
    assert(encoder_relative(-32760,32760)==16);
    assert(encoder_relative(32760,-32760)==-16);
    assert(encoder_relative(-1900,0)==-1900);
    reset_test(); step(0,0,0);
    assert(out1==230 && out2==230 && pins[1] && pins[3]);
    step(20,0,70); /* dead motor 1: hold the leading wheel */
    assert(out1==230 && out2==0 && !pins[2] && !pins[3]);
    step(260,0,70);
    assert(drive_fault==8 && !out1 && !out2 && !advanced);

    reset_test(); step(0,0,0); step(40,20,20);
    assert(!drive_starting && out1==180 && out2==180);
    step(60,120,70);
    assert(out1<out2); /* leading motor is slowed */
    step(80,1500,1500);
    assert(out1>=115 && out1<180 && out2>=115 && out2<180);

    reset_test(); step(0,0,0); step(40,20,20);
    step(100,1910,1800); /* average has not reached target */
    assert(!out1 && out2>0 && !advanced);
    step(120,1910,1900);
    assert(!out1 && !out2 && !advanced);
    step(260,1910,1900); assert(!advanced);
    step(280,1910,1900); assert(advanced==1 && !drive_fault);

    reset_test(); step(0,0,0); step(40,20,20);
    step(100,1900,1900); step(240,1905,1905); /* coast restarts quiet timer */
    step(300,1905,1905); assert(!advanced);
    step(400,1905,1905); assert(advanced==1);

    reset_test(); step(0,0,0); step(40,20,20);
    step(100,1900,1900); step(120,1950,1900);
    assert(drive_fault==7 && !advanced && !out1 && !out2);

    reset_test(); step(0,0,0); step(40,20,20); step(1040,20,20);
    assert(drive_fault==3 && !advanced);
    reset_test(); step(0,0,0); step(20,-25,0);
    assert(drive_fault==1);
    reset_test(); step(0,0,0); step(40,20,20); step(60,400,20);
    assert(drive_fault==2);
    reset_test(); step(0,0,0); step(40,20,20); step(15000,100,100);
    assert(drive_fault==4);

    reset_test(); update_drive(0,0,0,1900);
    camera_ms=40; update_drive(0,20,20,1900);
    assert(pins[0] && !pins[1] && pins[2] && !pins[3]);
    camera_ms=60; update_drive(0,120,70,1900);
    assert(out1<out2 && !drive_fault);

    drive_integral=0; drive_done1=drive_done2=0;
    for(int i=0;i<10000;i++) compute_drive_pwm(100,0,1000,20,&a,&b);
    assert(drive_integral>0 && drive_integral<=DRIVE_INTEGRAL_LIMIT);
    drive_integral=0;
    for(int i=0;i<10000;i++) compute_drive_pwm(1000,0,1000,20,&a,&b);
    assert(drive_integral==0); /* saturation must not wind up */
    for(int e=-300;e<=300;e+=5) {
        for(int r=1;r<=1900;r+=17) {
            compute_drive_pwm(400+e,400,r,20,&a,&b);
            assert(a>=MIN_DRIVE_PWM && a<=255 && b>=MIN_DRIVE_PWM && b<=255);
        }
    }
    reset_test(); step(0,0,0); assert(drive_integral==0);
    reset_test(); step(UINT32_MAX-10,0,0); step(9,20,20);
    assert(drive_update_interval_ms==20 && !drive_fault);
    reset_test(); update_motor_test(1,0,0);
    assert(out1==230 && out2==0 && pins[1] && !pins[3]);
    camera_ms=499; update_motor_test(1,0,0); assert(out1==230 && !advanced);
    camera_ms=500; update_motor_test(1,0,0);
    assert(!out1 && !out2 && advanced==1 && !drive_fault);
    reset_test(); update_motor_test(2,0,0);
    assert(out2==230 && out1==0 && pins[3] && !pins[1]);
    camera_ms=500; update_motor_test(2,0,-100);
    assert(!out1 && !out2 && advanced==1);
    puts("PASS: nonblocking UART/full FIFO/overflow/wrap, startup, missing feedback, forward/backward, PI bounds, slowdown, per-wheel stop, settling and faults");
    return 0;
}
'''

if not VCVARS.exists():
    raise SystemExit("Set DRIVE_TEST_VCVARS to your Visual Studio vcvars64.bat path")
with tempfile.TemporaryDirectory(prefix="drive_test_") as temp:
    temp = Path(temp)
    (temp / "drive_test.c").write_text(stub + settings + serial + scan + advance + controller + cases)
    build = temp / "build.cmd"
    build.write_text(
        f'@echo off\ncall "{VCVARS}" >nul\n'
        'if errorlevel 1 exit /b 1\n'
        'cl /nologo /std:c11 /W3 drive_test.c /Fe:drive_test.exe\n'
        'exit /b %errorlevel%\n'
    )
    subprocess.run(["cmd", "/c", str(build)], cwd=temp, check=True)
    subprocess.run([str(temp / "drive_test.exe")], cwd=temp, check=True)
