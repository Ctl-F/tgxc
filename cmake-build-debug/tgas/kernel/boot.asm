#include "../tgstd/stddef.tdef"
#include "../tgstd/stdgfx.tdef"
#include "../tgstd/stdio.tdef"

#define ENABLE_ERR_CHECKING

#ifdef ENABLE_ERR_CHECKING
#define CHECK_SYSCALL_ERR() jmp SyscallHasError
#else
#define CHECK_SYSCALL_ERR()
#endif

$section meta

@export ENTRY


$section data


DisplayGameCommandBuffer:
    i64 0:10
DisplayGameCommandBufferEnd:

WhiteCol: struct gColorRGB 0

BallLoc: struct gRect 0
BallSpeed: i32 0, 0

Paddle0Loc: struct gRect 0
Paddle1Loc: struct gRect 0


SwiErrCode: i32 0
SwiErrStr: i8 0:1024

InputBuffer: struct ControllerResultBuffer 0
KeyboardMap: struct KeyboardMapping 0
HostBuffer: struct HostResultBuffer 0

WindowTitle: str "Pong"

WindowInitParams: struct gWindowInitParams 0
WindowClearColor: struct gColorRGB 0

Scores: i32 0:2

RandomSeed: i32 0

GFX_ErrorStackBeg: i8 0:512
GFX_ErrorStackEnd:
GFX_ErrorMessageFmt: str "GPU Error %d\n";
CPU_IntMessage: str "CPU Interrupt\n";

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

#define SCREEN_CENTER_X 420
#define SCREEN_CENTER_Y 240


$section prog
ENTRY:
    mov rx, HANDLE_INT_UNEXPECTED_ERR;
    mov tbl INT_RES_UNEXPECTED_ERROR, rx;
    mov rx, HANDLE_INT_RES_GQ_FULL;
    mov tbl INT_RES_GQ_FULL, rx;
    mov rx, HANDLE_INT_RES_GQ_OVERFLOW;
    mov tbl INT_RES_GQ_OVERFLOW, rx;

    trace 0x0000;
    jmp InitializeErrorChecking;
    trace 0x0010;
    jmp InitializeInput;
    trace 0x0020;
    jmp InitializeGraphics;
    trace 0x0030;
    jmp RecordGameCommandBuffer;
    trace 0x0040;
    jmp SeedRand;
    trace 0x0050;
    jmp InitBall;
    trace 0x0060;
    jmp InitPaddles;

    trace 0x0070;
MainLoop:
    syscall INPUT_CONTROL_POLL;

    trace 0x0075;
    ; main logic here
    jmp BallUpdate;

    ;; TODO: Update Paddles
    trace 0x0076;
    jmp GFXHasError;

    trace 0x007A;
    GFX_AWAIT_GPU();
    gqr;
    mov ra, DisplayGameCommandBuffer;
    mov rb, DisplayGameCommandBufferEnd;
    gqsi ra, rb;
    gqs;

    trace 0x007C;
    xor rc, rc;
    mov ra, HostBuffer;
    mov rb, offsetof[HostResultBuffer:quit_event];
    mov rcs, [ra]+rb;
    cmp rcs, 1;
    jnz MainLoop;
MainLoopEnd:
    trace 0x0080;
    jmp ShutdownGraphics;
    hlt;


RecordGameCommandBuffer:
    push jr;
    push;

    xor m0, m0;
    xor m1, m1;

    break;

    mov ra, DisplayGameCommandBuffer;

    ; clear screen
    mov rb, GFX_COMMAND_ID_CLEAR;
    mov m0h, rb;
    mov [ra], m0;
    add ra, GFX_INST_SIZE;

    ; draw ball
    mov rb, GFX_COMMAND_ID_FILL_RECT;
    mov m0h, rb;
    mov [ra], m0;
    add ra, 8;
    mov rb, BallLoc;
    mov m1l, rb;
    mov rb, WhiteCol;
    mov m1h, rb;
    mov [ra], m1;
    add ra, 8;

    ; draw paddle 0
    mov rb, GFX_COMMAND_ID_FILL_RECT;
    mov m0h, rb;
    mov [ra], m0;
    add ra, 8;
    mov rb, Paddle0Loc;
    mov m1l, rb;
    mov rb, WhiteCol;
    mov m1h, rb;
    mov [ra], m1;
    add ra, 8;

    ; draw paddle 1
    mov rb, GFX_COMMAND_ID_FILL_RECT;
    mov m0h, rb;
    mov [ra], m0;
    add ra, 8;
    mov rb, Paddle1Loc;
    mov m1l, rb;
    mov rb, WhiteCol;
    mov m1h, rb;
    mov [ra], m1;
    add ra, 8;

    ; eof
    mov rb, GFX_COMMAND_ID_EOF;
    mov m0h, rb;
    mov [ra], m0;

    pop;
    pop jr;
    ret;

InitBall:
    push jr;
    mov ra, BallLoc;
    mov ri, offsetof[gRect:width];
    mov rb, 16;
    mov [ra]+ri, rb;
    mov ri, offsetof[gRect:height];
    mov [ra]+ri, rb;

    mov ri, offsetof[gRect:x];
    mov rb, SCREEN_CENTER_X;
    mov [ra]+ri, rb;
    mov ri, offsetof[gRect:y];
    mov rb, SCREEN_CENTER_Y;
    mov [ra]+ri, rb;


    mov rb, BallSpeed;
    jmp Rand;
    mod ra, 3;
    mov [rb], ra;
    add rb, 4;
    jmp Rand;
    mod ra, 3;
    mov [rb], ra;

    pop jr;
    ret;

BallUpdate:
    push jr;
    push;

    mov ra, BallLoc;
    mov rb, offsetof[gRect:x];
    mov rx, [ra]+rb;
    mov rb, offsetof[gRect:y];
    mov ry, [ra]+rb;
    mov rc, offsetof[gRect:width];
    mov rz, [ra]+rb;

    mov rf, BallSpeed;
    mov re, [rf];
    add rf, 4;
    mov rf, [rf];

    ;; rx: ball x
    ;; ry: ball y
    ;; rz: ball size
    ;; re: ball x speed
    ;; rf: ball y speed

    ; add ball velocity to position
    add rx, re;
    add ry, rf;


    ; boundry world check (top and bottom)
    cmp ry, 0;
    jgt SKIP_TOP_HIT;
    ; top was hit

    sub ry, rf;
    neg rf;

    jmp SKIP_BOTTOM_HIT;
SKIP_TOP_HIT:
    mov rd, SCREEN_HEIGHT;
    sub rd, rz;
    cmp ry, rd;
    jlt SKIP_BOTTOM_HIT;
    ;  bottom was hit

    add ry, rf;
    neg rf;
SKIP_BOTTOM_HIT:
    ; check out of bounds (left)
    cmp rx, 0;
    jgt SKIP_LEFT_HIT;
    ;; left out of bounds

    ; add one to player 1 score
    mov rd, Scores;
    add rd, 4;
    mov rc, [rd];
    inc rc;
    mov [rd], rc;

    jmp InitBall;
    jmp RETURN;
SKIP_LEFT_HIT:
    mov rd, SCREEN_WIDTH;
    sub rd, rz;
    cmp rx, rd;
    jlt SKIP_RIGHT_HIT;
    ; right out of bounds

    ; add 1 to player 0 score
    mov rd, Scores;
    mov rc, [rd];
    inc rc;
    mov [rd], rc;

    jmp InitBall;
    jmp RETURN;
SKIP_RIGHT_HIT:
    ; check paddles
    ; TODO

    ; record ball new position
    mov ra, BallLoc;
    mov rb, offsetof[gRect:x];
    mov [ra]+rb, rx;

    mov rb, offsetof[gRect:y];
    mov [ra]+rb, ry;

    mov ra, BallSpeed;
    mov [ra], re;
    add ra, 4;
    mov [ra], rf;

RETURN:
    pop;
    pop jr;
    ret;

InitPaddles:
    push jr;
    mov ra, Paddle0Loc;

    mov ri, offsetof[gRect:width];
    mov rb, 16;
    mov [ra]+ri, rb;
    mov rb, 48;
    mov ri, offsetof[gRect:height];
    mov [ra]+ri, rb;

    mov ri, offsetof[gRect:x];
    mov rb, 40;
    mov [ra]+ri, rb;
    mov ri, offsetof[gRect:y];
    mov rb, SCREEN_CENTER_Y;
    mov [ra]+ri, rb;

    mov ra, Paddle1Loc;

    mov ri, offsetof[gRect:width];
    mov rb, 16;
    mov [ra]+ri, rb;
    mov rb, 48;
    mov ri, offsetof[gRect:height];
    mov [ra]+ri, rb;

    mov ri, offsetof[gRect:x];
    mov rb, 600;
    mov [ra]+ri, rb;
    mov ri, offsetof[gRect:y];
    mov rb, SCREEN_CENTER_Y;
    mov [ra]+ri, rb;

    pop jr;
    ret;


InitializeErrorChecking:
    push jr;
    push ra;

    trace 0x0001;

    push i32 SwiErrCode;
    syscall SYSCALL_ENABLE_ERRORS;

    trace 0x0002;

    mov ra, GFX_COMMAND_ID_ENABLE_ERRORS;
    mov m0h, ra;
    mov ra, GFX_ErrorStackBeg;
    mov m1l, ra;
    mov ra, GFX_ErrorStackEnd;
    mov m1h, rb;
    gqai;

    trace 0x0003;

    mov ra, GFX_COMMAND_ID_EOF;
    mov m0h, ra;
    xor m1, m1;
    gqai;
    gqs;

    trace 0x0004;

    GFX_AWAIT_GPU();

    trace 0x0005;
    pop ra;
    pop jr;
    ret;

GFXHasError:
    push jr;
    trace 0xDEAD;
    mov ri, GFX_ErrorStackBeg;

GFXErrorLoopBeg:
    trace 0xBEEF;
    xor rb, rb;
    mov rbs, [ri];
    cmp rbs, 0;
    jgt GFXErrorLoopDisplay;
GFXErrorLoopEnd:
    trace 0xF00B;
    ; TODO: Double Buffer the command buffer so we dont need to synchronize on EVERY write.
    GFX_AWAIT_GPU();

    mov ra, GFX_COMMAND_ID_CLEAR_ERRORS
    mov m0h, ra;
    xor m1, m1;
    gqai;

    mov ra, GFX_COMMAND_ID_EOF;
    mov m0h, ra;
    gqai;
    gqs; no need to synchronize here, we can just move on.

    pop jr;
    ret;
GFXErrorLoopDisplay:
    trace 0xB00B;
    push rb;
    mov ra, GFX_ErrorMessageFmt;
    syscall DEBUG_SERIAL_PRINTF;
    inc ri;
    jmp GFXErrorLoopBeg;



SyscallHasError:
    push jr;
    push ;
    mov ra, SwiErrCode;
    mov ra, [ra];
    cmp ra, 0;
    jz SyscallHE_End;
    mov ra, SwiErrStr;
    syscall DEBUG_SERIAL_PRINTF;
SyscallHE_End:
    pop ;
    pop jr;
    ret;



InitializeInput:
    push jr;
    push;

    mov ra, KeyboardMap;
    mov rb, offsetof[KeyboardMapping:key_for_left];
    mov rc, PKEY_A;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_right];
    mov rc, PKEY_D;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_up];
    mov rc, PKEY_W;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_down];
    mov rc, PKEY_S;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_start];
    mov rc, PKEY_RETURN;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_select];
    mov rc, PKEY_ESCAPE;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_a];
    mov rc, PKEY_LEFT;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_b];
    mov rc, PKEY_DOWN;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_x];
    mov rc, PKEY_UP;
    mov [ra]+rb, rc;

    mov rb, offsetof[KeyboardMapping:key_for_y];
    mov rc, PKEY_RIGHT;
    mov [ra]+rb, rc;


    push i32 KeyboardMap;
    push i32 InputBuffer;
    push i32 INPUT_MD_KEYMAP;
    syscall INPUT_CONTROL_SET_MD;

    CHECK_SYSCALL_ERR();

    push i32 HostBuffer;
    push i32 INPUT_HMD_ENABLED;
    syscall INPUT_CONTROL_SET_HMD;
    CHECK_SYSCALL_ERR();

    pop;
    pop jr;
    ret;


; WindowInitParams: struct gWindowInitParams 0
; WindowClearColor: struct gcolorRGB 0
InitializeGraphics:
    push jr;
    push;

    xor rb, rb;
    mov ra, WhiteCol;
    mov rbs, 255;
    mov [ra], rbs;
    inc ra;
    mov [ra], rbs;
    inc ra;
    mov [ra], rbs;

    ; initialize window para ms
    mov ra, WindowTitle;
    mov rb, WindowInitParams;
    mov [rb], ra;

    xor ra, ra;
    mov ras, 50;
    mov rb, WindowClearColor;
    mov ri, offsetof[gColorRGB:g];
    mov [rb]+ri, ras;
    inc ri;
    mov ras, 75;
    mov [rb]+ri, ras;

    GFX_RECORD_INIT_WINDOW(WindowInitParams);
    GFX_RECORD_SET_CLEAR_COLOR(WindowClearColor);
    GFX_RECORD_CLEAR();
    GFX_RECORD_PRESENT();
    GFX_RECORD_EOF();
    gqs;

    pop;
    pop jr;
    ret;


ShutdownGraphics:
    push jr;
    push;
    GFX_AWAIT_GPU();
    gqr;
    GFX_RECORD_FREE_WINDOW();
    GFX_RECORD_EOF();
    gqs;
    GFX_AWAIT_GPU();
    pop;
    pop jr;
    ret;

SeedRand:
    push;
    syscall GET_TIME_MS;
    mov rb, RandomSeed;
    mov [rb], ra;
    pop;
    ret;

Rand: ; ra[result] rand()
    push jr;
    push rb;
    push rc;

    mov ra, RandomSeed;

    mov rb, [ra];
    mov rc, rb;
    rbl rc, 13;
    xor rb, rc;

    mov rc, rb;
    rbr rc, 17;
    xor rb, rc;

    mov rc, rb;
    rbl rc, 5;
    xor rb, rc;

    mov [ra], rb;
    mov ra, rb;

    pop rc;
    pop rb;
    pop jr;
    ret;

HANDLE_INT_RES_GQ_OVERFLOW:
HANDLE_INT_RES_GQ_FULL:
HANDLE_INT_UNEXPECTED_ERR:
    push jr;
    push;

    mov ra, CPU_IntMessage;
    syscall DEBUG_SERIAL_PRINTF;

    pop;
    pop jr;
    ret;
