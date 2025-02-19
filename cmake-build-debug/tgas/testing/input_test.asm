#include "../tgstd/stddef.tdef"
#include "../tgstd/stdgfx.tdef"
#include "../tgstd/stdio.tdef"

$section meta

#define PLAYER_SPEED 0.1

$section data
WindowTitle: str "Input Test"

WindowParams: struct gWindowInitParams 0
WindowColor: i8 64, 125, 255

White: i8 255, 255, 255

HostInputBuffer: struct HostResultBuffer 0

SquarePos: f32 0:2;

SquareLoc: struct gRect 0

SquareCommandBuffer:
SqCmd_Clear:    i32 0, GFX_COMMAND_ID_CLEAR, 0, 0
SqCmd_FillRect: i32 0, GFX_COMMAND_ID_FILL_RECT, 0, 0,
SqCmd_Present:  i32 0, GFX_COMMAND_ID_PRESENT, 0, 0
SqCmd_Eof:      i32 0, GFX_COMMAND_ID_EOF, 0, 0
SquareCommandBufferEnd:


DebugOutput: str "x: %f, y: %f\n";

InputBuffer: struct ControllerResultBuffer 0;
KeyMap: struct KeyboardMapping 0;

$section prog
ENTRY:

    ; initialize graphics
    mov ra, WindowParams;
    mov rb, offsetof[gWindowInitParams:pWindowTitle];
    mov rc, WindowTitle;
    mov [ra]+rb, rc;

    gqr;
    mov ri, GFX_COMMAND_ID_WININIT;
    mov m0h, ri;
    mov m1l, ra;
    gqai;

    mov [ra]+rb, rc;

    mov ra, WindowColor;
    mov ri, GFX_COMMAND_ID_SETCLRC;
    mov m0h, ri;
    mov m1l, ra;
    gqai;

    mov ri, GFX_COMMAND_ID_CLEAR;
    mov m0h, ri;
    gqai;

    mov ri, GFX_COMMAND_ID_PRESENT;
    mov m0h, ri;
    gqai;

    mov ri, GFX_COMMAND_ID_EOF;
    mov m0h, ri;
    gqai;

    gqs;
    gqawait;


    ; populate SquareParams
    mov rb, SqCmd_FillRect;
    add rb, 8;
    mov [rb], SquareLoc;
    add rb, 4;
    mov [rb], White;

    ; Fill Square loc
    mov ra, SquareLoc;
    mov rb, offsetof[gRect:x];
    mov rc, 16;
    mov [ra]+rb, rc;

    mov rb, offsetof[gRect:width];
    mov [ra]+rb, rc;

    mov rb, offsetof[gRect:height];
    mov [ra]+rb, rc;

    mov rb, offsetof[gRect:y];
    mov rc, 240;
    mov [ra]+rb, rc;


    push i32 HostInputBuffer;
    push i32 INPUT_HMD_ENABLED;
    syscall INPUT_CONTROL_SET_HMD;

    jmp InitKeymap;

    push i32 KeyMap;
    push i32 InputBuffer;
    push i32 INPUT_MD_KEYMAP;
    syscall INPUT_CONTROL_SET_MD;

    xor rf, rf;

main_loop:
    syscall INPUT_CONTROL_POLL;


    mov ra, SquareCommandBuffer;
    mov rb, SquareCommandBufferEnd;
    gqawait;
    gqr;
    gqsi ra, rb;
    gqs;

    ; load ball position
    mov ra, SquarePos;
    mov fx0, [ra];
    add ra, 4;
    mov fy0, [ra];

    xor rx, rx;
    xor ry, ry;

    ; load input vector;
    mov ra, InputBuffer;
    mov rb, offsetof[ControllerResultBuffer:left];
    mov rxs, [ra]+rb;
    mov rb, offsetof[ControllerResultBuffer:right];
    mov rys, [ra]+rb;


    sub ry, rx;
    movc fx1, ry;

    xor ry, ry;
    xor rx, rx;

    mov rb, offsetof[ControllerResultBuffer:up]
    mov rxs, [ra]+rb;
    mov rb, offsetof[ControllerResultBuffer:down];
    mov rys, [ra]+rb;

    sub ry, rx;
    movc fy1, ry;

    vdot fx2, ve1, ve1;
    cmp fx2, 0.0;
    jz SkipMove;

    vnorm ve1;
    vadd ve0, ve1;
SkipMove:


;    inc rf;
;    mod rf, 10;
;    cmp rf, 0;
;    jnz SKIP_DPRINT;
;    push fy0;
;    push fx0;
;    mov ra, DebugOutput;
;    syscall DEBUG_SERIAL_PRINTF;
;SKIP_DPRINT:

    ; update visual square
    mov ra, SquareLoc;
    mov rb, offsetof[gRect:x];
    movc rc, fx0;
    mov [ra]+rb, rc;

    mov rb, offsetof[gRect:y];
    movc rc, fy0;
    mov [ra]+rb, rc;

    ; update real square position
    mov ra, SquarePos;
    mov [ra], fx0;
    add ra, 4;
    mov [ra], fy0;


    mov ra, HostInputBuffer;
    mov rb, offsetof[HostResultBuffer:quit_event];
    xor rc, rc;
    mov rcs, [ra]+rb;
    cmp rcs, 1;
    jnz main_loop;



    hlt;


InitKeymap:
    push jr;

    mov ra, KeyMap;
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

    pop jr;
    ret;
