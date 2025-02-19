#include "../tgstd/stddef.tdef"
#include "../tgstd/stdgfx.tdef"

$section meta


$section data
WindowTitle: str "Window Square"

WindowParams: struct gWindowInitParams 0
WindowColor: i8 64, 125, 255

White: i8 255, 255, 255

HostInputBuffer: struct HostResultBuffer 0

SquareLoc: struct gRect 0

SquareCommandBuffer:
SqCmd_Clear:    i32 0, GFX_COMMAND_ID_CLEAR, 0, 0
SqCmd_FillRect: i32 0, GFX_COMMAND_ID_FILL_RECT, 0, 0,
SqCmd_Present:  i32 0, GFX_COMMAND_ID_PRESENT, 0, 0
SqCmd_Eof:      i32 0, GFX_COMMAND_ID_EOF, 0, 0
SquareCommandBufferEnd:

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

    break;

    push i32 HostInputBuffer;
    push i32 INPUT_HMD_ENABLED;
    syscall INPUT_CONTROL_SET_HMD;

main_loop:
    syscall INPUT_CONTROL_POLL;


    mov ra, SquareCommandBuffer;
    mov rb, SquareCommandBufferEnd;
    gqawait;
    gqr;
    gqsi ra, rb;
    gqs;

    ; move ball

    mov ra, SquareLoc;
    mov rb, offsetof[gRect:x];
    mov rc, [ra]+rb;
    inc rc;

    cmp rc, 640;
    jle SKIP_WARP;

    sub rc, 640;

SKIP_WARP:
    mov [ra]+rb, rc;




    mov ra, HostInputBuffer;
    mov rb, offsetof[HostResultBuffer:quit_event];
    xor rc, rc;
    mov rcs, [ra]+rb;
    cmp rcs, 1;
    jnz main_loop;



    hlt;
