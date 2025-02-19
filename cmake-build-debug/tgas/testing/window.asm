#include "../tgstd/stddef.tdef"
#include "../tgstd/stdgfx.tdef"

$section meta


$section data
WindowTitle: str "Hello Window"

WindowParams: struct gWindowInitParams 0
WindowColor: i8 64, 125, 255

HostInputBuffer: struct HostResultBuffer 0

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

    push i32 HostInputBuffer;
    push i32 INPUT_HMD_ENABLED;
    syscall INPUT_CONTROL_SET_HMD;

main_loop:
    syscall INPUT_CONTROL_POLL;

    mov ra, HostInputBuffer;
    mov rb, offsetof[HostResultBuffer:quit_event];
    xor rc, rc;
    mov rcs, [ra]+rb;
    cmp rcs, 1;
    jnz main_loop;



    hlt;
