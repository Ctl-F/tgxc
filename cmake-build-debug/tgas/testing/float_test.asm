#include "../tgstd/stddef.tdef"

$section meta

$section data

FORMAT: str "%f %f\n";

$section prog
ENTRY:
    mov fx0, 2.0;
    mov fy0, 10.0;

    push fy0;
    push fx0;
    mov ra, FORMAT;
    syscall DEBUG_SERIAL_PRINTF;

    mov fx1, 18.0;
    mov fy1, 15.0;

    push fy1;
    push fx1;
    mov ra, FORMAT;
    syscall DEBUG_SERIAL_PRINTF;

    vadd ve0, ve1;

    push fy0;
    push fx0;
    mov ra, FORMAT;
    syscall DEBUG_SERIAL_PRINTF;

    hlt;

