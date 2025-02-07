#include "../tgstd/stddef.tdef"
$section meta

$section data

results: str "Total Time %d for %d operations\n"

$section prog
main:
    mov rb, 100000000 ; target
    mov rc, 0;

    syscall GET_TIME_MS;
    mov ri, ra;

BEG_LOOP:
    cmp rb, rc;
    jz END_LOOP;

    add rz, rx;
    inc rc;

    jmp BEG_LOOP;
END_LOOP:
    syscall GET_TIME_MS;
    sub ra, ri;

    push rb;
    push ra;
    mov ra, results;
    syscall DEBUG_SERIAL_PRINTF;
    hlt;