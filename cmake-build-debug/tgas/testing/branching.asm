#include "../tgstd/stddef.tdef"

$section  meta



$section data

msg_less_than: str "Less Than\n"
msg_greater_than: str "Greater Than\n";
msg_zero: str "Zero\n";
msg_nz: str "Non Zero\n";

$section prog
ENTRY:
    mov ra, 100;
    mov rb, 1000;

    cmp ra, rb;
    jlt LESS_THAN;
    jgt GREATER_THAN;
    jz ZERO;
    jnz NOT_ZERO;
    jle LESS_THAN;
    jge GREATER_THAN;

    hlt;

LESS_THAN:
    mov ra, msg_less_than;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

GREATER_THAN:
    mov ra, msg_greater_than;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

ZERO:
    mov ra, msg_zero;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

NOT_ZERO:
    mov ra, msg_nz;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

