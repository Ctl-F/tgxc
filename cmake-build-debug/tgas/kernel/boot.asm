#include "../tgstd/stddef.tdef"
$section meta
@export ENTRY
$section data


$section prog
ENTRY:

    jmp shell_init;
    jmp shell_start;
    hlt;

#include "shell.asm"
