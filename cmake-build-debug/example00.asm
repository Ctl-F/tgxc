#include "tgstd/stddef.tdef"

$section meta
@export main

$section data
test_var_arr: i8 0:0x02 ;
test_var_i16: i16 0b1111111100001111 ;
test_var_i32: i32 0xDEADBEEF:2, 0;
test_var_i64: i64 42;
test_var_float: f32 3.1415;
test_var_str: str "\"Hello World\", she said...\n";
printf_test_fmt: str "%x\n";

$section prog
main:
    mov rb, test_var_i32;
    mov rb, [rb];
    push rb;
    mov ra, printf_test_fmt;
    syscall DEBUG_SERIAL_PRINTF;

    inc rb;
    push rb;
    mov ra, printf_test_fmt;
    syscall DEBUG_SERIAL_PRINTF;

    mov ra, test_var_str;
    syscall DEBUG_SERIAL_PRINTF;

    hlt ; test comment

