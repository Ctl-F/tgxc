#include "../tgstd/stddef.tdef"
$section meta
@export shell_init
@export shell_start

$section data

;; TODO: add actual user sessions
user_name: str "root"
user_name_end:

user_pwrd: str "admin"
user_pwrd_end:

login_prompt: str "Login User: "
login_prompt_pwrd: str "Password: "

#define LOGIN_BUFFER_SIZE 256

login_buffer: i8 0:LOGIN_BUFFER_SIZE
login_buffer_end:

login_msg_unknown_usr: str "User not found\n"
login_msg_invalid_pwrd: str "Password is incorrect\n"
login_msg_authenticated: str "Welcome %s\n"

shell_dummy: str "<SHELL>\n";

$section prog
shell_init:
    push jr;

    jmp shell_login;
    pop ra;
    jz shell_init;

    pop jr;
    ret;

shell_start:
    push jr;

    mov ra, shell_dummy;
    syscall DEBUG_SERIAL_PRINTF;

    pop jr;
    ret;


shell_login:
    break 0x0001;
    ; allocate return value
    sub sp, 4;
    ; rf holds the address of the return value
    mov rf, sp;
    push jr;

    ; display the prompt
    mov ra, login_prompt;
    syscall DEBUG_SERIAL_PRINTF;

    ; read serial input
    mov ra, login_buffer;
    mov rb, LOGIN_BUFFER_SIZE;
    syscall DEBUG_SERIAL_FGETS;
    jmp shell_bsan;

    ; check the username
    mov rd, user_name_end;
    sub rd, user_name;
    dec rd;
    mov rc, user_name;
    mcmp rz, ra, rc, rd;
    jnz SH_INVALID_USERNAME;

    ; clear the buffer
    mov rd, login_buffer;
    mov re, login_buffer_end;
    mov rfs, 0
    mclr rd, re, rfs

    ; display password prompt
    mov ra, login_prompt_pwrd;
    syscall DEBUG_SERIAL_PRINTF;

    ; read password input
    mov ra, login_buffer;
    mov rb, LOGIN_BUFFER_SIZE
    syscall DEBUG_SERIAL_FGETS;
    jmp shell_bsan;

    ; check the password
    mov rd, user_pwrd_end;
    sub rd, user_pwrd;
    dec rd;
    mov rc, user_pwrd;
    mcmp ra, ra, rc, rd;
    jnz SH_INVALID_PSWD;

    ; welcome the user
    mov ra, user_name;
    push ra;
    mov ra, login_msg_authenticated;
    syscall DEBUG_SERIAL_PRINTF;

    ; return true
    mov [rf], 1;
    jmp SH_END;
SH_INVALID_USERNAME:
    syscall DEBUG_SERIAL_PRINT_REGS;

    mov ra, login_msg_unknown_usr;
    syscall DEBUG_SERIAL_PRINTF;
    mov [rf], 0;
    jmp SH_END;
SH_INVALID_PSWD:
    mov ra, login_msg_invalid_pwrd;
    syscall DEBUG_SERIAL_PRINTF;
    mov [rf], 0;
SH_END:
    pop jr;
    ret;

shell_bsan:
    push jr;
    push;

    mov ra, login_buffer;
SH_BSAN_LBEG:
    mov rcs, 10;
    mov rbs, [ra];
    jz SH_BSAN_LEND; ; if we already hit the null terminator then we break out of the loop
    cmp rcs, rbs;
    jz SH_BSAN_LEND;
    inc ra;
    jmp SH_BSAN_LBEG;
SH_BSAN_LEND:
    xor rcs, rcs;
    mov [ra], rcs;

    pop;
    pop jr;
    ret;
  //TODO Debugging tools