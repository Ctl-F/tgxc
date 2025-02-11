#include "../tgstd/stddef.tdef"
#include "../tgstd/stdgfx.tdef"
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

window_title: str "Hello Window";


WindowInitParams: struct gWindowInitParams 0
WindowClearColor: struct gColorRGB 0

quit_event_triggered: i8 0

debug_i32: str "%d\n"

$section prog
init_graphics_test:
    push jr;


    gqr;

    ; initialize system
    mov ra, window_title
    mov rb, WindowInitParams;
    mov rc, offsetof[gWindowInitParams:pWindowTitle]
    mov [rb]+rc, ra

    ; initialize color
    xor ra, ra;
    mov ras, 100
    mov rb, WindowClearColor;
    mov rc, offsetof[gColorRGB:g];
    mov [rb]+rc, ras;
    add ras, 50
    mov rc, offsetof[gColorRGB:b];
    mov [rb]+rc, ras;

    GFX_RECORD_INIT_WINDOW(WindowInitParams)
    GFX_RECORD_SET_CLEAR_COLOR(WindowClearColor)
    GFX_RECORD_CLEAR()
    GFX_RECORD_EOF()

    gqs;
    gqr;


    GFX_AWAIT_GPU()

    GFX_RECORD_SET_CLEAR_COLOR(WindowClearColor)
    GFX_RECORD_CLEAR()
    GFX_RECORD_PRESENT()

    mov ra, 0x00000007;
    mov m0h, ra;
    mov ra, quit_event_triggered;
    mov m1l, ra;
    gqai;

    GFX_RECORD_EOF()
    xor rb, rb;


    ; dummy eventloop
event_loop_beg:
    gqr;
    gqs;
    GFX_AWAIT_GPU()

    xor rb, rb
    mov ra, WindowClearColor
    mov rbs, [ra]
    inc rbs
    mov [ra], rbs
    inc ra
    mov rbs, [ra]
    inc rbs
    mov [ra], rbs
    inc ra
    mov rbs, [ra]
    inc rbs
    mov [ra], rbs


    mov ra, quit_event_triggered;
    mov rbs, [ra];
    cmp rbs, 0;
    jz event_loop_beg;
event_loop_end:

    gqr;
    GFX_RECORD_FREE_WINDOW()
    GFX_RECORD_EOF()


    gqs;
    GFX_AWAIT_GPU()

    pop jr;
    ret;


;sync_loop:
;    push jr;
;sync_loop_beg:
;    gqps;
;    jnz sync_loop_beg;
;    pop jr;
;    ret;

shell_init:
    push jr;

    jmp init_graphics_test;
    pop jr;
    ret;

;; disabled
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

    mov ra, login_buffer;
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
    mov ra, login_buffer;
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
    break;
    mov ra, login_msg_unknown_usr;
    syscall DEBUG_SERIAL_PRINTF;
    mov [rf], 0;
    jmp SH_END;
SH_INVALID_PSWD:
    break;
    mov ra, login_msg_invalid_pwrd;
    syscall DEBUG_SERIAL_PRINTF;
    mov [rf], 0;
SH_END:
    pop jr;
    ret;

shell_bsan:
    push jr;
    push;

    xor rb, rb;
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
