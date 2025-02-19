#include "../tgstd/stddef.tdef"
$section  meta



$section data

msg_Greeting: str "Welcome! What is your name? ";
msg_GreetUser: str "Welcome %s\n";
msg_AskAge: str "How old are you? ";
msg_RepeatAge: str "Ok %s, You are %d years old\n";
msg_CanVote: str "You are old enough to vote\n";
msg_CantVote: str "You are not old enough to vote.\n";
msg_Is18: str "You are just old enough to vote.\n";

user_Age: i32 0
user_Name: i8 0:32
user_NameEof:

$section prog
ENTRY:
    mov ra, msg_Greeting;
    syscall DEBUG_SERIAL_PRINTF;

    mov rb, user_NameEof;
    mov ra, user_Name;
    sub rb, ra;
    syscall DEBUG_SERIAL_FGETS;

    ; remove trailing newline
    jmp RemoveNewLine;

    push i32 user_Name;
    mov ra, msg_GreetUser;
    syscall DEBUG_SERIAL_PRINTF;

    mov ra, msg_AskAge;
    syscall DEBUG_SERIAL_PRINTF;

    syscall DEBUG_SERIAL_SCANF_RA;
    mov rb, user_Age;
    mov [rb], ra;

    push ra;
    push i32 user_Name;
    mov ra, msg_RepeatAge;
    syscall DEBUG_SERIAL_PRINTF;

    break;
    mov ra, user_Age;
    mov ra, [ra];
    mov rb, 18;
    cmp ra, rb;
    jlt CANT_VOTE;
    jgt CAN_VOTE;
    jz IS_18;

    hlt;


CAN_VOTE:
    mov ra, msg_CanVote;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

CANT_VOTE:
    mov ra, msg_CantVote;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

IS_18:
    mov ra, msg_Is18;
    syscall DEBUG_SERIAL_PRINTF;
    ret;

RemoveNewLine:
    push jr;
    push;

    xor rc, rc;
    xor rd, rd;

    mov rds, 10;

    mov ra, user_Name;
nl_Loop:
    mov rcs, [ra];
    cmp rcs, rds;
    jz nl_LoopEnd;

    cmp rcs, 0;
    jz nl_Finished;

    inc ra;
    jmp nl_Loop;

nl_LoopEnd:
    xor rd, rd;
    mov [ra], rds;
nl_Finished:
    pop;
    pop jr;
    ret;

