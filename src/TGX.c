//
// Created by ctlf on 1/8/25.
//
#include "TGX.h"
#include <math.h>
#include <immintrin.h>

#define CPU_TICKS(result) asm volatile ("rdtsc" : "=A"(result))


int alloc_principle_memory(PrincipleMemory* mem){
    mem->memory_begin = malloc(PRINCIPLE_MEMORY_SIZE);
    if (mem->memory_begin == NULL) {
        return TGX_ERROR;
    }

    mem->memory_end = mem->memory_begin + PRINCIPLE_MEMORY_SIZE;
    mem->rom_begin = mem->memory_begin;
    mem->rom_end = mem->rom_begin + ROM_SIZE;
    mem->program_cache_begin = mem->rom_end;
    mem->program_cache_end = mem->program_cache_begin + PROGRAM_CACHE_SIZE;
    mem->graphics_cache_begin = mem->program_cache_end;
    mem->graphics_cache_end = mem->graphics_cache_begin + GRAPHICS_CACHE_SIZE;
    mem->graphics_queue_begin = mem->graphics_cache_end;
    mem->graphics_queue_end = mem->graphics_queue_begin + GRAPHICS_QUEUE_SIZE;
    mem->shared_ram_begin = mem->graphics_queue_end;
    mem->shared_ram_end = mem->shared_ram_begin + SHARED_RAM_SIZE;
    mem->instruction_cache_begin = mem->shared_ram_end;
    mem->instruction_cache_end = mem->instruction_cache_begin + INSTRUCTION_CACHE_SIZE;

    if (mem->instruction_cache_end != mem->memory_end) {
        free(mem->memory_begin);
        *mem = (PrincipleMemory){0};
        fprintf(stderr, "Misalignment with memory pointers, re-evaluate your code.");
        return TGX_ERROR;
    }

    return TGX_SUCCESS;
}



void free_principle_memory(PrincipleMemory* mem){
    free(mem->memory_begin);
    *mem = (PrincipleMemory){0};
}

int init_program_thread(ProgramThread* pu) {
    // enable the pu
    for (int i=0; i<REG32_COUNT; i++) {
        pu->gp32[i].full = 0;
    }
    for (int i=0; i<REG64_COUNT; i++) {
        pu->gp64[i].full = 0;
    }
    for (int i=0; i<REGF32_COUNT; i++) {
        pu->f32[i] = 0;
    }

    pu->gp32[REG_PF].full = TGX_PF_Flag_SysActiveBit;

    pu->gp32[REG_SH].full = ROM_SIZE + PROGRAM_CACHE_SIZE;
    pu->gp32[REG_SB].full = (ROM_SIZE + PROGRAM_CACHE_SIZE) - DEFAULT_STACK_SIZE;

    pu->gp32[REG_SP].full = pu->gp32[REG_SH].full;

    return TGX_SUCCESS;
}

void destroy_program_thread(ProgramThread* pu){

}
int init_graphics_thread(GraphicsThread* gu){

    return TGX_SUCCESS;
}

void destroy_graphics_thread(GraphicsThread* gu){

}

#define TGX_CASE(lbl) tgx_##lbl
#define TGX_ADDR(lbl) &&tgx_##lbl

/*(sys).PU.gp32[REG_JR].full = (sys).PU.gp32[REG_IP].full; \*/
// We only really need to update JR on jumps. There really isn't a need to update it
// with every instruction
#define TGX_NEXT_INSTR(sys) (sys).PU.gp32[REG_IP].full += (uint32_t)sizeof(Instruction)

#define TGX_DISPATCH(table, instr, sys) instr = *((Instruction*)((sys).Memory.memory_begin + (sys).PU.gp32[REG_IP].full)); goto *table[instr.opcode]

//#define TGX_PROFILE

#ifdef TGX_PROFILE

#include <stdio.h>

typedef struct {
    const char* name;
    uint64_t calls;
    uint64_t total_time;
} ProfilerEntry_;

static ProfilerEntry_ ProfilerTable_[TGX_OPCODE_COUNT];
static uint64_t ProfilerTimestamp_Start_, ProfilerTimestamp_End_;

#define TGX_PROFILER_INITIALIZE() for(size_t i=0; i<TGX_OPCODE_COUNT; i++) ProfilerTable_[i] = (ProfilerEntry_){ 0 }

#define TGX_PROFILE_CALL(op, code) ProfilerTable_[code].name = #op; ProfilerTable_[code].calls++; CPU_TICKS(ProfilerTimestamp_Start_);
#define TGX_PROFILE_END(op, code) CPU_TICKS(ProfilerTimestamp_End_); ProfilerTable_[code].total_time += ProfilerTimestamp_End_ - ProfilerTimestamp_Start_;

#define TGX_PROFILE_REPORT(file) do { FILE *__pfr = fopen(file, "w");  \
    if(__pfr != NULL) { \
        for(size_t i=0; i<TGX_OPCODE_COUNT; i++) { \
            fprintf(__pfr, "%15s[%4lX]| Calls, Total Time, Avg Time: %9lu, %9lu, %f\n", ProfilerTable_[i].name, i, ProfilerTable_[i].calls, ProfilerTable_[i].total_time, (float)ProfilerTable_[i].total_time / (float)ProfilerTable_[i].calls); \
        } \
        fclose(__pfr); \
    } } while(false)

#else

#define TGX_PROFILE_REPORT(file)
#define TGX_PROFILER_INITIALIZE()
#define TGX_PROFILE_CALL(op, code)
#define TGX_PROFILE_END(op, code)

#endif

#define CLEAR_BIT(reg, mask) (reg) &= ~(mask)

#define REG64(sys, idx) (sys->PU.gp64[idx]).full
#define REG64L(sys, idx) (sys->PU.gp64[idx]).half[0]
#define REG64H(sys, idx) (sys->PU.gp64[idx]).half[1]

#define REG32(sys, idx) (sys->PU.gp32[idx]).full
#define REG16(sys, idx) (sys->PU.gp16[idx])
#define REG8(sys, idx) (sys->PU.gp32[idx]).single[0]
#define REGF32(sys, idx) (sys->PU.f32[idx])

#define RVEC(sys, idx) (sys->PU.vec[idx])

#define CONST48(instr) ((((uint64_t)instr.param16) << 32) | instr.const_i32)
#define CONST40(instr) ((((uint64_t)instr.params[1]) << 32) | instr.const_i32)

#define GEN_FLAG_IS_ZERO(reg) (((reg) == 0) << TGX_PF_Shift_ZeroBit)

//TODO: Use memprotect to enforce ROM in allocated ROM section
#define MEMPTR(type, sys, index) (type*)(sys->Memory.memory_begin + (index))
#define MEMACCESS(type, sys, index) *MEMPTR(type, sys, index)
//#define MEMACCESS(type, sys, index) *(type*)(sys->Memory.program_cache_begin + (index))

#define UPDATE_PF_ZERO_NEG_BITS_I(sys, source, size) CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit); \
    REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(source) | GEN_FLAG_IS_NEG(source, size)
#define UPDATE_PF_ZERO_NEG_BITS_F(sys, source) CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit); \
    REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(source) | GEN_FLAG_IS_NEGF32(source)

#define BRANCHING_INSTRUCTION

#define SQR(a) ((a) * (a))

//NOTE: GodBolt shows that the compiler emits the same assembly for reg < 0 and reg & sign_bit. So in reality it's
// a matter of which one is clearer, and it doesn't matter so much in the end as far as end assembly is concerned
#define GEN_FLAG_IS_NEG(reg, size) (((int##size##_t)(reg) < 0) << TGX_PF_Shift_NegBit)
#define GEN_FLAG_IS_NEGF32(reg) ((reg < 0.0f) << TGX_PF_Shift_NegBit)

static int LaunchDebugShell(TGXContext* ctx);

int program_thread_exec(TGXContext* sys){
    void* _jTable[] = {
/* 0x0000 */        TGX_ADDR(NOP),               TGX_ADDR(MOVR32_R32),     TGX_ADDR(MOVR16_R16),     TGX_ADDR(MOVR8_R8),
/* 0x0004 */        TGX_ADDR(MOVR64_R64),        TGX_ADDR(MOVAR32_C32),    TGX_ADDR(MOVAR32_C8),     TGX_ADDR(MOVTBL_R32_IC8),
/* 0x0008 */        TGX_ADDR(MOVTBL_R32_IR8),    TGX_ADDR(MOVR32_TBL_IC8), TGX_ADDR(MOVR32_TBL_IR8), TGX_ADDR(MOVR32_R64H),
/* 0x000C */        TGX_ADDR(MOVR32_R64L),       TGX_ADDR(MOVR64H_R32),    TGX_ADDR(MOVR64L_R32),    TGX_ADDR(MOVR32_C32),
/* 0x0010 */        TGX_ADDR(MOVR16_C16),        TGX_ADDR(MOVR8_C8),       TGX_ADDR(MOVM0_C48),      TGX_ADDR(MOVM1_C48),
/* 0x0014 */        TGX_ADDR(MOVF32_F32),        TGX_ADDR(MOVF32_C32),     TGX_ADDR(MOVR32_F32),     TGX_ADDR(MOVR32_F32_C),
/* 0x0018 */        TGX_ADDR(MOVF32_R32),        TGX_ADDR(MOVF32_R32_C),   TGX_ADDR(MOVR32_AR32),    TGX_ADDR(MOVR8_AR32),
/* 0x001C */        TGX_ADDR(MOVR16_AR32),       TGX_ADDR(MOVR64_AR32),    TGX_ADDR(MOVAR32_R8),     TGX_ADDR(MOVAR32_R16),
/* 0x0020 */        TGX_ADDR(MOVAR32_R32),       TGX_ADDR(MOVAR32_R64),    TGX_ADDR(MOVAR32_F32),    TGX_ADDR(MOVAR32_AR32),
/* 0x0024 */        TGX_ADDR(MOVF32_AR32),       TGX_ADDR(MOVAR32_AR32_I), TGX_ADDR(MOVAR32_AR32_D), TGX_ADDR(SWAP_R32),
/* 0x0028 */        TGX_ADDR(SWAP_R8),           TGX_ADDR(SWAP_R16),       TGX_ADDR(SWAP_R64),       TGX_ADDR(SWAP_F32),
/* 0x002C */        TGX_ADDR(MEMCPY),            TGX_ADDR(MEMCMP),         TGX_ADDR(MEMCLR_R8),      TGX_ADDR(MEMCLR_R16),
/* 0x0030 */        TGX_ADDR(MEMCLR_R32),        TGX_ADDR(MEMCLR_R64),     TGX_ADDR(PUSH_R8),        TGX_ADDR(PUSH_C8),
/* 0x0034 */        TGX_ADDR(PUSH_R16),          TGX_ADDR(PUSH_C16),       TGX_ADDR(PUSH_R32),       TGX_ADDR(PUSH_C32),
/* 0x0038 */        TGX_ADDR(PUSH_R64),          TGX_ADDR(PUSH_C48),       TGX_ADDR(PUSH_F32),       TGX_ADDR(PUSH_CF32),
/* 0x003C */        TGX_ADDR(POP_R8),            TGX_ADDR(POP_R16),        TGX_ADDR(POP_R32),        TGX_ADDR(POP_R64),
/* 0x0040 */        TGX_ADDR(POP_F32),           TGX_ADDR(PUSH_RARZ),      TGX_ADDR(POP_RZRA),       TGX_ADDR(MOVR32_TBL_R8_R8),
/* 0x0044 */        TGX_ADDR(MOVTBL_R8_R8_AR32),
/* 0x0045 */        TGX_ADDR(ADDR32_R32),        TGX_ADDR(ADDR8_R8),       TGX_ADDR(ADDR16_R16),     TGX_ADDR(ADDR64_R64),
/* 0x0049 */        TGX_ADDR(ADDR32_C32),        TGX_ADDR(ADDR8_C8),       TGX_ADDR(ADDR16_C16),     TGX_ADDR(ADDR64_C40),
/* 0x004D */        TGX_ADDR(SUBR32_R32),        TGX_ADDR(SUBR8_R8),       TGX_ADDR(SUBR16_R16),     TGX_ADDR(SUBR64_R64),
/* 0x0051 */        TGX_ADDR(SUBR32_C32),        TGX_ADDR(SUBR8_C8),       TGX_ADDR(SUBR16_C16),     TGX_ADDR(SUBR64_C40),
/* 0x0055 */        TGX_ADDR(MULR32_R32),        TGX_ADDR(MULR8_R8),       TGX_ADDR(MULR16_R16),     TGX_ADDR(MULR64_R64),
/* 0x0059 */        TGX_ADDR(MULR32_C32),        TGX_ADDR(MULR8_C8),       TGX_ADDR(MULR16_C16),     TGX_ADDR(MULR64_C40),
/* 0x005D */        TGX_ADDR(DIVR32_R32),        TGX_ADDR(DIVR8_R8),       TGX_ADDR(DIVR16_R16),     TGX_ADDR(DIVR64_R64),
/* 0x0061 */        TGX_ADDR(DIVR32_C32),        TGX_ADDR(DIVR8_C8),       TGX_ADDR(DIVR16_C16),     TGX_ADDR(DIVR64_C40),
/* 0x0065 */        TGX_ADDR(MODR32_R32),        TGX_ADDR(MODR8_R8),       TGX_ADDR(MODR16_R16),     TGX_ADDR(MODR64_R64),
/* 0x0069 */        TGX_ADDR(MODR32_C32),        TGX_ADDR(MODR8_C8),       TGX_ADDR(MODR16_C16),     TGX_ADDR(MODR64_C40),
/* 0x006D */        TGX_ADDR(NEGR32),            TGX_ADDR(NEGR8),          TGX_ADDR(NEGR16),         TGX_ADDR(NEGR64),
/* 0x0071 */        TGX_ADDR(INCR32),            TGX_ADDR(INCR8),          TGX_ADDR(INCR16),         TGX_ADDR(INCR64),
/* 0x0075 */        TGX_ADDR(DECR32),            TGX_ADDR(DECR8),          TGX_ADDR(DECR16),         TGX_ADDR(DECR64),
/* 0x0079 */        TGX_ADDR(RBRR32_R32),        TGX_ADDR(RBRR8_R8),       TGX_ADDR(RBRR16_R16),     TGX_ADDR(RBRR64_R64),
/* 0x007D */        TGX_ADDR(RBRR32_C32),        TGX_ADDR(RBRR8_C8),       TGX_ADDR(RBRR16_C16),     TGX_ADDR(RBRR64_C40),
/* 0x0081 */        TGX_ADDR(RBL3R2_R32),        TGX_ADDR(RBLR8_R8),       TGX_ADDR(RBLR16_R16),     TGX_ADDR(RBLR64_R64),
/* 0x0085 */        TGX_ADDR(RBLR32_C32),        TGX_ADDR(RBLR8_C8),       TGX_ADDR(RBLR16_C16),     TGX_ADDR(RBLR64_C40),
/* 0x0089 */        TGX_ADDR(CMPR32_R32),        TGX_ADDR(CMPR8_R8),       TGX_ADDR(CMPR16_R16),     TGX_ADDR(CMPR64_R64),
/* 0x008D */        TGX_ADDR(CMPR32_C32),        TGX_ADDR(CMPR8_C8),       TGX_ADDR(CMPR16_C16),     TGX_ADDR(CMPR64_C40),
/* 0x0091 */        TGX_ADDR(LANDR32_R32),       TGX_ADDR(LANDR8_R8),      TGX_ADDR(LANDR16_R16),    TGX_ADDR(LANDR64_R64),
/* 0x0095 */        TGX_ADDR(LORR32_R32),        TGX_ADDR(LORR8_R8),       TGX_ADDR(LORR16_R16),     TGX_ADDR(LORR64_R64),
/* 0x0099 */        TGX_ADDR(LNOTR32_R32),       TGX_ADDR(LNOTR8_R8),      TGX_ADDR(LNOTR16_R16),    TGX_ADDR(LNOTR64_R64),
/* 0x009D */        TGX_ADDR(ANDR32_R32),        TGX_ADDR(ANDR8_R8),       TGX_ADDR(ANDR16_R16),     TGX_ADDR(ANDR64_R64),
/* 0x00A1 */        TGX_ADDR(NOP),
/* 0x00A2 */        TGX_ADDR(XORR32_R32),        TGX_ADDR(XORR8_R8),       TGX_ADDR(XORR16_R16),     TGX_ADDR(XORR64_R64),
/* 0x00A6 */        TGX_ADDR(ORR32_R32),         TGX_ADDR(ORR8_R8),        TGX_ADDR(ORR16_R16),      TGX_ADDR(ORR64_R64),
/* 0x00AA */        TGX_ADDR(NOTR32_R32),        TGX_ADDR(NOTR8_R8),       TGX_ADDR(NOTR16_R16),     TGX_ADDR(NOTR64_R64),
/* 0x00AE */        TGX_ADDR(MXB32),             TGX_ADDR(MXBF32),          TGX_ADDR(SQR32),          TGX_ADDR(SQR64),
/* 0x00B2 */        TGX_ADDR(ABS8),              TGX_ADDR(ABS16),          TGX_ADDR(ABS32),          TGX_ADDR(ABS64),
/* 0x00B6 */        TGX_ADDR(ADDF32),            TGX_ADDR(ADDFC32),        TGX_ADDR(SUBF32),         TGX_ADDR(SUBFC32),
/* 0x00BA */        TGX_ADDR(MULF32),            TGX_ADDR(MULFC32),        TGX_ADDR(DIVF32),         TGX_ADDR(DIVFC32),
/* 0x00BE */        TGX_ADDR(MODF32),            TGX_ADDR(MODFC32),        TGX_ADDR(FLRF32),         TGX_ADDR(CEILF32),
/* 0x00C2 */        TGX_ADDR(ROUNDF32),          TGX_ADDR(SQRTF32),        TGX_ADDR(CMPF32),         TGX_ADDR(CMPFC32),
/* 0x00C6 */        TGX_ADDR(VEC4NORM),          TGX_ADDR(VEC4ADD),        TGX_ADDR(VEC4SUB),        TGX_ADDR(VEC4MUL),
/* 0x00CA */        TGX_ADDR(VEC4DIV),           TGX_ADDR(DOT),            TGX_ADDR(LEN),            TGX_ADDR(COSF32),
/* 0x00CE */        TGX_ADDR(COSFC32),           TGX_ADDR(SINF32),         TGX_ADDR(SINFC32),        TGX_ADDR(TANF32),
/* 0x00D2 */        TGX_ADDR(TANFC32),           TGX_ADDR(ACOSF32),        TGX_ADDR(ACOSFC32),       TGX_ADDR(ASINF32),
/* 0x00D6 */        TGX_ADDR(ASINFC32),          TGX_ADDR(ATANF32),        TGX_ADDR(ATANFC32),       TGX_ADDR(ATAN2),
/* 0x00DA */        TGX_ADDR(ACOSBCD),           TGX_ADDR(ASINBCD),        TGX_ADDR(ATANBCD),        TGX_ADDR(NEGF32),
/* 0x00DE */        TGX_ADDR(SWZL),              TGX_ADDR(POWF32),         TGX_ADDR(LOGF32),         TGX_ADDR(LNF32),
/* 0x00E2 */        TGX_ADDR(EF32),              TGX_ADDR(LDPI),           TGX_ADDR(LDE),            TGX_ADDR(INVERSE),
/* 0x00E6 */        TGX_ADDR(ABSF32),
/* 0x00E7 */        TGX_ADDR(JMPPR32),           TGX_ADDR(JMPC32),         TGX_ADDR(JZR32),          TGX_ADDR(JZC32),
/* 0x00EB */        TGX_ADDR(JNZR32),            TGX_ADDR(JNZC32),         TGX_ADDR(JZDR32),         TGX_ADDR(JNZDR32),
/* 0x00EF */        TGX_ADDR(JGTR32),            TGX_ADDR(JGTC32),         TGX_ADDR(JLTR32),         TGX_ADDR(JLTC32),
/* 0x00F3 */        TGX_ADDR(JGER32),            TGX_ADDR(JGEC32),         TGX_ADDR(JLER32),         TGX_ADDR(JLEC32),
/* 0x00F7 */        TGX_ADDR(NOP),               TGX_ADDR(NOP),            TGX_ADDR(NOP),            TGX_ADDR(NOP),
/* 0x00FB */        TGX_ADDR(NOP),               TGX_ADDR(NOP),            TGX_ADDR(NOP),            TGX_ADDR(NOP),
/* 0x00FF */        TGX_ADDR(NOP),               TGX_ADDR(NOP),            TGX_ADDR(NOP),            TGX_ADDR(NOP),
/* 0x0103 */        TGX_ADDR(NOP),               TGX_ADDR(NOP),            TGX_ADDR(NOP),            TGX_ADDR(NOP),
/* 0x0107 */        TGX_ADDR(NOP),               TGX_ADDR(NOP),            TGX_ADDR(NOP),            TGX_ADDR(NOP),
/* 0x010B */        TGX_ADDR(RET),               TGX_ADDR(HLT),            TGX_ADDR(INTR8),          TGX_ADDR(INTC8),
/* 0x010F */        TGX_ADDR(INITEXR32),         TGX_ADDR(INITEXC32),      TGX_ADDR(INVKEXR32),      TGX_ADDR(INVKEXC32),
/* 0x0113 */        TGX_ADDR(DEINITEXR32),       TGX_ADDR(DEINITEXC32),    TGX_ADDR(SYSCALLC32),     TGX_ADDR(BREAK),
/* 0x0117 */        TGX_ADDR(GQPS),              TGX_ADDR(GQS),            TGX_ADDR(GQAI),           TGX_ADDR(GQR),
/* 0x011B */        TGX_ADDR(GQSI),              TGX_ADDR(GQPC),           TGX_ADDR(GQPF),           TGX_ADDR(MOVRR32),
    };

    uint8_t t8;
    uint16_t t16;
    uint32_t t32;
    uint64_t t64;
    float tf32;
    int64_t ti64;


    TGX_PROFILER_INITIALIZE();

    // TODO: Extension Ports
    // TODO: Graphics API
    Instruction instruction;

    TGX_DISPATCH(_jTable, instruction, *sys);

    TGX_CASE(NOP):
    TGX_PROFILE_CALL(NOP, 0x0000);
    // NOP
    TGX_PROFILE_END(NOP, 0x0000);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_R32):
    TGX_PROFILE_CALL(MOVR32_R32, 0x0001);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_R32, 0x0001);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR16_R16):
    TGX_PROFILE_CALL(MOVR16_R16, 0x0002);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG16(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MOVR16_R16, 0x0002);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR8_R8):
    TGX_PROFILE_CALL(MOVR8_R8, 0x0003);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG8(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MOVR8_R8, 0x0003);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR64_R64):
    TGX_PROFILE_CALL(MOVR64_R64, 0x0004);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG64(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MOVR64_R64, 0x0004);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


    TGX_CASE(MOVAR32_C32):
    TGX_PROFILE_CALL(MOVAR32_C32, 0x0005);

    MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[0])) = instruction.const_i32;

    TGX_PROFILE_END(MOVAR32_C32, 0x0005);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);

    TGX_CASE(MOVAR32_C8):
    TGX_PROFILE_CALL(MOVAR32_C8, 0x0006);

    MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[0])) = instruction.ext_params[0];

    TGX_PROFILE_END(MOVAR32_C8, 0x0006);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);

    TGX_CASE(MOVTBL_R32_IC8):
    TGX_PROFILE_CALL(MOVTBL_R32_IC8, 0x0007);

    sys->PU.int_table[instruction.ext_params[0]] = REG32(sys, instruction.params[1]);

    TGX_PROFILE_END(MOVTBL_R32_IC8, 0x0007);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVTBL_R32_IR8):
    TGX_PROFILE_CALL(MOVTBL_R32_IR8, 0x0008);

    sys->PU.int_table[REG8(sys, instruction.params[0])] = REG32(sys, instruction.params[1]);

    TGX_PROFILE_END(MOVTBL_R32_IR8, 0x0008);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_TBL_IC8):
    TGX_PROFILE_CALL(MOVR32_TBL_IC8, 0x0009);

    REG32(sys, instruction.params[0]) = sys->PU.int_table[instruction.ext_params[0]];

    TGX_PROFILE_END(MOVR32_TBL_IC8, 0x0009);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_TBL_IR8):
    TGX_PROFILE_CALL(MOVR32_TBL_IR8, 0x000A);

    REG32(sys, instruction.params[0]) = sys->PU.int_table[REG8(sys, instruction.params[1])];

    TGX_PROFILE_END(MOVR32_TBL_IR8, 0x000A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_R64H):
    TGX_PROFILE_CALL(MOVR32_R64H, 0x000B);

    REG32(sys, instruction.params[0]) = REG64H(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_R64H, 0x000B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_R64L):
    TGX_PROFILE_CALL(MOVR32_R64L, 0x000C);

    REG32(sys, instruction.params[0]) = REG64L(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_R64L, 0x000C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR64H_R32):
    TGX_PROFILE_CALL(MOVR64H_R32, 0x000D);

    REG64H(sys, instruction.params[0]) = REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64H(sys, instruction.params[0]), 64);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG64H(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG64H(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MOVR64H_R32, 0x000D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR64L_R32):
    TGX_PROFILE_CALL(MOVR64L_R32, 0x000E);

    REG64L(sys, instruction.params[0]) = REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64L(sys, instruction.params[0]), 64);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG64L(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG64L(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MOVR64L_R32, 0x000E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


    TGX_CASE(MOVR32_C32):
    TGX_PROFILE_CALL(MOVR32_C32, 0x000F);

    REG32(sys, instruction.params[0]) = instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_C32, 0x000F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR16_C16):
    TGX_PROFILE_CALL(MOVR16_C16, 0x0010);

    REG16(sys, instruction.params[0]) = instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG16(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MOVR16_C16, 0x0010);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR8_C8):
    TGX_PROFILE_CALL(MOVR8_C8, 0x0011);

    REG8(sys, instruction.params[0]) = instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG8(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MOVR8_C8, 0x0011);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVM0_C48):
    TGX_PROFILE_CALL(MOVM0_C48, 0x0012);

    REG64(sys, 0) = CONST48(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, 0), 64);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG64(sys, 0)) | GEN_FLAG_IS_NEG(REG64(sys, 0), 64);

    TGX_PROFILE_END(MOVM0_C48, 0x0012);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVM1_C48):
    TGX_PROFILE_CALL(MOVM1_C48, 0x0013);

    REG64(sys, 1) = CONST48(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, 1), 64);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG64(sys, 0)) | GEN_FLAG_IS_NEG(REG64(sys, 0), 64);

    TGX_PROFILE_END(MOVM1_C48, 0x0013);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


    TGX_CASE(MOVF32_F32):
    TGX_PROFILE_CALL(MOVF32_F32, 0x0014);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REGF32(sys, instruction.params[0])) | GEN_FLAG_IS_NEGF32(REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MOVF32_F32, 0x0014);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


    TGX_CASE(MOVF32_C32):
    TGX_PROFILE_CALL(MOVF32_C32, 0x0015);

    REGF32(sys, instruction.params[0]) = instruction.const_f32;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REGF32(sys, instruction.params[0])) | GEN_FLAG_IS_NEGF32(REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MOVF32_C32, 0x0015);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_F32):
    TGX_PROFILE_CALL(MOVR32_F32, 0x0016);

    REG32(sys, instruction.params[0]) = *(uint32_t*)(&REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_F32, 0x0016);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_F32_C):
    TGX_PROFILE_CALL(MOVR32_F32_C, 0x0017);

    REG32(sys, instruction.params[0]) = (uint32_t)(int32_t)REGF32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_F32_C, 0x0017);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVF32_R32):
    TGX_PROFILE_CALL(MOVF32_R32, 0x0018);

    REGF32(sys, instruction.params[0]) = *(float*)(&REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REGF32(sys, instruction.params[0])) | GEN_FLAG_IS_NEGF32(REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MOVF32_R32, 0x0018);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVF32_R32_C):
    TGX_PROFILE_CALL(MOVF32_R32_C, 0x0019);

    REGF32(sys, instruction.params[0]) = (float)(REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REGF32(sys, instruction.params[0])) | GEN_FLAG_IS_NEGF32(REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MOVF32_R32_C, 0x0019);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_AR32):
    TGX_PROFILE_CALL(MOVR32_AR32, 0x001A);

    REG32(sys, instruction.params[0]) = MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);
    //CLEAR_BIT(REG32(sys, REG_PF), TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit);
    //REG32(sys, REG_PF) |= GEN_FLAG_IS_ZERO(REG32(sys, instruction.params[0])) | GEN_FLAG_IS_NEG(REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MOVR32_AR32, 0x001A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR8_AR32):
    TGX_PROFILE_CALL(MOVR8_AR32, 0x001B);

    REG8(sys, instruction.params[0]) = MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MOVR8_AR32, 0x001B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR16_AR32):
    TGX_PROFILE_CALL(MOVR16_AR32, 0x001C);

    REG16(sys, instruction.params[0]) = MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MOVR16_AR32, 0x001C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR64_AR32):
    TGX_PROFILE_CALL(MOVR64_AR32, 0x001D);

    REG64(sys, instruction.params[0]) = MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MOVR64_AR32, 0x001D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_R8):
    TGX_PROFILE_CALL(MOVAR32_R8, 0x001E);

    MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[0])) = REG8(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[1]), 8);

    TGX_PROFILE_END(MOVAR32_R8, 0x001E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_R16):
    TGX_PROFILE_CALL(MOVAR32_R16, 0x001F);

    MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[0])) = REG16(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[1]), 16);

    TGX_PROFILE_END(MOVAR32_R16, 0x001F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_R32):
    TGX_PROFILE_CALL(MOVAR32_R32, 0x0020);

    MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[0])) = REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[1]), 32);

    TGX_PROFILE_END(MOVAR32_R32, 0x0020);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_R64):
    TGX_PROFILE_CALL(MOVAR32_R64, 0x0021);

    MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[0])) = REG64(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[1]), 64);

    TGX_PROFILE_END(MOVAR32_R64, 0x0021);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_F32):
    TGX_PROFILE_CALL(MOVAR32_F32, 0x0022);

    MEMACCESS(float, sys, REG32(sys, instruction.params[0])) = REGF32(sys, instruction.params[1]);

    TGX_PROFILE_END(MOVAR32_F32, 0x0022);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_AR32):
    TGX_PROFILE_CALL(MOVAR32_AR32, 0x0023);
    BRANCHING_INSTRUCTION

    switch (instruction.ext_params[0]) {
        default:
        case MODE8:
            MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[1]));
            break;
        case MODE16:
            MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[1]));
            break;
        case MODE32:
            MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[1]));
            break;
        case MODE64:
            MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[1]));
            break;
    }

    TGX_PROFILE_END(MOVAR32_AR32, 0x0023);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVF32_AR32):
    TGX_PROFILE_CALL(MOVF32_AR32, 0x0024);

    REGF32(sys, instruction.params[0]) = MEMACCESS(float, sys, REG32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MOVF32_AR32, 0x0024);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_AR32_I):
    TGX_PROFILE_CALL(MOVAR32_AR32_I, 0x0025);
    BRANCHING_INSTRUCTION

    switch (instruction.ext_params[0]) {
        default:
        case MODE8:
            MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[1]));
            REG32(sys, instruction.params[0]) += sizeof(uint8_t); REG32(sys, instruction.params[1]) += sizeof(uint8_t);
        break;
        case MODE16:
            MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[1]));
            REG32(sys, instruction.params[0]) += sizeof(uint16_t); REG32(sys, instruction.params[1]) += sizeof(uint16_t);
        break;
        case MODE32:
            MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[1]));
            REG32(sys, instruction.params[0]) += sizeof(uint32_t); REG32(sys, instruction.params[1]) += sizeof(uint32_t);
        break;
        case MODE64:
            MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[1]));
            REG32(sys, instruction.params[0]) += sizeof(uint64_t); REG32(sys, instruction.params[1]) += sizeof(uint64_t);
        break;
    }

    TGX_PROFILE_END(MOVAR32_AR32_I, 0x0025);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVAR32_AR32_D):
    TGX_PROFILE_CALL(MOVAR32_AR32_D, 0x0026);
    BRANCHING_INSTRUCTION

        switch (instruction.ext_params[0]) {
            default:
            case MODE8:
                MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint8_t, sys, REG32(sys, instruction.params[1]));
                REG32(sys, instruction.params[0]) -= sizeof(uint8_t); REG32(sys, instruction.params[1]) -= sizeof(uint8_t);
            break;
            case MODE16:
                MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint16_t, sys, REG32(sys, instruction.params[1]));
                REG32(sys, instruction.params[0]) -= sizeof(uint16_t); REG32(sys, instruction.params[1]) -= sizeof(uint16_t);
            break;
            case MODE32:
                MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint32_t, sys, REG32(sys, instruction.params[1]));
                REG32(sys, instruction.params[0]) -= sizeof(uint32_t); REG32(sys, instruction.params[1]) -= sizeof(uint32_t);
            break;
            case MODE64:
                MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[0])) = MEMACCESS(uint64_t, sys, REG32(sys, instruction.params[1]));
                REG32(sys, instruction.params[0]) -= sizeof(uint64_t); REG32(sys, instruction.params[1]) -= sizeof(uint64_t);
            break;
        }
    TGX_PROFILE_END(MOVAR32_AR32_D, 0x0026);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWAP_R32):
    TGX_PROFILE_CALL(SWAP_R32, 0x0027);

    t32 = REG32(sys, instruction.params[0]);
    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]);
    REG32(sys, instruction.params[1]) = t32;

    TGX_PROFILE_END(SWAP_R32, 0x0027);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWAP_R8):
    TGX_PROFILE_CALL(SWAP_R8, 0x0028);

    t8 = REG8(sys, instruction.params[0]);
    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]);
    REG8(sys, instruction.params[1]) = t8;

    TGX_PROFILE_END(SWAP_R8, 0x0028);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWAP_R16):
    TGX_PROFILE_CALL(SWAP_R16, 0x0029);

    t16 = REG16(sys, instruction.params[0]);
    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]);
    REG16(sys, instruction.params[1]) = t16;

    TGX_PROFILE_END(SWAP_R16, 0x0029);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWAP_R64):
    TGX_PROFILE_CALL(SWAP_R64, 0x002A);

    t64 = REG64(sys, instruction.params[0]);
    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]);
    REG64(sys, instruction.params[1]) = t64;

    TGX_PROFILE_END(SWAP_R64, 0x002A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWAP_F32):
    TGX_PROFILE_CALL(SWAP_F32, 0x002B);

    tf32 = REGF32(sys, instruction.params[0]);
    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]);
    REGF32(sys, instruction.params[1]) = tf32;

    TGX_PROFILE_END(SWAP_F32, 0x002B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCPY):
    TGX_PROFILE_CALL(MEMCPY, 0x002C);

    memcpy(MEMPTR(void, sys, instruction.params[0]), MEMPTR(void, sys, instruction.params[1]), (size_t)REG32(sys, instruction.ext_params[0]));

    TGX_PROFILE_END(MEMCPY, 0x002C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCMP):
    TGX_PROFILE_CALL(MEMCMP, 0x002D);

    REG32(sys, instruction.params[0]) = memcmp(MEMPTR(void, sys, REG32(sys, instruction.params[1])),
                                MEMPTR(void, sys, REG32(sys, instruction.ext_params[0])),
                                (size_t)REG32(sys, instruction.ext_params[1]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MEMCMP, 0x002D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCLR_R8):
    TGX_PROFILE_CALL(MEMCLR_R8, 0x002E);

    memset(MEMPTR(void, sys, REG32(sys, instruction.params[0])), REG8(sys, instruction.ext_params[0]), REG32(sys, instruction.params[1]) - REG32(sys, instruction.params[0]));

    TGX_PROFILE_END(MEMCLR_R8, 0x002E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCLR_R16):
    TGX_PROFILE_CALL(MEMCLR_R16, 0x002F);

    t32 = REG32(sys, instruction.params[0]);
    while (t32 < REG32(sys, instruction.params[1])) {
        MEMACCESS(uint16_t, sys, t32) = REG16(sys, instruction.ext_params[0]);
        t32 += sizeof(uint16_t);
    }

    TGX_PROFILE_END(MEMCLR_R16, 0x002F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCLR_R32):
    TGX_PROFILE_CALL(MEMCLR_R32, 0x0030);

    t32 = REG32(sys, instruction.params[0]);
    while (t32 < REG32(sys, instruction.params[1])) {
        MEMACCESS(uint32_t, sys, t32) = REG32(sys, instruction.ext_params[0]);
        t32 += sizeof(uint32_t);
    }

    TGX_PROFILE_END(MEMCLR_R32, 0x0030);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MEMCLR_R64):
    TGX_PROFILE_CALL(MEMCLR_R64, 0x0031);

    t32 = REG32(sys, instruction.params[0]);
    while (t32 < REG32(sys, instruction.params[1])) {
        MEMACCESS(uint64_t, sys, t32) = REG64(sys, instruction.ext_params[0]);
        t32 += sizeof(uint64_t);
    }

    TGX_PROFILE_END(MEMCLR_R64, 0x0031);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_R8):
    TGX_PROFILE_CALL(PUSH_R8, 0x0032);

    REG32(sys, REG_SP) -= sizeof(uint8_t);
    MEMACCESS(uint8_t, sys, REG32(sys, REG_SP)) = REG8(sys, instruction.params[0]);

    TGX_PROFILE_END(PUSH_R8, 0x0032);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_C8):
    TGX_PROFILE_CALL(PUSH_C8, 0x0033);

    REG32(sys, REG_SP) -= sizeof(uint8_t);
    MEMACCESS(uint8_t, sys, REG32(sys, REG_SP)) = instruction.params[0];

    TGX_PROFILE_END(PUSH_C8, 0x0033);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_R16):
    TGX_PROFILE_CALL(PUSH_R16, 0x0034);

    REG32(sys, REG_SP) -= sizeof(uint16_t);
    MEMACCESS(uint16_t, sys, REG32(sys, REG_SP)) = REG16(sys, instruction.params[0]);

    TGX_PROFILE_END(PUSH_R16, 0x0034);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_C16):
    TGX_PROFILE_CALL(PUSH_C16, 0x0035);

    REG32(sys, REG_SP) -= sizeof(uint16_t);
    MEMACCESS(uint16_t, sys, REG32(sys, REG_SP)) = instruction.ext_params16[0];

    TGX_PROFILE_END(PUSH_C16, 0x0035);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_R32):
    TGX_PROFILE_CALL(PUSH_R32, 0x0036);

    REG32(sys, REG_SP) -= sizeof(uint32_t);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)) = REG32(sys, instruction.params[0]);

    TGX_PROFILE_END(PUSH_R32, 0x0036);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_C32):
    TGX_PROFILE_CALL(PUSH_C32, 0x0037);

    REG32(sys, REG_SP) -= sizeof(uint32_t);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)) = instruction.const_i32;

    TGX_PROFILE_END(PUSH_C32, 0x0037);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_R64):
    TGX_PROFILE_CALL(PUSH_R64, 0x0038);

    REG32(sys, REG_SP) -= sizeof(uint64_t);
    MEMACCESS(uint64_t, sys, REG32(sys, REG_SP)) = REG64(sys, instruction.params[0]);

    TGX_PROFILE_END(PUSH_R64, 0x0038);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_C48):
    TGX_PROFILE_CALL(PUSH_C48, 0x0039);

    REG32(sys, REG_SP) -= sizeof(uint64_t);
    MEMACCESS(uint64_t, sys, REG32(sys, REG_SP)) = CONST48(instruction);

    TGX_PROFILE_END(PUSH_C48, 0x0039);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_F32):
    TGX_PROFILE_CALL(PUSH_F32, 0x003A);

    REG32(sys, REG_SP) -= sizeof(float);
    MEMACCESS(float, sys, REG32(sys, REG_SP)) = REGF32(sys, instruction.params[0]);

    TGX_PROFILE_END(PUSH_F32, 0x003A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_CF32):
    TGX_PROFILE_CALL(PUSH_CF32, 0x003B);

    REG32(sys, REG_SP) -= sizeof(float);
    MEMACCESS(float, sys, REG32(sys, REG_SP)) = instruction.const_f32;

    TGX_PROFILE_END(PUSH_CF32, 0x003B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_R8):
    TGX_PROFILE_CALL(POP_R8, 0x003C);

    REG8(sys, instruction.params[0]) = MEMACCESS(uint8_t, sys, REG32(sys, REG_SP));
    REG32(sys, REG_SP) += sizeof(uint8_t);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(POP_R8, 0x003C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_R16):
    TGX_PROFILE_CALL(POP_R16, 0x003D);

    REG16(sys, instruction.params[0]) = MEMACCESS(uint16_t, sys, REG32(sys, REG_SP));
    REG32(sys, REG_SP) += sizeof(uint16_t);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(POP_R16, 0x003D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_R32):
    TGX_PROFILE_CALL(POP_R32, 0x003E);

    REG32(sys, instruction.params[0]) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP));
    REG32(sys, REG_SP) += sizeof(uint32_t);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(POP_R32, 0x003E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_R64):
    TGX_PROFILE_CALL(POP_R64, 0x003F);

    REG64(sys, instruction.params[0]) = MEMACCESS(uint64_t, sys, REG32(sys, REG_SP));
    REG32(sys, REG_SP) += sizeof(uint64_t);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(POP_R64, 0x003F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_F32):
    TGX_PROFILE_CALL(POP_F32, 0x0040);

    REGF32(sys, instruction.params[0]) = MEMACCESS(float, sys, REG32(sys, REG_SP));
    REG32(sys, REG_SP) += sizeof(float);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(POP_F32, 0x0040);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(PUSH_RARZ):
    TGX_PROFILE_CALL(PUSH_RARZ, 0x0041);

#define RARZ_NUM 10
#define RARZ_SIZE RARZ_NUM * sizeof(uint32_t)

    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  1)) = REG32(sys, REG_RA);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  2)) = REG32(sys, REG_RB);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  3)) = REG32(sys, REG_RC);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  4)) = REG32(sys, REG_RD);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  5)) = REG32(sys, REG_RE);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  6)) = REG32(sys, REG_RF);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  7)) = REG32(sys, REG_RW);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  8)) = REG32(sys, REG_RX);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) *  9)) = REG32(sys, REG_RY);
    MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)-(sizeof(uint32_t) * 10)) = REG32(sys, REG_RZ);

    REG32(sys, REG_SP) -= RARZ_SIZE; // verify that this is correct and that it's not RARZ_SIZE+1

    TGX_PROFILE_END(PUSH_RARZ, 0x0041);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POP_RZRA):
    TGX_PROFILE_CALL(POP_RZRA, 0x0042);

    REG32(sys, REG_RZ) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 0));
    REG32(sys, REG_RY) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 1));
    REG32(sys, REG_RX) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 2));
    REG32(sys, REG_RW) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 3));
    REG32(sys, REG_RF) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 4));
    REG32(sys, REG_RE) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 5));
    REG32(sys, REG_RD) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 6));
    REG32(sys, REG_RC) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 7));
    REG32(sys, REG_RB) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 8));
    REG32(sys, REG_RA) = MEMACCESS(uint32_t, sys, REG32(sys, REG_SP)+(sizeof(uint32_t) * 9));

    REG32(sys, REG_SP) += RARZ_SIZE;

#undef RARZ_SIZE
#undef RARZ_NUM

    TGX_PROFILE_END(POP_RZRA, 0x0042);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVR32_TBL_R8_R8):
    TGX_PROFILE_CALL(MOVR32_TBL_R8_R8, 0x0043);

    memcpy(MEMPTR(void, sys, REG32(sys, instruction.params[0])),
        (sys->PU.int_table + REG8(sys, instruction.params[1])),
        REG8(sys, instruction.ext_params[0]));

    TGX_PROFILE_END(MOVR32_TBL_R8_R8, 0x0043);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVTBL_R8_R8_AR32):
    TGX_PROFILE_CALL(MOVTBL_R8_R8_AR32, 0x0044);

    memcpy(sys->PU.int_table + REG8(sys, instruction.params[0]),
        MEMPTR(void, sys, REG32(sys, instruction.params[1])),
        REG8(sys, instruction.ext_params[0]));

    TGX_PROFILE_END(MOVTBL_R8_R8_AR32, 0x0044);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR32_R32):
    TGX_PROFILE_CALL(ADDR32_R32, 0x0045);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) + REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(ADDR32_R32, 0x0045);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR8_R8):
    TGX_PROFILE_CALL(ADDR8_R8, 0x0046);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) + REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(ADDR8_R8, 0x0046);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR16_R16):
    TGX_PROFILE_CALL(ADDR16_R16, 0x0047);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) + REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(ADDR16_R16, 0x0047);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR64_R64):
    TGX_PROFILE_CALL(ADDR64_R64, 0x0048);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) + REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(ADDR64_R64, 0x0048);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR32_C32):
    TGX_PROFILE_CALL(ADDR32_C32, 0x0049);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) + instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(ADDR32_C32, 0x0049);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR8_C8):
    TGX_PROFILE_CALL(ADDR8_C8, 0x004A);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) + instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(ADDR8_C8, 0x004A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR16_C16):
    TGX_PROFILE_CALL(ADDR16_C16, 0x004B);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) + instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(ADDR16_C16, 0x004B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDR64_C40):
    TGX_PROFILE_CALL(ADDR64_C40, 0x004C);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) + CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(ADDR64_C40, 0x004C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR32_R32):
    TGX_PROFILE_CALL(SUBR32_R32, 0x004D);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) - REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(SUBR32_R32, 0x004D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR8_R8):
    TGX_PROFILE_CALL(SUBR8_R8, 0x004E);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) - REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(SUBR8_R8, 0x004E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR16_R16):
    TGX_PROFILE_CALL(SUBR16_R16, 0x004F);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) - REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(SUBR16_R16, 0x004F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR64_R64):
    TGX_PROFILE_CALL(SUBR64_R64, 0x0050);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) - REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(SUBR64_R64, 0x0050);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR32_C32):
    TGX_PROFILE_CALL(SUBR32_C32, 0x0051);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) - instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(SUBR32_C32, 0x0051);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR8_C8):
    TGX_PROFILE_CALL(SUBR8_C8, 0x0052);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) - instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(SUBR8_C8, 0x0052);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR16_C16):
    TGX_PROFILE_CALL(SUBR16_C16, 0x0053);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) - instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(SUBR16_C16, 0x0053);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBR64_C40):
    TGX_PROFILE_CALL(SUBR64_C40, 0x0054);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) - CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(SUBR64_C40, 0x0054);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR32_R32):
    TGX_PROFILE_CALL(MULR32_R32, 0x0055);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) * REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MULR32_R32, 0x0055);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR8_R8):
    TGX_PROFILE_CALL(MULR8_R8, 0x0056);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) * REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MULR8_R8, 0x0056);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR16_R16):
    TGX_PROFILE_CALL(MULR16_R16, 0x0057);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) * REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MULR16_R16, 0x0057);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR64_R64):
    TGX_PROFILE_CALL(MULR64_R64, 0x0058);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) * REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MULR64_R64, 0x0058);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR32_C32):
    TGX_PROFILE_CALL(MULR32_C32, 0x0059);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) * instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MULR32_C32, 0x0059);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR8_C8):
    TGX_PROFILE_CALL(MULR8_C8, 0x005A);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) * instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MULR8_C8, 0x005A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR16_C16):
    TGX_PROFILE_CALL(MULR16_C16, 0x005B);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) * instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MULR16_C16, 0x005B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULR64_C40):
    TGX_PROFILE_CALL(MULR64_C40, 0x005C);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) * CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MULR64_C40, 0x005C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR32_R32):
    TGX_PROFILE_CALL(DIVR32_R32, 0x005D);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) / REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(DIVR32_R32, 0x005D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR8_R8):
    TGX_PROFILE_CALL(DIVR8_R8, 0x005E);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) / REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(DIVR8_R8, 0x005E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR16_R16):
    TGX_PROFILE_CALL(DIVR16_R16, 0x005F);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) / REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(DIVR16_R16, 0x005F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR64_R64):
    TGX_PROFILE_CALL(DIVR64_R64, 0x0060);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) / REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(DIVR64_R64, 0x0060);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR32_C32):
    TGX_PROFILE_CALL(DIVR32_C32, 0x0061);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) / instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(DIVR32_C32, 0x0061);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR8_C8):
    TGX_PROFILE_CALL(DIVR8_C8, 0x0062);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) / instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(DIVR8_C8, 0x0062);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR16_C16):
    TGX_PROFILE_CALL(DIVR16_C16, 0x0063);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) / instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(DIVR16_C16, 0x0063);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVR64_C40):
    TGX_PROFILE_CALL(DIVR64_C40, 0x0064);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) / CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(DIVR64_C40, 0x0064);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR32_R32):
    TGX_PROFILE_CALL(MODR32_R32, 0x0065);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) % REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MODR32_R32, 0x0065);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR8_R8):
    TGX_PROFILE_CALL(MODR8_R8, 0x0066);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) % REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MODR8_R8, 0x0066);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR16_R16):
    TGX_PROFILE_CALL(MODR16_R16, 0x0067);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) % REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MODR16_R16, 0x0067);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR64_R64):
    TGX_PROFILE_CALL(MODR64_R64, 0x0068);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) % REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MODR64_R64, 0x0068);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR32_C32):
    TGX_PROFILE_CALL(MODR32_C32, 0x0069);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) % instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MODR32_C32, 0x0069);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR8_C8):
    TGX_PROFILE_CALL(MODR8_C8, 0x006A);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) % instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(MODR8_C8, 0x006A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR16_C16):
    TGX_PROFILE_CALL(MODR16_C16, 0x006B);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) % instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(MODR16_C16, 0x006B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODR64_C40):
    TGX_PROFILE_CALL(MODR64_C40, 0x006C);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) % CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(MODR64_C40, 0x006C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NEGR32):
    TGX_PROFILE_CALL(NEGR32, 0x006D);

    REG32(sys, instruction.params[0]) = -REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(NEGR32, 0x006D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NEGR8):
    TGX_PROFILE_CALL(NEGR8, 0x006E);

    REG8(sys, instruction.params[0]) = -REG8(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(NEGR8, 0x006E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NEGR16):
    TGX_PROFILE_CALL(NEGR16, 0x006F);

    REG16(sys, instruction.params[0]) = -REG16(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(NEGR16, 0x006F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NEGR64):
    TGX_PROFILE_CALL(NEGR64, 0x0070);

    REG64(sys, instruction.params[0]) = -REG64(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(NEGR64, 0x0070);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INCR32):
    TGX_PROFILE_CALL(INCR32, 0x0071);

    REG32(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(INCR32, 0x0071);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INCR8):
    TGX_PROFILE_CALL(INCR8, 0x0072);

    REG8(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(INCR8, 0x0072);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INCR16):
    TGX_PROFILE_CALL(INCR16, 0x0073);

    REG16(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(INCR16, 0x0073);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INCR64):
    TGX_PROFILE_CALL(INCR64, 0x0074);

    REG64(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(INCR64, 0x0074);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DECR32):
    TGX_PROFILE_CALL(DECR32, 0x0075);

    REG32(sys, instruction.params[0])--;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(DECR32, 0x0075);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DECR8):
    TGX_PROFILE_CALL(DECR8, 0x0076);

    REG8(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(DECR8, 0x0076);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DECR16):
    TGX_PROFILE_CALL(DECR16, 0x0077);

    REG16(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(DECR16, 0x0077);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DECR64):
    TGX_PROFILE_CALL(DECR64, 0x0078);

    REG64(sys, instruction.params[0])++;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(DECR64, 0x0078);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR32_R32):
    TGX_PROFILE_CALL(RBRR32_R32, 0x0079);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) >> REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(RBRR32_R32, 0x0079);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR8_R8):
    TGX_PROFILE_CALL(RBRR8_R8, 0x007A);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) >> REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(RBRR8_R8, 0x007A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR16_R16):
    TGX_PROFILE_CALL(RBRR16_R16, 0x007B);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) >> REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(RBRR16_R16, 0x007B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR64_R64):
    TGX_PROFILE_CALL(RBRR64_R64, 0x007C);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) >> REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(RBRR64_R64, 0x007C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR32_C32):
    TGX_PROFILE_CALL(RBRR32_C32, 0x007D);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) >> instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(RBRR32_C32, 0x007D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR8_C8):
    TGX_PROFILE_CALL(RBRR8_C8, 0x007E);

    REG8(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) >> instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(RBRR8_C8, 0x007E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR16_C16):
    TGX_PROFILE_CALL(RBRR16_C16, 0x007F);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) >> instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(RBRR16_C16, 0x007F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBRR64_C40):
    TGX_PROFILE_CALL(RBRR64_C40, 0x0080);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) >> CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(RBRR64_C40, 0x0080);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBL3R2_R32):
    TGX_PROFILE_CALL(RBL3R2_R32, 0x0081);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) << REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(RBL3R2_R32, 0x0081);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR8_R8):
    TGX_PROFILE_CALL(RBLR8_R8, 0x0082);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) << REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(RBLR8_R8, 0x0082);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR16_R16):
    TGX_PROFILE_CALL(RBLR16_R16, 0x0083);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) << REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(RBLR16_R16, 0x0083);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR64_R64):
    TGX_PROFILE_CALL(RBLR64_R64, 0x0084);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) << REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(RBLR64_R64, 0x0084);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR32_C32):
    TGX_PROFILE_CALL(RBLR32_C32, 0x0085);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) << instruction.const_i32;
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(RBLR32_C32, 0x0085);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR8_C8):
    TGX_PROFILE_CALL(RBLR8_C8, 0x0086);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) << instruction.ext_params[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(RBLR8_C8, 0x0086);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR16_C16):
    TGX_PROFILE_CALL(RBLR16_C16, 0x0087);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) << instruction.ext_params16[0];
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(RBLR16_C16, 0x0087);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(RBLR64_C40):
    TGX_PROFILE_CALL(RBLR64_C40, 0x0088);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) << CONST40(instruction);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(RBLR64_C40, 0x0088);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR32_R32):
    TGX_PROFILE_CALL(CMPR32_R32, 0x0089);

    ti64 = (int32_t)REG32(sys, instruction.params[1]) - (int32_t)REG32(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR32_R32, 0x0089);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR8_R8):
    TGX_PROFILE_CALL(CMPR8_R8, 0x008A);

    ti64 = (int8_t)REG8(sys, instruction.params[1]) - (int8_t)REG8(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR8_R8, 0x008A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR16_R16):
    TGX_PROFILE_CALL(CMPR16_R16, 0x008B);

    ti64 = (int16_t)REG16(sys, instruction.params[1]) - (int16_t)REG16(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR16_R16, 0x008B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR64_R64):
    TGX_PROFILE_CALL(CMPR64_R64, 0x008C);

    ti64 = (int64_t)REG64(sys, instruction.params[1]) - (int64_t)REG64(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR64_R64, 0x008C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR32_C32):
    TGX_PROFILE_CALL(CMPR32_C32, 0x008D);

    ti64 = (int32_t)instruction.const_i32 - (int32_t)REG32(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR32_C32, 0x008D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR8_C8):
    TGX_PROFILE_CALL(CMPR8_C8, 0x008E);

    ti64 = (int64_t)((int8_t)instruction.ext_params[0] - (int8_t)REG8(sys, instruction.params[0]));
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR8_C8, 0x008E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR16_C16):
    TGX_PROFILE_CALL(CMPR16_C16, 0x008F);

    ti64 = (int16_t)instruction.ext_params16[0] - (int16_t)REG16(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR16_C16, 0x008F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPR64_C40):
    TGX_PROFILE_CALL(CMPR64_C40, 0x0090);

    ti64 = (int64_t)CONST40(instruction) - (int64_t)REG64(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, ti64, 64);

    TGX_PROFILE_END(CMPR64_C40, 0x0090);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LANDR32_R32):
    TGX_PROFILE_CALL(LANDR32_R32, 0x0091);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) && REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(LANDR32_R32, 0x0091);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LANDR8_R8):
    TGX_PROFILE_CALL(LANDR8_R8, 0x0092);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) && REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(LANDR8_R8, 0x0092);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LANDR16_R16):
    TGX_PROFILE_CALL(LANDR16_R16, 0x0093);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) && REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(LANDR16_R16, 0x0093);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LANDR64_R64):
    TGX_PROFILE_CALL(LANDR64_R64, 0x0094);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) && REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(LANDR64_R64, 0x0094);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LORR32_R32):
    TGX_PROFILE_CALL(LORR32_R32, 0x0095);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) || REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(LORR32_R32, 0x0095);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LORR8_R8):
    TGX_PROFILE_CALL(LORR8_R8, 0x0096);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) || REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(LORR8_R8, 0x0096);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LORR16_R16):
    TGX_PROFILE_CALL(LORR16_R16, 0x0097);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) || REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(LORR16_R16, 0x0097);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LORR64_R64):
    TGX_PROFILE_CALL(LORR64_R64, 0x0098);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) || REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(LORR64_R64, 0x0098);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LNOTR32_R32):
    TGX_PROFILE_CALL(LNOTR32_R32, 0x0099);

    REG32(sys, instruction.params[0]) = !REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(LNOTR32_R32, 0x0099);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LNOTR8_R8):
    TGX_PROFILE_CALL(LNOTR8_R8, 0x009A);

    REG8(sys, instruction.params[0]) = !REG8(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(LNOTR8_R8, 0x009A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LNOTR16_R16):
    TGX_PROFILE_CALL(LNOTR16_R16, 0x009B);

    REG16(sys, instruction.params[0]) = !REG16(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(LNOTR16_R16, 0x009B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LNOTR64_R64):
    TGX_PROFILE_CALL(LNOTR64_R64, 0x009C);

    REG64(sys, instruction.params[0]) = !REG64(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(LNOTR64_R64, 0x009C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ANDR32_R32):
    TGX_PROFILE_CALL(ANDR32_R32, 0x009D);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) & REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(ANDR32_R32, 0x009D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ANDR8_R8):
    TGX_PROFILE_CALL(ANDR8_R8, 0x009E);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) & REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(ANDR8_R8, 0x009E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ANDR16_R16):
    TGX_PROFILE_CALL(ANDR16_R16, 0x009F);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) & REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(ANDR16_R16, 0x009F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ANDR64_R64):
    TGX_PROFILE_CALL(ANDR64_R64, 0x00A0);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) & REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(ANDR64_R64, 0x00A0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    /*NOP
    TGX_CASE(MOVR32_R32):
    TGX_PROFILE_CALL(MOVR32_R32, 0x00A1);
//#error "Not Implemented"
    TGX_PROFILE_END(MOVR32_R32, 0x00A1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);
    */

    TGX_CASE(XORR32_R32):
    TGX_PROFILE_CALL(XORR32_R32, 0x00A2);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) ^ REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(XORR32_R32, 0x00A2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(XORR8_R8):
    TGX_PROFILE_CALL(XORR8_R8, 0x00A3);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) ^ REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(XORR8_R8, 0x00A3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(XORR16_R16):
    TGX_PROFILE_CALL(XORR16_R16, 0x00A4);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) ^ REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(XORR16_R16, 0x00A4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(XORR64_R64):
    TGX_PROFILE_CALL(XORR64_R64, 0x00A5);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) ^ REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(XORR64_R64, 0x00A5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ORR32_R32):
    TGX_PROFILE_CALL(ORR32_R32, 0x00A6);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) | REG32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(ORR32_R32, 0x00A6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ORR8_R8):
    TGX_PROFILE_CALL(ORR8_R8, 0x00A7);

    REG8(sys, instruction.params[0]) = REG8(sys, instruction.params[1]) | REG8(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(ORR8_R8, 0x00A7);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ORR16_R16):
    TGX_PROFILE_CALL(ORR16_R16, 0x00A8);

    REG16(sys, instruction.params[0]) = REG16(sys, instruction.params[1]) | REG16(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(ORR16_R16, 0x00A8);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ORR64_R64):
    TGX_PROFILE_CALL(ORR64_R64, 0x00A9);

    REG64(sys, instruction.params[0]) = REG64(sys, instruction.params[1]) | REG64(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(ORR64_R64, 0x00A9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NOTR32_R32):
    TGX_PROFILE_CALL(NOTR32_R32, 0x00AA);

    REG32(sys, instruction.params[0]) = ~REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(NOTR32_R32, 0x00AA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NOTR8_R8):
    TGX_PROFILE_CALL(NOTR8_R8, 0x00AB);

    REG8(sys, instruction.params[0]) = ~REG8(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG8(sys, instruction.params[0]), 8);

    TGX_PROFILE_END(NOTR8_R8, 0x00AB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NOTR16_R16):
    TGX_PROFILE_CALL(NOTR16_R16, 0x00AC);

    REG16(sys, instruction.params[0]) = ~REG16(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG16(sys, instruction.params[0]), 16);

    TGX_PROFILE_END(NOTR16_R16, 0x00AC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NOTR64_R64):
    TGX_PROFILE_CALL(NOTR64_R64, 0x00AD);

    REG64(sys, instruction.params[0]) = ~REG64(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG64(sys, instruction.params[0]), 64);

    TGX_PROFILE_END(NOTR64_R64, 0x00AD);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MXB32):
    TGX_PROFILE_CALL(MXB32, 0x00AE);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1])
                                        + REG32(sys, instruction.ext_params[0]) * REG32(sys, instruction.ext_params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(MXB32, 0x00AE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MXBF32):
    TGX_PROFILE_CALL(MXBF32, 0x00AF);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1])
                                        + REGF32(sys, instruction.ext_params[0]) * REGF32(sys, instruction.ext_params[1]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MXBF32, 0x00AF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SQR32):
    TGX_PROFILE_CALL(SQR32, 0x00B0);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) * REG32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_I(sys, REG32(sys, instruction.params[0]), 32);

    TGX_PROFILE_END(SQR32, 0x00B0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SQR64):
    TGX_PROFILE_CALL(SQR64, 0x00B1);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]) * REG32(sys, instruction.params[1]);

    TGX_PROFILE_END(SQR64, 0x00B1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ABS8):
    TGX_PROFILE_CALL(ABS8, 0x00B2);

    REG8(sys, instruction.params[0]) = (uint8_t)abs((int8_t)REG8(sys, instruction.params[1]));

    TGX_PROFILE_END(ABS8, 0x00B2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ABS16):
    TGX_PROFILE_CALL(ABS16, 0x00B3);

    REG16(sys, instruction.params[0]) = (uint16_t)abs((int16_t)REG16(sys, instruction.params[1]));

    TGX_PROFILE_END(ABS16, 0x00B3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ABS32):
    TGX_PROFILE_CALL(ABS32, 0x00B4);

    REG32(sys, instruction.params[0]) = (uint32_t)abs((int32_t)REG32(sys, instruction.params[1]));

    TGX_PROFILE_END(ABS32, 0x00B4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ABS64):
    TGX_PROFILE_CALL(ABS64, 0x00B5);

    REG64(sys, instruction.params[0]) = (uint64_t)labs((int64_t)REG64(sys, instruction.params[1]));

    TGX_PROFILE_END(ABS64, 0x00B5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDF32):
    TGX_PROFILE_CALL(ADDF32, 0x00B6);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) + REGF32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ADDF32, 0x00B6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ADDFC32):
    TGX_PROFILE_CALL(ADDFC32, 0x00B7);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) + instruction.const_f32;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ADDFC32, 0x00B7);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBF32):
    TGX_PROFILE_CALL(SUBF32, 0x00B8);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) - REGF32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(SUBF32, 0x00B8);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SUBFC32):
    TGX_PROFILE_CALL(SUBFC32, 0x00B9);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) - instruction.const_f32;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(SUBFC32, 0x00B9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULF32):
    TGX_PROFILE_CALL(MULF32, 0x00BA);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) * REGF32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MULF32, 0x00BA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MULFC32):
    TGX_PROFILE_CALL(MULFC32, 0x00BB);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) * instruction.const_f32;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MULFC32, 0x00BB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVF32):
    TGX_PROFILE_CALL(DIVF32, 0x00BC);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) / REGF32(sys, instruction.ext_params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(DIVF32, 0x00BC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DIVFC32):
    TGX_PROFILE_CALL(DIVFC32, 0x00BD);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) / instruction.const_f32;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(DIVFC32, 0x00BD);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODF32):
    TGX_PROFILE_CALL(MODF32, 0x00BE);

    REGF32(sys, instruction.params[0]) = fmodf(REGF32(sys, instruction.params[1]), REGF32(sys, instruction.ext_params[0]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MODF32, 0x00BE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MODFC32):
    TGX_PROFILE_CALL(MODFC32, 0x00BF);

    REGF32(sys, instruction.params[0]) = fmodf(REGF32(sys, instruction.params[1]), instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(MODFC32, 0x00BF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(FLRF32):
    TGX_PROFILE_CALL(FLRF32, 0x00C0);

    REGF32(sys, instruction.params[0]) = floorf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(FLRF32, 0x00C0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CEILF32):
    TGX_PROFILE_CALL(CEILF32, 0x00C1);

    REGF32(sys, instruction.params[0]) = ceilf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(CEILF32, 0x00C1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ROUNDF32):
    TGX_PROFILE_CALL(ROUNDF32, 0x00C2);

    REGF32(sys, instruction.params[0]) = roundf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ROUNDF32, 0x00C2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SQRTF32):
    TGX_PROFILE_CALL(SQRTF32, 0x00C3);

    REGF32(sys, instruction.params[0]) = sqrtf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(SQRTF32, 0x00C3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPF32):
    TGX_PROFILE_CALL(CMPF32, 0x00C4);

    tf32 = REGF32(sys, instruction.params[1]) - REGF32(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, tf32);

    TGX_PROFILE_END(CMPF32, 0x00C4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CMPFC32):
    TGX_PROFILE_CALL(CMPFC32, 0x00C5);

    tf32 = instruction.const_f32 - REGF32(sys, instruction.params[0]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, tf32);

    TGX_PROFILE_END(CMPFC32, 0x00C5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(VEC4NORM):
    TGX_PROFILE_CALL(VEC4NORM, 0x00C6);

    // length
    tf32 = SQR(RVEC(sys, instruction.params[0]).xyzw[0]) + SQR(RVEC(sys, instruction.params[0]).xyzw[1])
            + SQR(RVEC(sys, instruction.params[0]).xyzw[2]) + SQR(RVEC(sys, instruction.params[0]).xyzw[3]);
    tf32 = 1.0f / sqrtf(tf32);
    RVEC(sys, instruction.params[0]).xyzw[0] *= tf32;
    RVEC(sys, instruction.params[0]).xyzw[1] *= tf32;
    RVEC(sys, instruction.params[0]).xyzw[2] *= tf32;
    RVEC(sys, instruction.params[0]).xyzw[3] *= tf32;

    TGX_PROFILE_END(VEC4NORM, 0x00C6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(VEC4ADD):
    TGX_PROFILE_CALL(VEC4ADD, 0x00C7);

    //__m128 a = _mm_load_ps((float*)RVEC(sys, instruction.params[1]).xyzw);
    //__m128 b = _mm_load_ps((float*)RVEC(sys, instruction.ext_params[0]).xyzw);
    //a = _mm_add_ps(a,  b);
    //_mm_store_ps((float*)RVEC(sys, instruction.params[0]).xyzw, a);

    RVEC(sys, instruction.params[0]).xyzw[0] += RVEC(sys, instruction.params[1]).xyzw[0];
    RVEC(sys, instruction.params[0]).xyzw[1] += RVEC(sys, instruction.params[1]).xyzw[1];
    RVEC(sys, instruction.params[0]).xyzw[2] += RVEC(sys, instruction.params[1]).xyzw[2];
    RVEC(sys, instruction.params[0]).xyzw[3] += RVEC(sys, instruction.params[1]).xyzw[3];

    TGX_PROFILE_END(VEC4ADD, 0x00C7);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(VEC4SUB):
    TGX_PROFILE_CALL(VEC4SUB, 0x00C8);

    RVEC(sys, instruction.params[0]).xyzw[0] -= RVEC(sys, instruction.params[1]).xyzw[0];
    RVEC(sys, instruction.params[0]).xyzw[1] -= RVEC(sys, instruction.params[1]).xyzw[1];
    RVEC(sys, instruction.params[0]).xyzw[2] -= RVEC(sys, instruction.params[1]).xyzw[2];
    RVEC(sys, instruction.params[0]).xyzw[3] -= RVEC(sys, instruction.params[1]).xyzw[3];

    TGX_PROFILE_END(VEC4SUB, 0x00C8);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(VEC4MUL):
    TGX_PROFILE_CALL(VEC4MUL, 0x00C9);

    RVEC(sys, instruction.params[0]).xyzw[0] *= RVEC(sys, instruction.params[1]).xyzw[0];
    RVEC(sys, instruction.params[0]).xyzw[1] *= RVEC(sys, instruction.params[1]).xyzw[1];
    RVEC(sys, instruction.params[0]).xyzw[2] *= RVEC(sys, instruction.params[1]).xyzw[2];
    RVEC(sys, instruction.params[0]).xyzw[3] *= RVEC(sys, instruction.params[1]).xyzw[3];

    TGX_PROFILE_END(VEC4MUL, 0x00C9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(VEC4DIV):
    TGX_PROFILE_CALL(VEC4DIV, 0x00CA);

    RVEC(sys, instruction.params[0]).xyzw[0] /= RVEC(sys, instruction.params[1]).xyzw[0];
    RVEC(sys, instruction.params[0]).xyzw[1] /= RVEC(sys, instruction.params[1]).xyzw[1];
    RVEC(sys, instruction.params[0]).xyzw[2] /= RVEC(sys, instruction.params[1]).xyzw[2];
    RVEC(sys, instruction.params[0]).xyzw[3] /= RVEC(sys, instruction.params[1]).xyzw[3];

    TGX_PROFILE_END(VEC4DIV, 0x00CA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DOT):
    TGX_PROFILE_CALL(DOT, 0x00CB);

    REGF32(sys, instruction.params[0]) = (RVEC(sys, instruction.params[1]).xyzw[0] * RVEC(sys, instruction.ext_params[0]).xyzw[0]) +
                                            (RVEC(sys, instruction.params[1]).xyzw[1] * RVEC(sys, instruction.ext_params[0]).xyzw[1]) +
                                            (RVEC(sys, instruction.params[1]).xyzw[2] * RVEC(sys, instruction.ext_params[0]).xyzw[2]) +
                                            (RVEC(sys, instruction.params[1]).xyzw[3] * RVEC(sys, instruction.ext_params[0]).xyzw[3]);

    TGX_PROFILE_END(DOT, 0x00CB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LEN):
    TGX_PROFILE_CALL(LEN, 0x00CC);

    REGF32(sys, instruction.params[0]) = sqrtf(SQR(RVEC(sys, instruction.params[1]).xyzw[0]) +
                                                 SQR(RVEC(sys, instruction.params[1]).xyzw[1]) +
                                                 SQR(RVEC(sys, instruction.params[1]).xyzw[2]) +
                                                 SQR(RVEC(sys, instruction.params[1]).xyzw[3]));

    TGX_PROFILE_END(LEN, 0x00CC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(COSF32):
    TGX_PROFILE_CALL(COSF32, 0x00CD);

    REGF32(sys, instruction.params[0]) = cosf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(COSF32, 0x00CD);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(COSFC32):
    TGX_PROFILE_CALL(COSFC32, 0x00CE);

    REGF32(sys, instruction.params[0]) = cosf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(COSFC32, 0x00CE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SINF32):
    TGX_PROFILE_CALL(SINF32, 0x00CF);

    REGF32(sys, instruction.params[0]) = sinf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(SINF32, 0x00CF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SINFC32):
    TGX_PROFILE_CALL(SINFC32, 0x00D0);

    REGF32(sys, instruction.params[0]) = cosf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(SINFC32, 0x00D0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(TANF32):
    TGX_PROFILE_CALL(TANF32, 0x00D1);

    REGF32(sys, instruction.params[0]) = tanf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(TANF32, 0x00D1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(TANFC32):
    TGX_PROFILE_CALL(TANFC32, 0x00D2);

    REGF32(sys, instruction.params[0]) = tanf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(TANFC32, 0x00D2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ACOSF32):
    TGX_PROFILE_CALL(ACOSF32, 0x00D3);

    REGF32(sys, instruction.params[0]) = acosf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ACOSF32, 0x00D3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ACOSFC32):
    TGX_PROFILE_CALL(ACOSFC32, 0x00D4);

    REGF32(sys, instruction.params[0]) = acosf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ACOSFC32, 0x00D4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ASINF32):
    TGX_PROFILE_CALL(ASINF32, 0x00D5);

    REGF32(sys, instruction.params[0]) = asinf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ASINF32, 0x00D5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ASINFC32):
    TGX_PROFILE_CALL(ASINFC32, 0x00D6);

    REGF32(sys, instruction.params[0]) = asinf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ASINFC32, 0x00D6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ATANF32):
    TGX_PROFILE_CALL(ATANF32, 0x00D7);

    REGF32(sys, instruction.params[0]) = atanf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ATANF32, 0x00D7);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ATANFC32):
    TGX_PROFILE_CALL(ATANFC32, 0x00D8);

    REGF32(sys, instruction.params[0]) = atanf(instruction.const_f32);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ATANFC32, 0x00D8);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ATAN2):
    TGX_PROFILE_CALL(ATAN2, 0x00D9);

    REGF32(sys, instruction.params[0]) = atan2f(REGF32(sys, instruction.params[1]), REGF32(sys, instruction.ext_params[0]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ATAN2, 0x00D9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ACOSBCD):
    TGX_PROFILE_CALL(ACOSBCD, 0x00DA);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) *
        cosf(REGF32(sys, instruction.ext_params[0]) * REGF32(sys, instruction.ext_params[1]) + REGF32(sys, instruction.ext_params[2]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ACOSBCD, 0x00DA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ASINBCD):
    TGX_PROFILE_CALL(ASINBCD, 0x00DB);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) *
            sinf(REGF32(sys, instruction.ext_params[0]) * REGF32(sys, instruction.ext_params[1]) + REGF32(sys, instruction.ext_params[2]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ASINBCD, 0x00DB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ATANBCD):
    TGX_PROFILE_CALL(ATANBCD, 0x00DC);

    REGF32(sys, instruction.params[0]) = REGF32(sys, instruction.params[1]) *
            tanf(REGF32(sys, instruction.ext_params[0]) * REGF32(sys, instruction.ext_params[1]) + REGF32(sys, instruction.ext_params[2]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ATANBCD, 0x00DC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(NEGF32):
    TGX_PROFILE_CALL(NEGF32, 0x00DD);

    REGF32(sys, instruction.params[0]) = -REGF32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(NEGF32, 0x00DD);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SWZL):
    TGX_PROFILE_CALL(SWZL, 0x00DE);

    t16 /*mask*/ = REG16(sys, instruction.ext_params[0]);

    RVEC(sys, instruction.params[0]).xyzw[0] =  (RVEC(sys, instruction.params[1]).xyzw[0] * (float)!!(t16 & 0b1000000000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[1] * (float)!!(t16 & 0b0100000000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[2] * (float)!!(t16 & 0b0010000000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[3] * (float)!!(t16 & 0b0001000000000000));

    RVEC(sys, instruction.params[0]).xyzw[1] =  (RVEC(sys, instruction.params[1]).xyzw[0] * (float)!!(t16 & 0b0000100000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[1] * (float)!!(t16 & 0b0000010000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[2] * (float)!!(t16 & 0b0000001000000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[3] * (float)!!(t16 & 0b0000000100000000));

    RVEC(sys, instruction.params[0]).xyzw[2] =  (RVEC(sys, instruction.params[1]).xyzw[0] * (float)!!(t16 & 0b0000000010000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[1] * (float)!!(t16 & 0b0000000001000000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[2] * (float)!!(t16 & 0b0000000000100000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[3] * (float)!!(t16 & 0b0000000000010000));

    RVEC(sys, instruction.params[0]).xyzw[3] =  (RVEC(sys, instruction.params[1]).xyzw[0] * (float)!!(t16 & 0b0000000000001000)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[1] * (float)!!(t16 & 0b0000000000000100)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[2] * (float)!!(t16 & 0b0000000000000010)) +
                                                (RVEC(sys, instruction.params[1]).xyzw[3] * (float)!!(t16 & 0b0000000000000001));

    TGX_PROFILE_END(SWZL, 0x00DE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(POWF32):
    TGX_PROFILE_CALL(POWF32, 0x00DF);

    REGF32(sys, instruction.params[0]) = powf(REGF32(sys, instruction.params[1]), REGF32(sys, instruction.ext_params[0]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(POWF32, 0x00DF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LOGF32):
    TGX_PROFILE_CALL(LOGF32, 0x00E0);

    REGF32(sys, instruction.params[0]) = log10f(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(LOGF32, 0x00E0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LNF32):
    TGX_PROFILE_CALL(LNF32, 0x00E1);

    REGF32(sys, instruction.params[0]) = logf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(LNF32, 0x00E1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(EF32):
    TGX_PROFILE_CALL(EF32, 0x00E2);

    REGF32(sys, instruction.params[0]) = powf(M_E, REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(EF32, 0x00E2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LDPI):
    TGX_PROFILE_CALL(LDPI, 0x00E3);

    REGF32(sys, instruction.params[0]) = M_PI;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(LDPI, 0x00E3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(LDE):
    TGX_PROFILE_CALL(LDE, 0x00E4);

    REGF32(sys, instruction.params[0]) = M_E;
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(LDE, 0x00E4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INVERSE):
    TGX_PROFILE_CALL(INVERSE, 0x00E5);

    REGF32(sys, instruction.params[0]) = 1.0f / REGF32(sys, instruction.params[1]);
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(INVERSE, 0x00E5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(ABSF32):
    TGX_PROFILE_CALL(ABSF32, 0x00E6);

    REGF32(sys, instruction.params[0]) = fabsf(REGF32(sys, instruction.params[1]));
    UPDATE_PF_ZERO_NEG_BITS_F(sys, REGF32(sys, instruction.params[0]));

    TGX_PROFILE_END(ABSF32, 0x00E6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JMPPR32):
    TGX_PROFILE_CALL(JMPPR32, 0x00E7);

    // load address of next instruction in JR
    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);

    TGX_PROFILE_END(JMPPR32, 0x00E7);
    TGX_DISPATCH(_jTable, instruction, *sys);


    TGX_CASE(JMPC32):
    TGX_PROFILE_CALL(JMPC32, 0x00E8);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    REG32(sys, REG_IP) = instruction.const_i32;

    TGX_PROFILE_END(JMPC32, 0x00E8);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JZR32):
    TGX_PROFILE_CALL(JZR32, 0x00E9);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    /*
    TGX_PROFILE_END(JZR32, 0x00E9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);
    */



    TGX_CASE(JZC32):
    TGX_PROFILE_CALL(JZC32, 0x00EA);

    if (REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JZC32, 0x00EA);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JZC32, 0x00EA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JNZR32):
    TGX_PROFILE_CALL(JNZR32, 0x00EB);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (!(REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit)) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    /*
    TGX_PROFILE_END(JNZR32, 0x00EB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);*/



    TGX_CASE(JNZC32):
    TGX_PROFILE_CALL(JNZC32, 0x00EC);

    if (!(REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit)) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JZC32, 0x00EA);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JNZC32, 0x00EC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JZDR32):
    TGX_PROFILE_CALL(JZDR32, 0x00ED);

    if (REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JZDR32, 0x00ED);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JNZDR32):
    TGX_PROFILE_CALL(JNZDR32, 0x00EE);

    if (!(REG32(sys, REG_PF) & TGX_PF_Flag_ZeroBit)) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JZR32, 0x00E9);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JNZDR32, 0x00EE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JGTR32):
    TGX_PROFILE_CALL(JGTR32, 0x00EF);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (!(REG32(sys, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit))) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else if (instruction.params[1] & TGX_JMP_ELSE_FLAG) {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JGTR32, 0x00EF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JGTC32):
    TGX_PROFILE_CALL(JGTC32, 0x00F0);

    if (!(REG32(sys, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit))) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JGTC32, 0x00F0);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JGTC32, 0x00F0);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JLTR32):
    TGX_PROFILE_CALL(JLTR32, 0x00F1);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (REG32(sys, REG_PF) & TGX_PF_Flag_NegBit) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else if (instruction.params[1] & TGX_JMP_ELSE_FLAG) {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JLTR32, 0x00F1);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JLTC32):
    TGX_PROFILE_CALL(JLTC32, 0x00F2);

    if ((REG32(sys, REG_PF) & TGX_PF_Flag_NegBit)) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JGTC32, 0x00F0);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JLTC32, 0x00F2);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JGER32):
    TGX_PROFILE_CALL(JGER32, 0x00F3);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (!(REG32(sys, REG_PF) & (TGX_PF_Flag_NegBit))) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else if (instruction.params[1] & TGX_JMP_ELSE_FLAG) {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JGER32, 0x00F3);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JGEC32):
    TGX_PROFILE_CALL(JGEC32, 0x00F4);

    if (!(REG32(sys, REG_PF) & TGX_PF_Flag_NegBit)) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JGEC32, 0x00F4);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JGEC32, 0x00F4);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JLER32):
    TGX_PROFILE_CALL(JLER32, 0x00F5);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    if (REG32(sys, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit)) {
        REG32(sys, REG_IP) = REG32(sys, instruction.params[0]);
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }
    else if (instruction.params[1] & TGX_JMP_ELSE_FLAG) {
        t32 = instruction.const_i32;
        if (instruction.params[1] & TGX_JMP_USE_REG_FLAG) {
            t32 = REG32(sys, t32);
        }
        REG32(sys, REG_IP) = t32;
        TGX_PROFILE_END(JGTR32, 0x00EF);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JLER32, 0x00F5);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JLEC32):
    TGX_PROFILE_CALL(JLEC32, 0x00F6);

    if (REG32(sys, REG_PF) & (TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit)) {
        REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
        REG32(sys, REG_IP) = instruction.const_i32;
        TGX_PROFILE_END(JGEC32, 0x00F4);
        TGX_DISPATCH(_jTable, instruction, *sys);
    }

    TGX_PROFILE_END(JLEC32, 0x00F6);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


/* NOP
    TGX_CASE(JNER32):
    TGX_PROFILE_CALL(JNER32, 0x00F7);
//#error "Not Implemented"
    TGX_PROFILE_END(JNER32, 0x00F7);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(JNEC32):
    TGX_PROFILE_CALL(JNEC32, 0x00F8);
//#error "Not Implemented"
    TGX_PROFILE_END(JNEC32, 0x00F8);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);
*/

/* NOP
    TGX_CASE(CALLR32):
    TGX_PROFILE_CALL(CALLR32, 0x00F9);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLR32, 0x00F9);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLC32):
    TGX_PROFILE_CALL(CALLC32, 0x00FA);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLC32, 0x00FA);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLZR32):
    TGX_PROFILE_CALL(CALLZR32, 0x00FB);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLZR32, 0x00FB);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLZC32):
    TGX_PROFILE_CALL(CALLZC32, 0x00FC);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLZC32, 0x00FC);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLNZR32):
    TGX_PROFILE_CALL(CALLNZR32, 0x00FD);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLNZR32, 0x00FD);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLNZC32):
    TGX_PROFILE_CALL(CALLNZC32, 0x00FE);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLNZC32, 0x00FE);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLEQR32):
    TGX_PROFILE_CALL(CALLEQR32, 0x00FF);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLEQR32, 0x00FF);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLEQC32):
    TGX_PROFILE_CALL(CALLEQC32, 0x0100);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLEQC32, 0x0100);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLGTR32):
    TGX_PROFILE_CALL(CALLGTR32, 0x0101);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLGTR32, 0x0101);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLGTC32):
    TGX_PROFILE_CALL(CALLGTC32, 0x0102);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLGTC32, 0x0102);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLLTR32):
    TGX_PROFILE_CALL(CALLLTR32, 0x0103);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLLTR32, 0x0103);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLLTC32):
    TGX_PROFILE_CALL(CALLLTC32, 0x0104);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLLTC32, 0x0104);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLGER32):
    TGX_PROFILE_CALL(CALLGER32, 0x0105);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLGER32, 0x0105);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLGEC32):
    TGX_PROFILE_CALL(CALLGEC32, 0x0106);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLGEC32, 0x0106);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLLER32):
    TGX_PROFILE_CALL(CALLLER32, 0x0107);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLLER32, 0x0107);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLLEC32):
    TGX_PROFILE_CALL(CALLLEC32, 0x0108);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLLEC32, 0x0108);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLNER32):
    TGX_PROFILE_CALL(CALLNER32, 0x0109);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLNER32, 0x0109);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(CALLNEC32):
    TGX_PROFILE_CALL(CALLNEC32, 0x010A);
//#error "Not Implemented"
    TGX_PROFILE_END(CALLNEC32, 0x010A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);
*/


    TGX_CASE(RET):
    TGX_PROFILE_CALL(RET, 0x010B);

    REG32(sys, REG_IP) = REG32(sys, REG_JR);

    TGX_PROFILE_END(RET, 0x010B);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(HLT):
    TGX_PROFILE_CALL(HLT, 0x010C);
    TGX_PROFILE_END(HLT, 0x010C);
    goto PROGRAM_HALT;


    TGX_CASE(INTR8):
    TGX_PROFILE_CALL(INTR8, 0x010D);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    REG32(sys, REG_IP) = sys->PU.int_table[REG8(sys, instruction.params[0])];

    TGX_PROFILE_END(INTR8, 0x010D);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INTC8):
    TGX_PROFILE_CALL(INTC8, 0x010E);

    REG32(sys, REG_JR) = REG32(sys, REG_IP) + sizeof(Instruction);
    REG32(sys, REG_IP) = sys->PU.int_table[instruction.const_i32];

    TGX_PROFILE_END(INTC8, 0x010E);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INITEXR32):
    TGX_PROFILE_CALL(INITEXR32, 0x010F);
//#error "Not Implemented"
    TGX_PROFILE_END(INITEXR32, 0x010F);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INITEXC32):
    TGX_PROFILE_CALL(INITEXC32, 0x0110);
//#error "Not Implemented"
    TGX_PROFILE_END(INITEXC32, 0x0110);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INVKEXR32):
    TGX_PROFILE_CALL(INVKEXR32, 0x0111);
//#error "Not Implemented"
    TGX_PROFILE_END(INVKEXR32, 0x0111);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(INVKEXC32):
    TGX_PROFILE_CALL(INVKEXC32, 0x0112);
//#error "Not Implemented"
    TGX_PROFILE_END(INVKEXC32, 0x0112);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DEINITEXR32):
    TGX_PROFILE_CALL(DEINITEXR32, 0x0113);
//#error "Not Implemented"
    TGX_PROFILE_END(DEINITEXR32, 0x0113);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(DEINITEXC32):
    TGX_PROFILE_CALL(DEINITEXC32, 0x0114);
//#error "Not Implemented"
    TGX_PROFILE_END(DEINITEXC32, 0x0114);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(SYSCALLC32):
    TGX_PROFILE_CALL(SYSCALLC32, 0x0115);

    REG32(sys, REG_RA) = sys->SWIFunc(sys, instruction.const_i32);

    TGX_PROFILE_END(SYSCALLC32, 0x0115);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(BREAK):
    TGX_PROFILE_CALL(BREAK, 0x0116);

    LaunchDebugShell(sys);

    TGX_PROFILE_END(BREAK, 0x0116);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQPS):
    TGX_PROFILE_CALL(GQPS, 0x0117);
//#error "Not Implemented"
    TGX_PROFILE_END(GQPS, 0x0117);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQS):
    TGX_PROFILE_CALL(GQS, 0x0118);
//#error "Not Implemented"
    TGX_PROFILE_END(GQS, 0x0118);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQAI):
    TGX_PROFILE_CALL(GQAI, 0x0119);
//#error "Not Implemented"
    TGX_PROFILE_END(GQAI, 0x0119);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQR):
    TGX_PROFILE_CALL(GQR, 0x011A);
//#error "Not Implemented"
    TGX_PROFILE_END(GQR, 0x011A);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQSI):
    TGX_PROFILE_CALL(GQSI, 0x011B);
//#error "Not Implemented"
    TGX_PROFILE_END(GQSI, 0x011B);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQPC):
    TGX_PROFILE_CALL(GQPC, 0x011C);
//#error "Not Implemented"
    TGX_PROFILE_END(GQPC, 0x011C);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(GQPF):
    TGX_PROFILE_CALL(GQPF, 0x011D);
//#error "Not Implemented"
    TGX_PROFILE_END(GQPF, 0x011D);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);



    TGX_CASE(MOVRR32):
    TGX_PROFILE_CALL(MOVRR32, 0x011E);

    REG32(sys, instruction.params[0]) = REG32(sys, instruction.params[1]);

    TGX_PROFILE_END(MOVRR32, 0x011E);
    TGX_NEXT_INSTR(*sys);
    TGX_DISPATCH(_jTable, instruction, *sys);


PROGRAM_HALT:
    TGX_PROFILE_REPORT("ProfilerData.txt");

    return TGX_SUCCESS;
}

enum COMMANDS {
    Cmd_Unknown,
    Cmd_Help,
    Cmd_Quit,
    Cmd_PrintReg,
    Cmd_PeekStack,
    Cmd_PeekMem,
    Cmd_PrintLastInst,
    Cmd_PrintNextInst,
    Cmd_Push8,
    Cmd_Push16,
    Cmd_Push32,
    Cmd_Push64,
    Cmd_Pop8,
    Cmd_Pop16,
    Cmd_Pop32,
    Cmd_Pop64,
    Cmd_SetReg,
    Cmd_Kill,
    Cmd_Step,
    Cmd_StepOvr,
    Cmd_Break,
    Cmd_Restart,
};

enum REGS {
    UNDEF, RA, RB, RC, RD, RE, RF, RX, RY, RZ, RW, RI, RJ, RK, RL, IP, JR, SB, SP, SH, PF,
    M0, M1, X0, Y0, Z0, W0, X1, Y1, Z1, W1, X2, Y2, Z2, W2, X3, Y3, Z3, W3,
};

static void DebugShellInit(void) {
    printf("TGX Serial Debug Shell V0.1\n");
    printf("type 'help' for list of commands\n");
    printf("type 'cont' to resume execution.\n");
    printf("--------------------------------\n");
}

static void ShellHelp(void) {
    printf("help               Displays Command List\n");
    printf("cont               Resumes Execution\n");
    printf("kill               Kills Machine Execution\n");
    printf("print [reg]        Displays Contents of Register\n");
    printf("set   [reg] [val]  Sets the register to the value\n");
    printf("next               Shows the opcode of the next instruction\n");
    printf("last               Shows the opcode of the previous instruction\n");
    printf("peek [addr]        Shows the contents of memory at the address\n");
    printf("head               Shows the contents at the top of the stack\n");
    printf("step               Steps forward by one instruction\n");
    printf("stepovr            Steps forward skipping over function calls\n");
    printf("break [line]       Attempts to set a breakpoint for the line\n");
    printf("restart            Restarts the machine\n");
}

static uint32_t s_TmpBreak = {0};
static bool s_IsTmp = {0};
static int s_FixedTmpCount = 0;
static uint32_t s_FixedTmp[1024] = {0};

static void SkipSpaces(const char** ptr) {
    while (**ptr && **ptr == ' ') (*ptr)++;
}

static int ParseCommand(const char** ptr) {
    SkipSpaces(ptr);
    if (!**ptr) {
        return Cmd_Unknown;
    }

    if (strncmp(*ptr, "help", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_Help; }
    if (strncmp(*ptr, "cont", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_Quit; }
    if (strncmp(*ptr, "kill", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_Kill; }
    if (strncmp(*ptr, "print", 5) == 0 && !isalnum((*ptr)[5])){ (*ptr)+=5; return Cmd_PrintReg; }
    if (strncmp(*ptr, "set",  3) == 0 && !isalnum((*ptr)[3])){ (*ptr)+=3; return Cmd_SetReg; }
    if (strncmp(*ptr, "next", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_PrintNextInst; }
    if (strncmp(*ptr, "last", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_PrintLastInst; }
    if (strncmp(*ptr, "peek", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_PeekMem; }
    if (strncmp(*ptr, "head", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_PeekStack; }
    if (strncmp(*ptr, "step", 4) == 0 && !isalnum((*ptr)[4])){ (*ptr)+=4; return Cmd_Step; }
    if (strncmp(*ptr, "stepovr", 7) == 0 && !isalnum((*ptr)[7])){ (*ptr)+=7; return Cmd_StepOvr; }
    if (strncmp(*ptr, "break", 5) == 0 && !isalnum((*ptr)[5])){ (*ptr)+=5; return Cmd_Break; }
    if (strncmp(*ptr, "restart", 7) == 0 && !isalnum((*ptr)[7])){ (*ptr)+=7; return Cmd_Restart; }

    return Cmd_Unknown;
}
static int ParseReg(const char** ptr) {
    SkipSpaces(ptr);

    if (!**ptr) {
        return UNDEF;
    }

    char a, b, c;
    a = (*ptr)[0];
    b = (*ptr)[1];
    c = (*ptr)[2];

    if (isalnum(c)) {
        return UNDEF;
    }

    if (a == 'r' && b == 'a') return RA;
    if (a == 'r' && b == 'b') return RB;
    if (a == 'r' && b == 'c') return RC;
    if (a == 'r' && b == 'd') return RD;
    if (a == 'r' && b == 'e') return RE;
    if (a == 'r' && b == 'f') return RF;
    if (a == 'r' && b == 'x') return RX;
    if (a == 'r' && b == 'y') return RY;
    if (a == 'r' && b == 'z') return RZ;
    if (a == 'r' && b == 'w') return RW;
    if (a == 'r' && b == 'i') return RI;
    if (a == 'r' && b == 'j') return RJ;
    if (a == 'r' && b == 'k') return RK;
    if (a == 'r' && b == 'l') return RL;
    if (a == 'i' && b == 'p') return IP;
    if (a == 'p' && b == 'f') return PF;
    if (a == 'j' && b == 'r') return JR;
    if (a == 's' && b == 'b') return SB;
    if (a == 's' && b == 'h') return SH;
    if (a == 's' && b == 'p') return SP;
    if (a == 'm' && b == '0') return M0;
    if (a == 'm' && b == '1') return M0;
    if (a == 'x' && b == '0') return X0;
    if (a == 'x' && b == '1') return X1;
    if (a == 'x' && b == '2') return X2;
    if (a == 'x' && b == '3') return X3;
    if (a == 'y' && b == '0') return Y0;
    if (a == 'y' && b == '1') return Y1;
    if (a == 'y' && b == '2') return Y2;
    if (a == 'y' && b == '3') return Y3;
    if (a == 'z' && b == '0') return Z0;
    if (a == 'z' && b == '1') return Z1;
    if (a == 'z' && b == '2') return Z2;
    if (a == 'z' && b == '3') return Z3;
    if (a == 'w' && b == '0') return W0;
    if (a == 'w' && b == '1') return W1;
    if (a == 'w' && b == '2') return W2;
    if (a == 'w' && b == '3') return W3;

    return UNDEF;
}

uint64_t GetReg(TGXContext* ctx, int reg, int* is_float) {
    switch (reg) {
        default:
            return 0;
        case RA:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RA);
        case RB:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RB);
        case RC:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RC);
        case RD:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RD);
        case RE:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RE);
        case RF:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RF);
        case RX:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RX);
        case RY:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RY);
        case RZ:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RZ);
        case RW:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RW);
        case RI:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RI);
        case RJ:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RJ);
        case RK:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RK);
        case RL:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_RL);
        case IP:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_IP);
        case JR:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_JR);
        case SB:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_SB);
        case SH:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_SH);
        case SP:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_SP);
        case PF:
            if (is_float) *is_float = false;
            return REG32(ctx, REG_PF);
        case M0:
            if (is_float) *is_float = false;
            return REG64(ctx, 0);
        case M1:
            if (is_float) *is_float = false;
            return REG64(ctx, 1);
        case X0:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FX0);
        case Y0:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FY0);
        case Z0:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FZ0);
        case W0:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FW0);
        case X1:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FX1);
        case Y1:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FY1);
        case Z1:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FZ1);
        case W1:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FW1);
        case X2:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FX2);
        case Y2:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FY2);
        case Z2:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FZ2);
        case W2:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FW2);
        case X3:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FX3);
        case Y3:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FY3);
        case Z3:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FZ3);
        case W3:
            if (is_float) *is_float = true;
            return *(uint32_t*)&REGF32(ctx, REG_FW3);
    }

}

static double ParseVal(const char** ptr) {
    SkipSpaces(ptr);

    char *end;
    const char *beg = *ptr;
    if (beg[0] == '0' && (beg[1] == 'x' || beg[1] == 'X')) {
        long value = strtol(beg, &end, 16);
        if (end != *ptr) {
            return (double)value;
        }
    }

    double value = strtod(*ptr, &end);

    if (end == *ptr) {
        return NAN;
    }

    return value;
}

#define INCLUDE_DATA_TABLE
#include "optable.inc.c"

#define PARAM_TYPE_MASK 0b1111
#define PARAM_MASK_WIDTH 4
#define PARAM_TYPE_None 0
#define PARAM_TYPE_R32 1
#define PARAM_TYPE_R16 2
#define PARAM_TYPE_R8 3
#define PARAM_TYPE_R64 4
#define PARAM_TYPE_R64H 5
#define PARAM_TYPE_R64L 6
#define PARAM_TYPE_F32 7
#define PARAM_TYPE_RVec 8
#define PARAM_TYPE_Table 9
#define PARAM_TYPE_DeR32 10
#define PARAM_TYPE_Imm 11
#define PARAM_TYPE_Size8 12
#define PARAM_TYPE_Size16 13
#define PARAM_TYPE_Size32 14
#define PARAM_TYPE_Size64 15

static const char* Reg32_Names[] = {
    "ra", "rb", "rc", "rd", "re", "rf", "rw", "rx", "ry", "rz",
    "ip", "jr", "sb", "sp", "sh", "pf", "ri", "rj", "rk", "rl"
};
static const char* Reg16_Names[] = {
    "ral", "rah", "rbl", "rbh", "rcl", "rch", "rdl", "rdh",
    "rel", "reh", "rfl", "rfh", "rwl", "rwh", "rxl", "rxh",
    "ryl", "ryh", "rzl", "rzh"
};
static const char* Reg8_Names[] = {
    "ras", "rbs", "rcs", "rds", "res", "rfs", "rws", "rxs", "rys", "rzs",
};
static const char* Reg64_Names[] = { "m0", "m1" };
static const char* Reg64H_Names[] = { "m0h", "m1h" };
static const char* Reg64L_Names[] = { "m0l", "m1l" };
static const char* RegVec_Names[] = { "ve0", "ve1", "ve2", "ve3" };
static const char* RegF32_Names[] = {
    "fx0", "fy0", "fz0", "fw0",
    "fx1", "fy1", "fz1", "fw1",
    "fx2", "fy2", "fz2", "fw2",
    "fx3", "fy3", "fz3", "fw3",
};

static void PrintParam(uint32_t type, Instruction* inst, int index) {
    switch (type) {
        default:
        case PARAM_TYPE_None:
            return;
        case PARAM_TYPE_R32:
            if (index < 2) {
                printf("%s ", Reg32_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg32_Names[inst->ext_params[index - 2]]);
            }
            return;
        case PARAM_TYPE_R16:
            if (index < 2) {
                printf("%s ", Reg16_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg16_Names[inst->ext_params[index - 2]]);
            }
            return;
        case PARAM_TYPE_R8:
            if (index < 2) {
                printf("%s ", Reg8_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg8_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_R64:
            if (index < 2) {
                printf("%s ", Reg64_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg64_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_R64H:
            if (index < 2) {
                printf("%s ", Reg64H_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg64H_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_R64L:
            if (index < 2) {
                printf("%s ", Reg64L_Names[inst->params[index]]);
            }
            else {
                printf("%s ", Reg64L_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_F32:
            if (index < 2) {
                printf("%s ", RegF32_Names[inst->params[index]]);
            }
            else {
                printf("%s ", RegF32_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_Table:
            printf("tbl ");
            return;
        case PARAM_TYPE_RVec:
            if (index < 2) {
                printf("%s ", RegVec_Names[inst->params[index]]);
            }
            else {
                printf("%s ", RegVec_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_DeR32:
            if (index < 2) {
                printf("[%s] ", Reg32_Names[inst->params[index]]);
            }
            else {
                printf("[%s] ", Reg32_Names[inst->ext_params[index - 2]]);
            }
        return;
        case PARAM_TYPE_Imm:
            printf("%d (0x%08X | %f)", inst->const_i32, inst->const_i32, inst->const_f32);
            return;
        case PARAM_TYPE_Size8:
            printf("i8 ");
            return;
        case PARAM_TYPE_Size16:
            printf("i16 ");
            return;
        case PARAM_TYPE_Size32:
            printf("i32 ");
            return;
        case PARAM_TYPE_Size64:
            printf("i64 ");
            return;
    }
}

static void PrintParamIDs(Mnemonic *spec, Instruction* inst) {
    uint32_t params[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t *cursor = params+7;
    uint32_t par = spec->param_id;
    while ((par & PARAM_TYPE_MASK) != PARAM_TYPE_None) {
        *cursor = par & PARAM_TYPE_MASK;
        par >>= PARAM_MASK_WIDTH;
        cursor--;
        if (cursor < params) {
            printf("Invalid ParamID! Too many parameters detected.\n");
            return;
        }
    }
    cursor++;

    uint32_t *beg = params;
    while (cursor < params+8) {
        *beg = *cursor;
        beg++; cursor++;
    }
    while (beg < cursor) {
        *beg = 0; beg++;
    }

    beg = params;
    int index = 0;
    while (*beg) {
        PrintParam(*beg, inst, index);
        index++;
        beg++;
    }
    printf("\n");
}

static void PrintInst(Instruction* inst) {
    if ((inst-1)->opcode == 0x0000) {
        printf("%4u: ", (inst-1)->const_i32);
    }

    Mnemonic *op = SearchOpCode(inst->opcode);
    uint32_t opcode = inst->opcode;

    if (op == NULL) {
        uint32_t param0 = inst->params[0];
        uint32_t param1 = inst->params[1];
        printf("[%04X] ??? %02X %02X %08X (%f)\n", opcode, param0, param1, inst->const_i32, inst->const_f32);
        return;
    }

    printf("[%04X] %s ", opcode, op->name);
    PrintParamIDs(op, inst);
}

static void StepNext(TGXContext* ctx, Instruction* currentInst, int step_into) {
    uint32_t ip = REG32(ctx, REG_IP);

    //Instruction* nextInstr = currentInst + 1;

    ip += sizeof(Instruction) * 2;
    currentInst += 2;
    if (!s_IsTmp) {
        currentInst++;
        ip += sizeof(Instruction);
        //nextInstr++;
    }

    Instruction* nextInstr = currentInst - 1;

    switch (nextInstr->opcode) {
        case 0x00E7:
            ip = REG32(ctx, nextInstr->params[0]) ;
            currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
            nextInstr = currentInst + 1;
            break;
        case 0x00E8:
            ip = nextInstr->const_i32 ;
            currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
            nextInstr = currentInst + 1;
            break;
        case 0x00E9:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            ip = nextInstr->const_i32;
            if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                ip = REG32(ctx, ip);
            }
            ip -= sizeof(Instruction);
            currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
            nextInstr = currentInst + 1;
            break;
        case 0x00EA:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x00EB:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit)) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            ip = nextInstr->const_i32;
            if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                ip = REG32(ctx, ip);
            }
            ip -= sizeof(Instruction);
            currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
            nextInstr = currentInst + 1;
            break;
        case 0x00EC:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit)) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00ED:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00EE:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit)) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00EF:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit))) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            else if (nextInstr->params[1] & TGX_JMP_ELSE_FLAG) {
                ip = nextInstr->const_i32;
                if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                    ip = REG32(ctx, ip);
                }
                ip -= sizeof(Instruction);
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00F0:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit))) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00F1:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & TGX_PF_Flag_NegBit) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            if (nextInstr->params[1] & TGX_JMP_ELSE_FLAG) {
                ip = nextInstr->const_i32;
                if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                    ip = REG32(ctx, ip);
                }
                ip -= sizeof(Instruction);
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
            }
            break;
        case 0x00F2:
            if (!step_into) break;
            if ((REG32(ctx, REG_PF) & TGX_PF_Flag_NegBit)) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x00F3:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & (TGX_PF_Flag_NegBit))) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            if (nextInstr->params[1] & TGX_JMP_ELSE_FLAG) {
                ip = nextInstr->const_i32;
                if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                    ip = REG32(ctx, ip);
                }
                ip -= sizeof(Instruction);
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x00F4:
            if (!step_into) break;
            if (!(REG32(ctx, REG_PF) & TGX_PF_Flag_NegBit)) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x00F5:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & (TGX_PF_Flag_NegBit | TGX_PF_Flag_ZeroBit)) {
                ip = REG32(ctx, nextInstr->params[0]) ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            if (nextInstr->params[1] & TGX_JMP_ELSE_FLAG) {
                ip = nextInstr->const_i32;
                if (nextInstr->params[1] & TGX_JMP_USE_REG_FLAG) {
                    ip = REG32(ctx, ip);
                }
                ip -= sizeof(Instruction);
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x00F6:
            if (!step_into) break;
            if (REG32(ctx, REG_PF) & (TGX_PF_Flag_ZeroBit | TGX_PF_Flag_NegBit)) {
                ip = nextInstr->const_i32 ;
                currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
                nextInstr = currentInst + 1;
                break;
            }
            break;
        case 0x010B: // ret
            ip = REG32(ctx, REG_JR) ;
            currentInst = (Instruction*)(ctx->Memory.memory_begin + ip);
            nextInstr = currentInst + 1;
            break;
        default: break;
    }

    PrintInst(nextInstr);

    s_TmpBreak = ip;
    currentInst->opcode = 0x0116; // TODO: Account for jumps and ret
}
static void PrintReg(TGXContext* ctx, const char* buffer) {
    int reg = ParseReg(&buffer);
    if (reg == UNDEF) {
        printf("Unknown register\n");
    }

    int is_float;
    uint64_t val = (uint32_t)GetReg(ctx, reg, &is_float);

    if (is_float) {
        printf("%f\n", *(float*)&val);
    }
    else {
        printf("%016lX (%ld)", val, val);

        if (reg == PF) {
            printf("   Z%u | N%u | A%u | G%u | E%u Code %02X",
                (REG32(ctx, REG_PF) & TGX_PF_Flag_ZeroBit) >> TGX_PF_Shift_ZeroBit,
                (REG32(ctx, REG_PF) & TGX_PF_Flag_NegBit) >> TGX_PF_Shift_NegBit,
                (REG32(ctx, REG_PF) & TGX_PF_Flag_SysActiveBit) >> TGX_PF_Shift_SysActiveBit,
                (REG32(ctx, REG_PF) & TGX_PF_Flag_GraphicsFree) >> TGX_PF_Shift_GraphicsFree,
                (REG32(ctx, REG_PF) & TGX_PF_Flag_ErrorBit) >> TGX_PF_Shift_ErrorBit,
                (REG32(ctx, REG_PF) & TGX_PF_Flag_ErrorMask));
        }

        printf("\n");
    }
}
static void SetReg(TGXContext* ctx, const char* buffer){
    int reg = ParseReg(&buffer);
    if (reg == UNDEF) {
        printf("Unknown register.\n");
        return;
    }
    buffer += 2;
    double value = ParseVal(&buffer);

    if (isnan(value)) {
        printf("Error parsing value.\n");
        return;
    }

    switch (reg) {
        default:
            return;
        case RA:
            REG32(ctx, REG_RA) = (uint32_t)value; break;
        case RB:
            REG32(ctx, REG_RB) = (uint32_t)value; break;
        case RC:
            REG32(ctx, REG_RC) = (uint32_t)value; break;
        case RD:
            REG32(ctx, REG_RD) = (uint32_t)value; break;
        case RE:
            REG32(ctx, REG_RE) = (uint32_t)value; break;
        case RF:
            REG32(ctx, REG_RF) = (uint32_t)value; break;
        case RX:
            REG32(ctx, REG_RX) = (uint32_t)value; break;
        case RY:
            REG32(ctx, REG_RY) = (uint32_t)value; break;
        case RZ:
            REG32(ctx, REG_RZ) = (uint32_t)value; break;
        case RW:
            REG32(ctx, REG_RW) = (uint32_t)value; break;
        case RI:
            REG32(ctx, REG_RI) = (uint32_t)value; break;
        case RJ:
            REG32(ctx, REG_RJ) = (uint32_t)value; break;
        case RK:
            REG32(ctx, REG_RK) = (uint32_t)value; break;
        case RL:
            REG32(ctx, REG_RL) = (uint32_t)value; break;
        case IP:
            REG32(ctx, REG_IP) = (uint32_t)value; break;
        case JR:
            REG32(ctx, REG_JR) = (uint32_t)value; break;
        case SB:
            REG32(ctx, REG_SB) = (uint32_t)value; break;
        case SH:
            REG32(ctx, REG_SH) = (uint32_t)value; break;
        case SP:
            REG32(ctx, REG_SP) = (uint32_t)value; break;
        case PF:
            REG32(ctx, REG_PF) = (uint32_t)value; break;
        case M0:
            REG64(ctx, 0) = value; break;
        case M1:
            REG64(ctx, 1) = value; break;
        case X0:
            REGF32(ctx, REG_FX0) = (float)value; break;
        case Y0:
            REGF32(ctx, REG_FY0) = (float)value; break;
        case Z0:
            REGF32(ctx, REG_FZ0) = (float)value; break;
        case W0:
            REGF32(ctx, REG_FW0) = (float)value; break;
        case X1:
            REGF32(ctx, REG_FX1) = (float)value; break;
        case Y1:
            REGF32(ctx, REG_FY1) = (float)value; break;
        case Z1:
            REGF32(ctx, REG_FZ1) = (float)value; break;
        case W1:
            REGF32(ctx, REG_FW1) = (float)value; break;
        case X2:
            REGF32(ctx, REG_FX2) = (float)value; break;
        case Y2:
            REGF32(ctx, REG_FY2) = (float)value; break;
        case Z2:
            REGF32(ctx, REG_FZ2) = (float)value; break;
        case W2:
            REGF32(ctx, REG_FW2) = (float)value; break;
        case X3:
            REGF32(ctx, REG_FX3) = (float)value; break;
        case Y3:
            REGF32(ctx, REG_FY3) = (float)value; break;
        case Z3:
            REGF32(ctx, REG_FZ3) = (float)value; break;
        case W3:
            REGF32(ctx, REG_FW3) = (float)value; break;
    }
}
static void Next(TGXContext* ctx, Instruction* currentInst) {
    ++currentInst;
    if (!s_IsTmp) {
        ++currentInst;
    }

    PrintInst(currentInst);
}
static void Last(TGXContext* ctx, Instruction* currentInst) {
    --currentInst;
    if (!s_IsTmp) {
        --currentInst;
    }

    PrintInst(currentInst);
}
static void Peek(TGXContext* ctx, const char* buffer) {
    int reg = ParseReg(&buffer);

    uint32_t addr = 0;
    if (reg != UNDEF) {
        buffer += 2;

        int is_float;
        addr = (uint32_t)GetReg(ctx, reg, &is_float);

        if (is_float) {
            printf("Cannot use a float point register for addresses.\n");
            return;
        }
    }
    else {
        double d = ParseVal(&buffer);
        if (isnan(d)) {
            printf("Could not parse an address\n");
            return;
        }

        addr = (uint32_t)d;
    }

#define DISP_LINES 2
#define DISP_WIDTH 16
    uint8_t *cursor = ctx->Memory.memory_begin + addr;
    printf("%08X: ", addr);

    for (int i=0; i<DISP_LINES; i++) {
        uint8_t *start = cursor;
        for (int j=0; j<DISP_WIDTH; j++) {
            uint32_t byte = *start++;
            printf("%02X ", byte);

            if (j == (DISP_WIDTH / 2) - 1) {
                putc(' ', stdout);
            }
        }
        printf("  ");
        start = cursor;
        for (int j=0; j<DISP_WIDTH; j++) {
            char c = (char)*start++;

            if (c < ' ' || c == '\n' || c == '\r' || c == '\t') {
                c = '.';
            }

            putc(c, stdout);
        }

        printf("\n");
        if (i+1 == DISP_LINES) break;

        printf("%08X: ", addr+DISP_WIDTH);
        cursor = start;
    }


#undef DISP_LINES
#undef DISP_WIDTH
}
static void Head(TGXContext* ctx) {
    uint64_t word = MEMACCESS(uint64_t, ctx, REG32(ctx, REG_SP));
    printf("%08X: %016lX (%f)\n", REG32(ctx, REG_SP), word, *(float*)&word);
}

static void SetBreak(TGXContext* ctx, Instruction* current, uint32_t currentLine, const char* buffer) {
    SkipSpaces(&buffer);

    double line = ParseVal(&buffer);

    if (isnan(line)) {
        printf("Error reading the line number.\n");
        return;
    }

    uint32_t tline = (uint32_t)line;
    uint32_t cIp = REG32(ctx, REG_IP);
    int delta = 1;

    if (currentLine > tline) {
        delta = -1;
    }

    while (true) {
        current += delta;
        cIp += sizeof(Instruction) * delta;
        if (current->const_i32 == tline &&
            (current->opcode == 0x0000 || current->opcode == 0x0116)) {

            if (s_FixedTmpCount >= 1024) {
                printf("Breakpoint Limit hit\n");
                return;
            }
            s_FixedTmp[s_FixedTmpCount++] = cIp;

            current->opcode = 0x0116;
            printf("Breakpoint set: \n");
            PrintInst(current+1);
            break;
        }

        if ((void*)current < (void*)ctx->Memory.memory_begin ||
            (void*)current >= (void*)ctx->Memory.memory_end) {
            printf("Line %u not found\n", tline);
            break;
        }
    }


}

#define FALLTHROUGH

static int LaunchDebugShell(TGXContext* ctx) {
    static bool initialized = false;

    if (!initialized) {
        DebugShellInit();
        initialized = true;
    }

    char buffer[256];
    int _continue = true;

    if (s_TmpBreak != 0) {
        s_IsTmp = (s_TmpBreak == REG32(ctx, REG_IP));

        (*((Instruction*)(ctx->Memory.memory_begin + s_TmpBreak))).opcode = 0x0000;
        s_TmpBreak = 0;
    }
    else {
        s_IsTmp = false;
        for (int i=0; i<s_FixedTmpCount; i++) {
            if (s_FixedTmp[i] == REG32(ctx, REG_IP)) {
                s_IsTmp = true;
            }
        }
    }

    do {
        uint32_t line, code;
        Instruction* current = ((Instruction*)(ctx->Memory.memory_begin + REG32(ctx, REG_IP)));
        line = current->const_i32;
        code = current->param16;

        printf("%u|%02X> ", line, code);
        fgets(buffer, sizeof(buffer), stdin);
        buffer[strcspn(buffer, "\n")] = '\0';

        char* str = buffer;
        int cmd = ParseCommand((const char**)&str);
        switch (cmd) {
            default:
            case Cmd_Unknown:
                printf("Unknown command\n");
                break;
            case Cmd_Help:
                ShellHelp();
                break;
            case Cmd_Quit:
                _continue = false;
                break;
            case Cmd_Restart:
                ctx->ExitCode = TGX_EXIT_CODE_RESTART;
                FALLTHROUGH
            case Cmd_Kill:
                (*(Instruction*)(ctx->Memory.memory_begin + (REG32(ctx, REG_IP) + sizeof(Instruction)))).opcode = 0x010C;
                _continue = false;
                break;
            case Cmd_StepOvr:
                StepNext(ctx, current, false);
                _continue = false;
                break;
            case Cmd_Step:
                StepNext(ctx, current, true);
                _continue = false;
                break;
            case Cmd_PrintReg:
                PrintReg(ctx, str);
                break;
            case Cmd_SetReg:
                SetReg(ctx, str);
                break;
            case Cmd_PrintNextInst:
                Next(ctx, current);
                break;
            case Cmd_PrintLastInst:
                Last(ctx, current);
                break;
            case Cmd_PeekMem:
                Peek(ctx, str);
                break;
            case Cmd_PeekStack:
                Head(ctx);
                break;
            case Cmd_Break:
                SetBreak(ctx, current, line, str);
                break;

        }
    }
    while (_continue);

    return 0;
}