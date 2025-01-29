//
// Created by ctlf on 1/24/25.
//
#include "TGX.h"

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <sys/wait.h>
#include <stdexcept>
#include <fstream>
#include <optional>

std::string preprocess_input(const char* filename);

struct string_view {
    const char* begin;
    const char* end;
};

enum class Command {
    Undefined,
    Abs, Acos, Add, And, Asin, Atan, Atan2,
    Bcosmxd, Bsinmxd, Btanmxd,
    Ceil, Cmp, Cmxb, Cos,
    Dec, Div,
    Epow,
    Floor,
    Hlt,
    Int, Inc, Inv,
    Jmp, Jz, Jnz, Jgt, Jge, Jlt, Jle,
    Land, Lor, Lnot, Lde, Log10, Loge, Ldpi, Ldmsk,
    Mov, Movia, Movda, Movc, Movr, Mcpy, Mcmp, Mclr, Mul, Mod,
    Nop, Neg, Not,
    Or,
    Push, Pop, Pow,
    Rbr, Rbl, Ret,
    Swap, Sub, Sqr, Sqrt, Sin, Syscall,
    Tsto, Tld, Tan,
    Vnorm, Vadd, Vsub, Vmul, Vdiv, Vdot, Vlen, Vswz,
    Xor,
};

enum class RegisterID {
    RA, RB, RC, RD, RE, RF, RW, RX, RY, RZ,
    IP, JR, SB, SP, SH, PF, RI, RJ, RK, RL,
    RAL, RAH, RBL, RBH, RCL, RCH, RDL, RDH,
    REL, REH, RFL, RFH, RWL, RWH, RXL, RXH,
    RYL, RYH,
    RZL, RZH, RAS, RBS, RCS, RDS, RES, RFS,
    RWS, RXS, RYS, RZS, M0, M1, M0L, M0H,
    M1L, M1H, FX0, FY0, FZ0, FW0, FX1, FY1,
    FZ1, FW1, FX2, FY2, FZ2, FW2, FX3, FY3,
    FZ3, FW3, VE0, VE1, VE2, VE3,
    REGCOUNT
};

enum class Type {
    Integer, Float, Vec,
};

struct RegisterInfo {
    RegisterID id;
    Type type;
    int width;
    bool generalPurpose;
    bool sysAuto;
    int emitID;
};

struct CompilerContext {
    const char* input;
    std::vector<Instruction> program_section;
    std::vector<uint8_t> data_section;
    std::unordered_map<std::string, uint32_t> program_labels;
    std::unordered_map<std::string, uint32_t> data_labels;
    std::vector<std::string> export_labels;
};

typedef bool(*EmitFunc)(CompilerContext&);

bool TriggerUndefined(CompilerContext& ctx);
bool EmitAbs(CompilerContext& ctx);
bool EmitAcos(CompilerContext& ctx);
bool EmitAdd(CompilerContext& ctx);
bool EmitAnd(CompilerContext& ctx);
bool EmitAsin(CompilerContext& ctx);
bool EmitAtan(CompilerContext& ctx);
bool EmitAtan2(CompilerContext& ctx);
bool EmitBcosmxd(CompilerContext& ctx);
bool EmitBsinmxd(CompilerContext& ctx);
bool EmitBtanmxd(CompilerContext& ctx);
bool EmitCeil(CompilerContext& ctx);
bool EmitCmp(CompilerContext& ctx);
bool EmitCmxb(CompilerContext& ctx);
bool EmitCos(CompilerContext& ctx);
bool EmitDec(CompilerContext& ctx);
bool EmitDiv(CompilerContext& ctx);
bool EmitEpow(CompilerContext& ctx);
bool EmitFloor(CompilerContext& ctx);
bool EmitHlt(CompilerContext& ctx);
bool EmitInt(CompilerContext& ctx);
bool EmitInc(CompilerContext& ctx);
bool EmitInv(CompilerContext& ctx);
bool EmitJmp(CompilerContext& ctx);
bool EmitJz(CompilerContext& cxt);
bool EmitJnz(CompilerContext& ctx);
bool EmitJgt(CompilerContext& ctx);
bool EmitJge(CompilerContext& ctx);
bool EmitJlt(CompilerContext& ctx);
bool EmitJle(CompilerContext& ctx);
bool EmitLand(CompilerContext& ctx);
bool EmitLor(CompilerContext& ctx);
bool EmitLnot(CompilerContext& ctx);
bool EmitLde(CompilerContext& ctx);
bool EmitLog10(CompilerContext& ctx);
bool EmitLoge(CompilerContext& ctx);
bool EmitLdpi(CompilerContext& ctx);
bool EmitLdmsk(CompilerContext& ctx);
bool EmitMov(CompilerContext& ctx);
bool EmitMovia(CompilerContext& ctx);
bool EmitMovda(CompilerContext& ctx);
bool EmitMovc(CompilerContext& ctx);
bool EmitMovr(CompilerContext& ctx);
bool EmitMcpy(CompilerContext& ctx);
bool EmitMcmp(CompilerContext& ctx);
bool EmitMclr(CompilerContext& ctx);
bool EmitMul(CompilerContext& ctx);
bool EmitMod(CompilerContext& ctx);
bool EmitNop(CompilerContext& ctx);
bool EmitNeg(CompilerContext& ctx);
bool EmitNot(CompilerContext& ctx);
bool EmitOr(CompilerContext& ctx);
bool EmitPush(CompilerContext& ctx);
bool EmitPop(CompilerContext& ctx);
bool EmitPow(CompilerContext& ctx);
bool EmitRbr(CompilerContext& ctx);
bool EmitRbl(CompilerContext& ctx);
bool EmitRet(CompilerContext& ctx);
bool EmitSwap(CompilerContext& ctx);
bool EmitSub(CompilerContext& ctx);
bool EmitSqr(CompilerContext& ctx);
bool EmitSqrt(CompilerContext& ctx);
bool EmitSin(CompilerContext& ctx);
bool EmitSyscall(CompilerContext& ctx);
bool EmitTsto(CompilerContext& ctx);
bool EmitTld(CompilerContext& ctx);
bool EmitTan(CompilerContext& ctx);
bool EmitVnorm(CompilerContext& ctx);
bool EmitVadd(CompilerContext& ctx);
bool EmitVsub(CompilerContext& ctx);
bool EmitVmul(CompilerContext& ctx);
bool EmitVdiv(CompilerContext& ctx);
bool EmitVdot(CompilerContext& ctx);
bool EmitVlen(CompilerContext& ctx);
bool EmitVswz(CompilerContext& ctx);
bool EmitXor(CompilerContext& ctx);

static EmitFunc s_InstructionFuncs[] {
    TriggerUndefined, EmitAbs, EmitAcos, EmitAdd, EmitAnd, EmitAsin, EmitAtan, EmitAtan2,
    EmitBcosmxd, EmitBsinmxd, EmitBtanmxd,
    EmitCeil, EmitCmp, EmitCmxb, EmitCos,
    EmitDec, EmitDiv,
    EmitEpow, EmitFloor, EmitHlt,
    EmitInt, EmitInc, EmitInv,
    EmitJmp, EmitJz, EmitJnz, EmitJgt, EmitJge, EmitJlt, EmitJle,
    EmitLand, EmitLor, EmitLnot, EmitLde, EmitLog10, EmitLoge, EmitLdpi, EmitLdmsk,
    EmitMov, EmitMovia, EmitMovda, EmitMovc, EmitMovr, EmitMcpy, EmitMcmp, EmitMclr, EmitMul, EmitMod,
    EmitNop, EmitNeg, EmitNot,
    EmitOr, EmitPush, EmitPop, EmitPow,
    EmitRbr, EmitRbl, EmitRet,
    EmitSwap, EmitSub, EmitSqr, EmitSqrt, EmitSin, EmitSyscall,
    EmitTsto, EmitTld, EmitTan,
    EmitVnorm, EmitVadd, EmitVsub, EmitVmul, EmitVdiv, EmitVdot, EmitVlen, EmitVswz,
    EmitXor,
};

static RegisterInfo s_RegisterTable[RegisterID::REGCOUNT] {
    { .id = RegisterID::RA, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::RB, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::RC, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  2 },
    { .id = RegisterID::RD, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  3 },
    { .id = RegisterID::RE, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  4 },
    { .id = RegisterID::RF, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  5 },
    { .id = RegisterID::RW, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  6 },
    { .id = RegisterID::RX, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  7 },
    { .id = RegisterID::RY, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  8 },
    { .id = RegisterID::RZ, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  9 },
    { .id = RegisterID::IP, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = true, .emitID = 10 },
    { .id = RegisterID::JR, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = true, .emitID = 11 },
    { .id = RegisterID::SB, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = true, .emitID = 12 },
    { .id = RegisterID::SP, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = true, .emitID = 13 },
    { .id = RegisterID::SH, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = true, .emitID = 14 },
    { .id = RegisterID::PF, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = true, .emitID = 15 },
    { .id = RegisterID::RI, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 16 },
    { .id = RegisterID::RJ, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 17 },
    { .id = RegisterID::RK, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 18 },
    { .id = RegisterID::RL, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 19 },
    { .id = RegisterID::RAL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::RAH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::RBL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  2 },
    { .id = RegisterID::RBH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  3 },
    { .id = RegisterID::RCL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  4 },
    { .id = RegisterID::RCH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  5 },
    { .id = RegisterID::RDL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  6 },
    { .id = RegisterID::RDH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  7 },
    { .id = RegisterID::REL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  8 },
    { .id = RegisterID::REH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID =  9 },
    { .id = RegisterID::RFL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 10 },
    { .id = RegisterID::RFH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 11 },
    { .id = RegisterID::RWL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 12 },
    { .id = RegisterID::RWH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 13 },
    { .id = RegisterID::RXL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 14 },
    { .id = RegisterID::RXH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 15 },
    { .id = RegisterID::RYL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 16 },
    { .id = RegisterID::RYH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 17 },
    { .id = RegisterID::RZL, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 18 },
    { .id = RegisterID::RZH, .type = Type::Integer, .width = 2, .generalPurpose = true, .sysAuto = false, .emitID = 19 },
    { .id = RegisterID::RAS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::RBS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::RCS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  2 },
    { .id = RegisterID::RDS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  3 },
    { .id = RegisterID::RES, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  4 },
    { .id = RegisterID::RFS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  5 },
    { .id = RegisterID::RWS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  6 },
    { .id = RegisterID::RXS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  7 },
    { .id = RegisterID::RYS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  8 },
    { .id = RegisterID::RZS, .type = Type::Integer, .width = 1, .generalPurpose = true, .sysAuto = false, .emitID =  9 },
    { .id = RegisterID::M0, .type = Type::Integer, .width = 8, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::M1, .type = Type::Integer, .width = 8, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::M0L, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::M0H, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::M1L, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::M1H, .type = Type::Integer, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::FX0, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  0 },
    { .id = RegisterID::FY0, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  1 },
    { .id = RegisterID::FZ0, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  2 },
    { .id = RegisterID::FW0, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  3 },
    { .id = RegisterID::FX1, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  4 },
    { .id = RegisterID::FY1, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  5 },
    { .id = RegisterID::FZ1, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  6 },
    { .id = RegisterID::FW1, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  7 },
    { .id = RegisterID::FX2, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  8 },
    { .id = RegisterID::FY2, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID =  9 },
    { .id = RegisterID::FZ2, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 10 },
    { .id = RegisterID::FW2, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 11 },
    { .id = RegisterID::FX3, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 12 },
    { .id = RegisterID::FY3, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 13 },
    { .id = RegisterID::FZ3, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 14 },
    { .id = RegisterID::FW3, .type = Type::Float, .width = 4, .generalPurpose = true, .sysAuto = false, .emitID = 15 },
    { .id = RegisterID::VE0, .type = Type::Vec, .width = 16, .generalPurpose = true, .sysAuto = false, .emitID = 0 },
    { .id = RegisterID::VE1, .type = Type::Vec, .width = 16, .generalPurpose = true, .sysAuto = false, .emitID = 1 },
    { .id = RegisterID::VE2, .type = Type::Vec, .width = 16, .generalPurpose = true, .sysAuto = false, .emitID = 2 },
    { .id = RegisterID::VE3, .type = Type::Vec, .width = 16, .generalPurpose = true, .sysAuto = false, .emitID = 3 },
};

static std::unordered_map<char, std::vector<std::string>> s_Commands = {
    {'a', { "abs", "acos", "and", "asin", "atan2", "atan", "add" }},
    {'b', { "bcosmxd", "bsinmxd", "btanmxd", }},
    {'c', { "ceil", "cmp", "cmxb", "cos", }},
    {'d', { "dec", "div", }},
    {'e', { "epow", }},
    {'f', { "floor", }},
    {'h', { "hlt", }},
    {'i', { "int", "inc", "inv", }},
    {'j', { "jmp", "jz", "jnz", "jgt", "jge", "jlt", "jle", }},
    {'l', { "land", "lor", "lnot", "lde", "log10", "loge", "ldpi", "ldmsk", }},
    {'m', { "movia", "movda", "movc", "movr", "mov", "mcpy", "mcmp", "mclr", "mul", "mod", }},
    {'n', { "nop", "neg", "not", }},
    {'o', { "or", }},
    {'p', { "push", "pop", "pow", }},
    {'r', { "rbr", "rbl", "ret", }},
    {'s', { "swap", "sub", "sqrt", "sqr", "sin", "syscall", }},
    {'t', { "tsto", "tld", "tan", }},
    {'v', { "vnorm", "vadd", "vsub", "vmul", "vdiv", "vdot", "vlen", "vswz", }},
    {'x', { "xor", }},
};
static std::unordered_map<std::string, Command> s_CommandIDs = {
    {"abs", Command::Abs},         {"acos", Command::Acos},       {"and", Command::And},
    {"asin", Command::Asin},       {"atan2", Command::Atan2},     {"atan", Command::Atan},
    {"add", Command::Add},         {"bcosmxd", Command::Bcosmxd}, {"bsinmxd", Command::Bsinmxd},
    {"btanmxd", Command::Btanmxd}, {"ceil", Command::Ceil},       {"cmp", Command::Cmp},
    {"cmxb", Command::Cmxb},       {"cos", Command::Cos},         {"dec", Command::Dec},
    {"div", Command::Div},         {"epow", Command::Epow},       {"floor", Command::Floor},
    {"hlt", Command::Hlt},         {"int", Command::Int},         {"inc", Command::Inc},
    {"inv", Command::Inv},         {"jmp", Command::Jmp},         {"jz", Command::Jz},
    {"jnz", Command::Jnz},         {"jgt", Command::Jgt},         {"jge", Command::Jge},
    {"jlt", Command::Jlt},         {"jle", Command::Jle},         {"land", Command::Land},
    {"lor", Command::Lor},         {"lnot", Command::Lnot},       {"lde", Command::Lde},
    {"log10", Command::Log10},     {"loge", Command::Loge},       {"ldpi", Command::Ldpi},
    {"ldmsk", Command::Ldmsk},     {"movia", Command::Movia},     {"movda", Command::Movda},
    {"movc", Command::Movc},       {"movr", Command::Movr},       {"mov", Command::Mov},
    {"mcpy", Command::Mcpy},       {"mclr", Command::Mclr},       {"mcmp", Command::Mcmp},
    {"mul", Command::Mul},         {"mod", Command::Mod},         {"nop", Command::Nop},
    {"neg", Command::Neg},         {"not", Command::Not},         {"or", Command::Or},
    {"push", Command::Push},       {"pop", Command::Pop},         {"pow", Command::Pow},
    {"rbl", Command::Rbl},         {"rbr", Command::Rbr},         {"ret", Command::Ret},
    {"swap", Command::Swap},       {"sub", Command::Sub},         {"sqrt", Command::Sqrt},
    {"sqr", Command::Sqr},         {"sin", Command::Sin},         {"syscall", Command::Syscall},
    {"tsto", Command::Tsto},       {"tld", Command::Tld},         {"tan", Command::Tan},
    {"vnorm", Command::Vnorm},     {"vadd", Command::Vadd},       {"vsub", Command::Vsub},
    {"vmul", Command::Vmul},       {"vdiv", Command::Vdiv},       {"vdot", Command::Vdot},
    {"vlen", Command::Vlen},       {"vswz", Command::Vswz},       {"xor", Command::Xor},
};

static std::unordered_map<std::string, RegisterID> s_RegisterMap = {
    { "ra", RegisterID::RA }, { "rb", RegisterID::RB }, { "rc", RegisterID::RC },
    { "rd", RegisterID::RD }, { "re", RegisterID::RE }, { "rf", RegisterID::RF },
    { "rw", RegisterID::RW }, { "rx", RegisterID::RX }, { "ry", RegisterID::RY },
    { "rz", RegisterID::RZ }, { "ip", RegisterID::IP }, { "jr", RegisterID::JR },
    { "sb", RegisterID::SB }, { "sp", RegisterID::SP }, { "sh", RegisterID::SH },
    { "pf", RegisterID::PF }, { "ri", RegisterID::RI }, { "rj", RegisterID::RJ },
    { "rk", RegisterID::RK }, { "rl", RegisterID::RL },
    { "ral", RegisterID::RAL }, { "rah", RegisterID::RAH }, { "rbl", RegisterID::RBL }, { "rbh", RegisterID::RBH },
    { "rcl", RegisterID::RCL }, { "rch", RegisterID::RCH }, { "rdl", RegisterID::RDL }, { "rdh", RegisterID::RDH },
    { "rel", RegisterID::REL }, { "reh", RegisterID::REH }, { "rfl", RegisterID::RFL }, { "rfh", RegisterID::RFH },
    { "rwl", RegisterID::RWL }, { "rwh", RegisterID::RWH }, { "rxl", RegisterID::RXL }, { "rxh", RegisterID::RXH },
    { "ryl", RegisterID::RYL }, { "ryh", RegisterID::RYH }, { "rzl", RegisterID::RZL }, { "rzh", RegisterID::RZH },
    { "ras", RegisterID::RAS }, { "rbs", RegisterID::RBS }, { "rcs", RegisterID::RCS }, { "rds", RegisterID::RDS },
    { "res", RegisterID::RES }, { "rfs", RegisterID::RES }, { "rws", RegisterID::RWS }, { "rxs", RegisterID::RXS },
    { "rys", RegisterID::RYS }, { "rzs", RegisterID::RZS },
    { "m0", RegisterID::M0 }, { "m1", RegisterID::M1 },
    { "m0l", RegisterID::M0L }, { "m0h", RegisterID::M0H }, { "m1l", RegisterID::M1L }, { "m1h", RegisterID::M1H },
    { "fx0", RegisterID::FX0 }, { "fy0", RegisterID::FY0 }, { "fz0", RegisterID::FZ0 }, { "fw0", RegisterID::FW0 },
    { "fx1", RegisterID::FX1 }, { "fy1", RegisterID::FY1 }, { "fz1", RegisterID::FZ1 }, { "fw1", RegisterID::FW1 },
    { "fx2", RegisterID::FX2 }, { "fy2", RegisterID::FY2 }, { "fz2", RegisterID::FZ2 }, { "fw2", RegisterID::FW2 },
    { "fx3", RegisterID::FX3 }, { "fy3", RegisterID::FY3 }, { "fz3", RegisterID::FZ3 }, { "fw3", RegisterID::FW3 },
    { "ve0", RegisterID::VE0 }, { "ve1", RegisterID::VE1 }, { "ve2", RegisterID::VE2 }, { "ve3", RegisterID::VE3 },
};

char next(const char* &ch) {
    if (*(ch+1)) {
        ch++;
        return *ch;
    }
    return 0;
}

bool streq(const char* a, const char* b, size_t len) {
    for (size_t i=0; i<len; i++) {
        if (*(a+i) == 0 || *(b+i) == 0) {
            return false;
        }
        if (*(a+i) != *(b+i)) {
            return false;
        }
    }
    return true;
}

bool consume_comment(const char* &ptr) {
    if (*ptr == '#' || *ptr == ';') {
        while (*(++ptr)) {
            if (*ptr == '\n') {
                ptr++;
                return true;
            }
        }
    }
    else {
        return false;
    }
    return true;
}
bool consume_whitespace(const char* &ptr) {
    if (*ptr != ' ' && *ptr != '\t' && *ptr != '\n' && *ptr != '\r') return false;

    char c;
    do {
        c = *(ptr++);
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
            break;
        }
    } while (*ptr);
    return true;
}

Command get_command(const char* &ptr) {
    if (*ptr == 0) {
        return Command::Undefined;
    }

    auto iter = s_Commands.find(*ptr);
    if (iter != s_Commands.end()) {
        auto& bucket = iter->second;
        for (std::string& s : bucket) {
            if (streq(s.c_str(), ptr, s.length())) {
                return s_CommandIDs.at(s);
            }
        }
    }

    return Command::Undefined;
}


std::optional<string_view> parse_number(const char* ptr){
    string_view num = { nullptr, nullptr };

    bool has_dec = false;
    bool is_hex = false;
    bool is_bin = false;
    bool firstz = false;

    if (!isdigit(*ptr)) {
        return std::optional<string_view>{};
    }
    num.begin = ptr;
    num.end = ptr;

    firstz = *num.begin == '0';

    do {
        num.end++;
        char c = *num.end;

        if (firstz && (c == 'x' || c == 'X')) {
            is_hex = true;
            continue;
        }
        if (firstz && (c == 'b' || c == 'B')) {
            is_bin = true;
            continue;
        }

        if (is_hex) {
            if (!isdigit(c) && ( !('a' <= c && c <= 'f') && !('A' <= c && c <= 'F') )) {
                break;
            }
            continue;
        }

        if (is_bin) {
            if (c != '0' && c != '1') {
                break;
            }
            continue;
        }

        if (!isdigit(c)) {
            if (!has_dec && c == '.') {
                has_dec = true;
                continue;
            }
            break;
        }
    } while (*num.end);

    return { num };
}

std::optional<string_view> parse_identifier(const char* ptr) {
    string_view id = { nullptr, nullptr };

    if (!isalpha(*ptr) && *ptr != '_' && *ptr != '$') {
        return std::optional<string_view>{};
    }
    id.begin = ptr;
    id.end = ptr;

    do {
        id.end++;
        if (!isalnum(*id.end) && *id.end != '_' && *id.end != '$') {
            break;
        }
    } while (*id.end);
    return { id };
}
// TODO: Not passed by reference, will not be modified. This needs to be fixed
std::optional<RegisterID> parse_register(const char* ptr) {
    auto rid = parse_identifier(ptr);
    if (!rid.has_value()) {
        return std::optional<RegisterID>{};
    }
    std::string regstr{ rid.value().begin, rid.value().end };
    if (auto iter = s_RegisterMap.find(regstr); iter != s_RegisterMap.end()) {
        return iter->second();
    }

    return std::optional<RegisterID>{};
}

void consume_ignore_whitespace_and_comments(CompilerContext& ctx) {
    do {
        bool consumed = consume_whitespace(ctx.input);

        if (consumed) {
            continue;
        }

        consumed = consume_comment(ctx.input);

        if (consumed) {
            continue;
        }

        break;
    } while (true);
}

bool read_and_emit_instruction(CompilerContext& ctx) {
    consume_ignore_whitespace_and_comments(ctx);



    return true;
}



bool parse_label_export(CompilerContext& ctx) {
    const char* ptr = ctx.input;
    if (!streq(ptr, "@export", 7)) {
        return false;
    }
    ptr += 7;

    consume_whitespace(ptr);
    // TODO
    return false;
}

bool compile_and_emit(CompilerContext& ctx) {
    consume_ignore_whitespace_and_comments(ctx);

    //TODO
    return false;
}


int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Please provide a source file.\n";
        return 1;
    }

    std::string output;

    try {
        output = preprocess_input(argv[1]);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    // preprocessing is working, need to filter out preprocessor comments #...
    // then we need to actually tokenize and assemble the output.
    // this should be pretty straightforward, seeing as the preprocessor is done already.

    return 0;
}

std::string preprocess_input(const char *filename) {
    std::vector<char> bytes;
    int pipefd[2];
    pid_t pid;

    // preprocess file
    if (pipe(pipefd) == -1) {
        perror("pipe");
        throw std::runtime_error("Could not create pipe for preprocessing");
    }

    pid = fork();
    if (pid == -1) {
        perror("fork");
        throw std::runtime_error("Error forking the current process.");
    }

    if (pid == 0) {
        // child process
        close(pipefd[0]);
        if (dup2(pipefd[1], STDOUT_FILENO) == -1) {
            perror("dup2"); // dup2 redirects stdout to this pipe
            exit(EXIT_FAILURE);
        }
        close(pipefd[1]);

        execlp("cc", "cc", "-E", "-x", "c", filename, NULL);
        //execlp only returns on failure
        perror("execlp");
        exit(EXIT_FAILURE);
    }

    // parent process
    close(pipefd[1]);

    char buffer[1024];
    ssize_t bytes_read;

    while ((bytes_read = read(pipefd[0], buffer, sizeof(buffer)-1)) > 0) {
        bytes.insert(bytes.end(), buffer, buffer+bytes_read);
    }

    if (bytes_read == -1) {
        perror("read");
        close(pipefd[0]);
        throw std::runtime_error("Unable to read from output");
    }

    close(pipefd[0]);
    int status;
    waitpid(pid, &status, 0);

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        throw std::runtime_error("Preprocessing failed with an error.");
    }

    return { bytes.begin(), bytes.end() };
}


bool TriggerUndefined(CompilerContext& ctx){
    return true;
}
bool EmitAbs(CompilerContext& ctx){
    return false;
}
bool EmitAcos(CompilerContext& ctx){
    return false;
}
bool EmitAdd(CompilerContext& ctx){
    return false;
}
bool EmitAnd(CompilerContext& ctx){
    return false;
}
bool EmitAsin(CompilerContext& ctx){
    return false;
}
bool EmitAtan(CompilerContext& ctx){
    return false;
}
bool EmitAtan2(CompilerContext& ctx){
    return false;
}
bool EmitBcosmxd(CompilerContext& ctx){
    return false;
}
bool EmitBsinmxd(CompilerContext& ctx){
    return false;
}
bool EmitBtanmxd(CompilerContext& ctx){
    return false;
}
bool EmitCeil(CompilerContext& ctx){
    return false;
}
bool EmitCmp(CompilerContext& ctx){
    return false;
}
bool EmitCmxb(CompilerContext& ctx){
    return false;
}
bool EmitCos(CompilerContext& ctx){
    return false;
}
bool EmitDec(CompilerContext& ctx){
    return false;
}
bool EmitDiv(CompilerContext& ctx){
    return false;
}
bool EmitEpow(CompilerContext& ctx){
    return false;
}
bool EmitFloor(CompilerContext& ctx){
    return false;
}
bool EmitHlt(CompilerContext& ctx){
    return false;
}
bool EmitInt(CompilerContext& ctx){
    return false;
}
bool EmitInc(CompilerContext& ctx){
    return false;
}
bool EmitInv(CompilerContext& ctx){
    return false;
}
bool EmitJmp(CompilerContext& ctx){
    return false;
}
bool EmitJz(CompilerContext& cxt){
    return false;
}
bool EmitJnz(CompilerContext& ctx){
    return false;
}
bool EmitJgt(CompilerContext& ctx){
    return false;
}
bool EmitJge(CompilerContext& ctx){
    return false;
}
bool EmitJlt(CompilerContext& ctx){
    return false;
}
bool EmitJle(CompilerContext& ctx){
    return false;
}
bool EmitLand(CompilerContext& ctx){
    return false;
}
bool EmitLor(CompilerContext& ctx){
    return false;
}
bool EmitLnot(CompilerContext& ctx){
    return false;
}
bool EmitLde(CompilerContext& ctx){
    return false;
}
bool EmitLog10(CompilerContext& ctx){
    return false;
}
bool EmitLoge(CompilerContext& ctx){
    return false;
}
bool EmitLdpi(CompilerContext& ctx){
    return false;
}
bool EmitLdmsk(CompilerContext& ctx){
    return false;
}
bool EmitMov(CompilerContext& ctx){
    return false;
}
bool EmitMovia(CompilerContext& ctx){
    return false;
}
bool EmitMovda(CompilerContext& ctx){
    return false;
}
bool EmitMovc(CompilerContext& ctx){
    return false;
}
bool EmitMovr(CompilerContext& ctx){
    return false;
}
bool EmitMcpy(CompilerContext& ctx){
    return false;
}
bool EmitMcmp(CompilerContext& ctx){
    return false;
}
bool EmitMclr(CompilerContext& ctx){
    return false;
}
bool EmitMul(CompilerContext& ctx){
    return false;
}
bool EmitMod(CompilerContext& ctx){
    return false;
}
bool EmitNop(CompilerContext& ctx){
    return false;
}
bool EmitNeg(CompilerContext& ctx){
    return false;
}
bool EmitNot(CompilerContext& ctx){
    return false;
}
bool EmitOr(CompilerContext& ctx){
    return false;
}
bool EmitPush(CompilerContext& ctx){
    return false;
}
bool EmitPop(CompilerContext& ctx){
    return false;
}
bool EmitPow(CompilerContext& ctx){
    return false;
}
bool EmitRbr(CompilerContext& ctx){
    return false;
}
bool EmitRbl(CompilerContext& ctx){
    return false;
}
bool EmitRet(CompilerContext& ctx){
    return false;
}
bool EmitSwap(CompilerContext& ctx){
    return false;
}
bool EmitSub(CompilerContext& ctx){
    return false;
}
bool EmitSqr(CompilerContext& ctx){
    return false;
}
bool EmitSqrt(CompilerContext& ctx){
    return false;
}
bool EmitSin(CompilerContext& ctx){
    return false;
}
bool EmitSyscall(CompilerContext& ctx){
    return false;
}
bool EmitTsto(CompilerContext& ctx){
    return false;
}
bool EmitTld(CompilerContext& ctx){
    return false;
}
bool EmitTan(CompilerContext& ctx){
    return false;
}
bool EmitVnorm(CompilerContext& ctx){
    return false;
}
bool EmitVadd(CompilerContext& ctx){
    return false;
}
bool EmitVsub(CompilerContext& ctx){
    return false;
}
bool EmitVmul(CompilerContext& ctx){
    return false;
}
bool EmitVdiv(CompilerContext& ctx){
    return false;
}
bool EmitVdot(CompilerContext& ctx){
    return false;
}
bool EmitVlen(CompilerContext& ctx){
    return false;
}
bool EmitVswz(CompilerContext& ctx){
    return false;
}
bool EmitXor(CompilerContext& ctx){
    return false;
}
