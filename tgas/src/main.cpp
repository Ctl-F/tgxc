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

enum class CodeSection {
    Meta,
    Data,
    Program,
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
    std::unordered_map<std::string, uint32_t> program_labels; // this is index into the vector, not bytes yet
    std::unordered_map<std::string, uint32_t> data_labels; // this is byte index (since it's a vector of 8bit ints)
    std::vector<std::string> export_labels;
    std::vector<std::pair<uint32_t, std::string>> unlinked_program_labels;
    std::vector<uint32_t> instructions_to_relink;
    CodeSection section = CodeSection::Meta;
};

typedef bool(*EmitFunc)(CompilerContext&, Instruction&);

bool TriggerUndefined(CompilerContext& ctx, Instruction&);
bool EmitAbs(CompilerContext& ctx, Instruction&);
bool EmitAcos(CompilerContext& ctx, Instruction&);
bool EmitAdd(CompilerContext& ctx, Instruction&);
bool EmitAnd(CompilerContext& ctx, Instruction&);
bool EmitAsin(CompilerContext& ctx, Instruction&);
bool EmitAtan(CompilerContext& ctx, Instruction&);
bool EmitAtan2(CompilerContext& ctx, Instruction&);
bool EmitBcosmxd(CompilerContext& ctx, Instruction&);
bool EmitBsinmxd(CompilerContext& ctx, Instruction&);
bool EmitBtanmxd(CompilerContext& ctx, Instruction&);
bool EmitCeil(CompilerContext& ctx, Instruction&);
bool EmitCmp(CompilerContext& ctx, Instruction&);
bool EmitCmxb(CompilerContext& ctx, Instruction&);
bool EmitCos(CompilerContext& ctx, Instruction&);
bool EmitDec(CompilerContext& ctx, Instruction&);
bool EmitDiv(CompilerContext& ctx, Instruction&);
bool EmitEpow(CompilerContext& ctx, Instruction&);
bool EmitFloor(CompilerContext& ctx, Instruction&);
bool EmitHlt(CompilerContext& ctx, Instruction&);
bool EmitInt(CompilerContext& ctx, Instruction&);
bool EmitInc(CompilerContext& ctx, Instruction&);
bool EmitInv(CompilerContext& ctx, Instruction&);
bool EmitJmp(CompilerContext& ctx, Instruction&);
bool EmitJz(CompilerContext& cxt, Instruction&);
bool EmitJnz(CompilerContext& ctx, Instruction&);
bool EmitJgt(CompilerContext& ctx, Instruction&);
bool EmitJge(CompilerContext& ctx, Instruction&);
bool EmitJlt(CompilerContext& ctx, Instruction&);
bool EmitJle(CompilerContext& ctx, Instruction&);
bool EmitLand(CompilerContext& ctx, Instruction&);
bool EmitLor(CompilerContext& ctx, Instruction&);
bool EmitLnot(CompilerContext& ctx, Instruction&);
bool EmitLde(CompilerContext& ctx, Instruction&);
bool EmitLog10(CompilerContext& ctx, Instruction&);
bool EmitLoge(CompilerContext& ctx, Instruction&);
bool EmitLdpi(CompilerContext& ctx, Instruction&);
bool EmitLdmsk(CompilerContext& ctx, Instruction&);
bool EmitMov(CompilerContext& ctx, Instruction&);
bool EmitMovia(CompilerContext& ctx, Instruction&);
bool EmitMovda(CompilerContext& ctx, Instruction&);
bool EmitMovc(CompilerContext& ctx, Instruction&);
bool EmitMovr(CompilerContext& ctx, Instruction&);
bool EmitMcpy(CompilerContext& ctx, Instruction&);
bool EmitMcmp(CompilerContext& ctx, Instruction&);
bool EmitMclr(CompilerContext& ctx, Instruction&);
bool EmitMul(CompilerContext& ctx, Instruction&);
bool EmitMod(CompilerContext& ctx, Instruction&);
bool EmitNop(CompilerContext& ctx, Instruction&);
bool EmitNeg(CompilerContext& ctx, Instruction&);
bool EmitNot(CompilerContext& ctx, Instruction&);
bool EmitOr(CompilerContext& ctx, Instruction&);
bool EmitPush(CompilerContext& ctx, Instruction&);
bool EmitPop(CompilerContext& ctx, Instruction&);
bool EmitPow(CompilerContext& ctx, Instruction&);
bool EmitRbr(CompilerContext& ctx, Instruction&);
bool EmitRbl(CompilerContext& ctx, Instruction&);
bool EmitRet(CompilerContext& ctx, Instruction&);
bool EmitSwap(CompilerContext& ctx, Instruction&);
bool EmitSub(CompilerContext& ctx, Instruction&);
bool EmitSqr(CompilerContext& ctx, Instruction&);
bool EmitSqrt(CompilerContext& ctx, Instruction&);
bool EmitSin(CompilerContext& ctx, Instruction&);
bool EmitSyscall(CompilerContext& ctx, Instruction&);
bool EmitTsto(CompilerContext& ctx, Instruction&);
bool EmitTld(CompilerContext& ctx, Instruction&);
bool EmitTan(CompilerContext& ctx, Instruction&);
bool EmitVnorm(CompilerContext& ctx, Instruction&);
bool EmitVadd(CompilerContext& ctx, Instruction&);
bool EmitVsub(CompilerContext& ctx, Instruction&);
bool EmitVmul(CompilerContext& ctx, Instruction&);
bool EmitVdiv(CompilerContext& ctx, Instruction&);
bool EmitVdot(CompilerContext& ctx, Instruction&);
bool EmitVlen(CompilerContext& ctx, Instruction&);
bool EmitVswz(CompilerContext& ctx, Instruction&);
bool EmitXor(CompilerContext& ctx, Instruction&);

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

static RegisterInfo s_RegisterTable[static_cast<size_t>(RegisterID::REGCOUNT)] {
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
            ptr--;
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

struct NumberLiteral {
    union {
        uint64_t uint_val;
        float float_val;
    };
    bool is_float;
};

NumberLiteral get_number_literal(string_view str){
    NumberLiteral num{};

    std::string literal{str.begin, str.end};
    const char* cstr = literal.c_str();

    if(literal.size() >= 2 && literal[0] == '0'){
        if(literal[1] == 'x' || literal[1] == 'X'){
            num.uint_val = std::strtoull(cstr, nullptr, 16);
            return num;
        }
        if(literal[1] == 'b' || literal[1] == 'B'){
            num.uint_val = 0;
            for(size_t i=2; i<literal.size(); ++i){
                if(literal[i] == '0') num.uint_val <<= 1;
                else if(literal[i] == '1') num.uint_val = (num.uint_val << 1) | 1;
                else break;
            }
            return num;
        }
    }

    if(literal.find('.') != std::string::npos || literal.find('e') != std::string::npos || literal.find('E') != std::string::npos){
        num.float_val = std::strtof(cstr, nullptr);
        num.is_float = true;
        return num;
    }

    num.uint_val = strtoull(cstr, nullptr, 10);
    return num;
}

std::optional<string_view> parse_identifier(const char* ptr) {
    string_view id = { nullptr, nullptr };

    if (!isalpha(*ptr) && *ptr != '_') {
        return std::optional<string_view>{};
    }
    id.begin = ptr;
    id.end = ptr;

    do {
        id.end++;
        if (!isalnum(*id.end) && *id.end != '_') {
            break;
        }
    } while (*id.end);
    return { id };
}

std::optional<RegisterID> parse_register(const char* &ctx) {
    auto rid = parse_identifier(ctx);
    if (!rid.has_value()) {
        return std::optional<RegisterID>{};
    }

    std::string regstr{ rid.value().begin, rid.value().end };
    if (auto iter = s_RegisterMap.find(regstr); iter != s_RegisterMap.end()) {
        ctx = rid.value().end;
        return iter->second;
    }

    return std::optional<RegisterID>{};
}

std::optional<RegisterID> parse_register_deref(CompilerContext& ctx){
    const char* ptr = ctx.input;

    if(*ptr != '['){
        return std::optional<RegisterID>{};
    }

    ptr++;
    consume_whitespace(ptr);

    std::optional<RegisterID> reg = parse_register(ptr);

    if(!reg.has_value()){
        return std::optional<RegisterID>{};
    }

    consume_whitespace(ptr);
    if(*ptr != ']'){
        return std::optional<RegisterID>{};
    }

    ctx.input = ptr+1;
    return reg;
}

std::optional<string_view> parse_identifier_deref(CompilerContext& ctx){
    const char* ptr = ctx.input;

    if(*ptr != '['){
        return std::optional<string_view>{};
    }
    ptr++;
    consume_whitespace(ptr);

    std::optional<string_view> id = parse_identifier(ptr);

    if(!id.has_value()){
        return std::optional<string_view>{};
    }
    ptr = id.value().end;
    consume_whitespace(ptr);

    if(*ptr != ']'){
        return std::optional<string_view>{};
    }
    ctx.input = ptr+1;
    return id;
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

    Command cmd = get_command(ctx.input);
    Instruction& i = ctx.program_section.emplace_back();

    if(!s_InstructionFuncs[static_cast<size_t>(cmd)](ctx, i)){
        return false;
    }

    return true;
}



bool parse_label_export(CompilerContext& ctx) {
    const char* ptr = ctx.input;
    if (!streq(ptr, "@export", 7)) {
        return false;
    }
    ptr += 7;

    consume_whitespace(ptr);

    std::optional<string_view> label = parse_identifier(ptr);
    if(label.has_value()){
        ctx.input = label.value().end;
        ctx.export_labels.emplace_back( label.value().begin, label.value().end );
        return true;
    }

    return false;
}

bool parse_section(CompilerContext& ctx){
    const char* ptr = ctx.input;
    consume_whitespace(ptr);
    if(!streq(ptr, "$section ", 9)){
        return false;
    }
    ptr += 9;
    consume_whitespace(ptr);

    std::optional<string_view> label = parse_identifier(ptr);

    if(!label.has_value()){
        throw std::runtime_error("Expected data,prog,or meta after $section");
    }

    string_view lbl = label.value();

    if(streq(lbl.begin, "meta", 4)){
        ctx.section = CodeSection::Meta;
        ctx.input = lbl.end;
        return true;
    }

    if(streq(lbl.begin, "data", 4)){
        ctx.section = CodeSection::Data;
        ctx.input = lbl.end;
        return true;
    }

    if(streq(lbl.begin, "prog", 4)){
        ctx.section = CodeSection::Program;
        ctx.input = lbl.end;
        return true;
    }

    throw std::runtime_error(std::string("Unknown section type: ") + std::string{lbl.begin, lbl.end});
}

bool parse_label(CompilerContext& ctx){
    const char* ptr = ctx.input;
    consume_whitespace(ptr);

    std::optional<string_view> label = parse_identifier(ptr);

    if(!label.has_value()){
        return false;
    }

    ptr = label.value().end;
    if(*ptr != ':'){
        return false;
    }

    std::string lbl{ label.value().begin, label.value().end };

    if(ctx.section == CodeSection::Program){
        ctx.program_labels[lbl] = static_cast<uint32_t>(ctx.program_section.size());
        ctx.input = ptr + 1;
        return true;
    }

    if(ctx.section == CodeSection::Data){
        ctx.data_labels[lbl] = static_cast<uint32_t>(ctx.data_section.size());
        ctx.input = ptr + 1;
        return true;
    }

    std::cout << "Warning: Unexpected label. Labels are expected in a program or data section\n";
    std::cout << "    " << lbl << " will be ignored.\n";

    ctx.input = ptr + 1;
    return true;
}

bool compile_section(CompilerContext& ctx);


bool compile_program_section(CompilerContext& ctx){

    do {
        consume_ignore_whitespace_and_comments(ctx);

        if(!*ctx.input){
            return true;
        }

        if(parse_label(ctx)){
            continue;
        }

        if(read_and_emit_instruction(ctx)){
           continue;
        }

        return compile_section(ctx);
    } while(true);

}

bool export_str(CompilerContext& ctx){
    consume_whitespace(ctx.input);

    if(*ctx.input != '"'){
        return false;
    }

    ctx.input++;
    bool escaped = false;

    while(*ctx.input){
        char ch = *ctx.input++;

        if(escaped){
            switch(ch){
                case 'n':
                    ctx.data_section.push_back('\n'); break;
                case 't':
                    ctx.data_section.push_back('\t'); break;
                case 'r':
                    ctx.data_section.push_back('\r'); break;
                case '0':
                    ctx.data_section.push_back('\0'); break;
                case '\\':
                    ctx.data_section.push_back('\\'); break;
                default:
                    ctx.data_section.push_back(ch); break;
            }

            escaped = false;
            continue;
        }

        if(ch == '"'){
            ctx.input;
            break;
        }

        if(ch == '\\'){
            escaped = true;
            continue;
        }

        ctx.data_section.push_back(ch);
    }

    ctx.data_section.push_back(0);

    return true;
}

bool compile_data_section(CompilerContext& ctx){
    do {
        consume_ignore_whitespace_and_comments(ctx);

        if(!*ctx.input){
            return true;
        }

        if(parse_label(ctx)){
            continue;
        }

        std::optional<string_view> declarator = parse_identifier(ctx.input);

        if(!declarator.has_value()){
            return compile_section(ctx);
        }

        uint32_t size = 0;
        bool fp = false;

        string_view dec = declarator.value();

        if(streq(dec.begin, "i8", 2)){
            size = 1;
        }
        else if(streq(dec.begin, "i16", 3)){
            size = 2;
        }
        else if(streq(dec.begin, "i32", 3)){
            size = 4;
        }
        else if(streq(dec.begin, "f32", 3)){
            size = 4;
            fp = true;
        }
        else if(streq(dec.begin, "i64", 3)){
            size = 8;
        }
        else if(streq(dec.begin, "str", 3)){
            ctx.input = dec.end;
            if(!export_str(ctx)){
                std::cerr << "Error exporting str data\n";
                return false;
            }
            continue;
        }

        if(size == 0){
            std::cerr << "Unexpected data declaritor: " << std::string{dec.begin, dec.end} << "\n";
            return false;
        }

        ctx.input = dec.end;
        consume_whitespace(ctx.input);
        bool emitted = false;

        do {
            std::optional<string_view> mo = parse_number(ctx.input);
            if(!mo.has_value()){
                break;
            }
            string_view m = mo.value();
            string_view n{};

            ctx.input = m.end;

            if(*m.end == ':'){
                std::optional<string_view> no = parse_number(m.end+1);
                if(!no.has_value()){
                    std::cerr << "Expected number after ':' in data declaritor.\n";
                    return false;
                }
                n = no.value();

                ctx.input = n.end;
            }

            uint8_t buffer[8];

            NumberLiteral num = get_number_literal(m);
            uint32_t count = 1;
            if(n.begin != nullptr){
                NumberLiteral rep = get_number_literal(n);
                if(rep.is_float){
                    std::cerr << "Floating point value cannot be used as a repeater in a data declaritor.\n";
                    return false;
                }
                count = static_cast<uint32_t>(rep.uint_val);
            }

            switch(size){
                case 1: *buffer = static_cast<uint8_t>(num.uint_val); break;
                case 2: *(uint16_t*)buffer = static_cast<uint16_t>(num.uint_val); break;
                case 4: *(uint32_t*)buffer = static_cast<uint32_t>(num.uint_val); break;
                case 8: *(uint64_t*)buffer = num.uint_val; break;
            }

            for(uint32_t i=0; i<count; i++){
                for(uint32_t j=0; j<size; j++){
                    ctx.data_section.push_back(buffer[j]);
                }
            }

            emitted = true;

            if(!*ctx.input){
                return true;
            }

            if(*ctx.input == ','){
                ctx.input++;
                consume_whitespace(ctx.input);
                continue;
            }

            break;
        }
        while(true);
        if(!emitted) return compile_section(ctx);
    } while(true);
}

bool compile_meta_section(CompilerContext& ctx){

    do {
        consume_ignore_whitespace_and_comments(ctx);

        if(!*ctx.input) return true;

        if(parse_label_export(ctx)) continue;

        return compile_section(ctx);
    } while(true);
}

bool link_and_finalize(CompilerContext& ctx){
    uint32_t program_length = static_cast<uint32_t>(ctx.program_section.size()) * sizeof(Instruction);
    for(auto& lp : ctx.unlinked_program_labels){
        uint32_t instructionIdx = lp.first;
        std::string& labelName = lp.second;

        if(auto where = ctx.data_labels.find(labelName); where != ctx.data_labels.end()){
            ctx.program_section[instructionIdx].const_i32 = program_length + where->second;
            continue;
        }

        if(auto where = ctx.program_labels.find(labelName); where != ctx.program_labels.end()){
            ctx.program_section[instructionIdx].const_i32 = where->second * sizeof(Instruction);
            continue;
        }

        std::cerr << "Undefined label: " << labelName << "\n";
        return false;
    }

    return true;
}

bool export_binary(CompilerContext& ctx, const char* filename, bool output_debug_labels){
    std::ofstream stream(filename, std::ios::binary);

    if(!stream.is_open()){
        return false;
    }

    for(const Instruction& instr : ctx.program_section){
        stream.write(reinterpret_cast<const char*>(&instr), sizeof(Instruction));
    }

    for(uint8_t byte : ctx.data_section){
        stream << byte;
    }

    stream.close();

    std::string labelName(filename);
    labelName += ".lnk";

    stream.open(labelName);

    if(!stream.is_open()){
        std::cout << "Error writing link file. Binary was exported but will not be relocatable.\n";
        return false;
    }

    for(uint32_t idx : ctx.instructions_to_relink){
        stream << idx << "\n";
    }


    stream << "!DEBUG\n";
    for(const std::string& exp : ctx.export_labels){

        if(auto where = ctx.program_labels.find(exp); where != ctx.program_labels.end()){
            stream << where->second * sizeof(Instruction) << " " << where->first << "\n";
            continue;
        }

        if(auto where = ctx.data_labels.find(exp); where != ctx.data_labels.end()){
            stream << where->second + (sizeof(Instruction) * ctx.program_section.size()) << " " << where->first << "\n";
            continue;
        }

    }

    stream.close();

    return true;
}

bool compile_section(CompilerContext& ctx) {
    consume_ignore_whitespace_and_comments(ctx);

    if(!*ctx.input){
        return true;
    }

    if(!parse_section(ctx)){
        std::cerr << "Expected section declaration\n";
        return false;
    }

    if(ctx.section == CodeSection::Meta){
        return compile_meta_section(ctx);
    }

    if(ctx.section == CodeSection::Program){
        return compile_program_section(ctx);
    }

    return compile_data_section(ctx);;
}


int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Please provide a source file.\n";
        return 1;
    }

    const char* output_file = "a.out";
    bool output_debug_labels = false;

    for(int i=2; i<argc; i++){
        if(argv[i][0] == '-'){
            if(argv[i][1] == 'o'){
                output_file = (argv[i] + 2);
            }
            else if(argv[i][1] == 'g'){
                output_debug_labels = true;
            }
        }
    }


    std::string output;

    try {
        output = preprocess_input(argv[1]);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    try {
        CompilerContext ctx{0};
        ctx.input = output.c_str();
        std::cout << "Compiling...\n";
        if(!compile_section(ctx)){
            std::cerr << "Error compiling " << argv[1] << "\n";
            return 1;
        }
        std::cout << "Linking...\n";
        if(!link_and_finalize(ctx)){
            std::cerr << "Error linking " << argv[1] << "\n";
            return 1;
        }
        std::cout << "Emitting binary to " << output_file << "\n";
        if(!export_binary(ctx, output_file, output_debug_labels)){
            std::cout << "Error writing to " << output_file << "\n";
            return 1;
        }

        std::cout << "Finished\n";
    }
    catch(const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

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


bool TriggerUndefined(CompilerContext& ctx, Instruction& inst){
    throw std::runtime_error(std::string("Undefined instruction sitting around: ") + std::string{ctx.input, ctx.input+3});
}
bool EmitAbs(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAcos(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAdd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAnd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAsin(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAtan(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitAtan2(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitBcosmxd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitBsinmxd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitBtanmxd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitCeil(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitCmp(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitCmxb(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitCos(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitDec(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitDiv(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitEpow(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitFloor(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitHlt(CompilerContext& ctx, Instruction& inst){
    inst.opcode = 0x010C;
    ctx.input += sizeof("hlt");
    return true;
}
bool EmitInt(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitInc(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitInv(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJmp(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJz(CompilerContext& cxt, Instruction& inst){
    return false;
}
bool EmitJnz(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJgt(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJge(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJlt(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitJle(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLand(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLor(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLnot(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLde(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLog10(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLoge(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLdpi(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitLdmsk(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMov(CompilerContext& ctx, Instruction& inst){

    //TODO: What is a good way to implement this???
    // TODO: Perhaps use a naive pattern matching for this??

    return false;
}
bool EmitMovia(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMovda(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMovc(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMovr(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMcpy(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMcmp(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMclr(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMul(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitMod(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitNop(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitNeg(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitNot(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitOr(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitPush(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitPop(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitPow(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitRbr(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitRbl(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitRet(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSwap(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSub(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSqr(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSqrt(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSin(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitSyscall(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitTsto(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitTld(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitTan(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVnorm(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVadd(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVsub(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVmul(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVdiv(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVdot(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVlen(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitVswz(CompilerContext& ctx, Instruction& inst){
    return false;
}
bool EmitXor(CompilerContext& ctx, Instruction& inst){
    return false;
}
