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
    Bcosmxd, Bsinmxd, Btanmxd, Break,
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
    { .id = RegisterID::M0L, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = false, .emitID =  0 }, // these are generalPurpose = false for the sake of
    { .id = RegisterID::M0H, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = false, .emitID =  0 }, // being able to distinguish them from the other 32bit
    { .id = RegisterID::M1L, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = false, .emitID =  1 }, // registers, as these cannot be used for dereferencing
    { .id = RegisterID::M1H, .type = Type::Integer, .width = 4, .generalPurpose = false, .sysAuto = false, .emitID =  1 }, // directly
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
    {'b', { "bcosmxd", "bsinmxd", "btanmxd", "break", }},
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
    {"add", Command::Add},         {"bcosmxd", Command::Bcosmxd}, {"bsinmxd", Command::Bsinmxd}, { "break", Command::Break },
    {"btanmxd", Command::Btanmxd}, {"ceil", Command::Ceil},       {"cmp", Command::Cmp},
    {"cmxb", Command::Cmxb},       {"cos", Command::Cos},         {"dec", Command::Dec},
    {"div", Command::Div},         {"epow", Command::Epow},       {"floor", Command::Floor},
    {"hlt", Command::Hlt},         {"int", Command::Int},         {"inc", Command::Inc},
    {"inv", Command::Inv},         {"jmp", Command::Jmp},         {"jz", Command::Jz},
    {"jnz", Command::Jnz},         {"jgt", Command::Jgt},         {"jge", Command::Jge},
    {"jlt", Command::Jlt},         {"jle", Command::Jle},         {"land", Command::Land},
    {"lor", Command::Lor},         {"lnot", Command::Lnot},       {"lde", Command::Lde},
    {"log", Command::Log10},       {"ln", Command::Loge},         {"ldpi", Command::Ldpi},
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

static std::unordered_map<Command, std::string> s_CommandNames = {
    {Command::Abs, "abs"},         {Command::Acos, "acos"},       {Command::And, "and"},
    {Command::Asin, "asin"},       {Command::Atan2, "atan2"},     {Command::Atan, "atan"},
    {Command::Add, "add"},         {Command::Bcosmxd, "bcosmxd"}, {Command::Bsinmxd, "bsinmxd"}, { Command::Break, "break" },
    {Command::Btanmxd, "btanmxd"}, {Command::Ceil, "ceil"},       {Command::Cmp, "cmp"},
    {Command::Cmxb, "cmxb"},       {Command::Cos, "cos"},         {Command::Dec, "dec"},
    {Command::Div, "div"},         {Command::Epow, "epow"},       {Command::Floor, "floor"},
    {Command::Hlt, "hlt"},         {Command::Int, "int"},         {Command::Inc, "inc"},
    {Command::Inv, "inv"},         {Command::Jmp, "jmp"},         {Command::Jz, "jz"},
    {Command::Jnz, "jnz"},         {Command::Jgt, "jgt"},         {Command::Jge, "jge"},
    {Command::Jlt, "jlt"},         {Command::Jle, "jle"},         {Command::Land, "land"},
    {Command::Lor, "lor"},         {Command::Lnot, "lnot"},       {Command::Lde, "lde"},
    {Command::Log10, "log"},       {Command::Loge, "ln"},         {Command::Ldpi, "ldpi"},
    {Command::Ldmsk, "ldmsk"},     {Command::Movia, "movia"},     {Command::Movda, "movda"},
    {Command::Movc, "movc"},       {Command::Movr, "movr"},       {Command::Mov, "mov"},
    {Command::Mcpy, "mcpy"},       {Command::Mclr, "mclr"},       {Command::Mcmp, "mcmp"},
    {Command::Mul, "mul"},         {Command::Mod, "mod"},         {Command::Nop, "nop"},
    {Command::Neg, "neg"},         {Command::Not, "not"},         {Command::Or, "or"},
    {Command::Push, "push"},       {Command::Pop, "pop"},         {Command::Pow, "pow"},
    {Command::Rbl, "rbl"},         {Command::Rbr, "rbr"},         {Command::Ret, "ret"},
    {Command::Swap, "swap"},       {Command::Sub, "sub"},         {Command::Sqrt, "sqrt"},
    {Command::Sqr, "sqr"},         {Command::Sin, "sin"},         {Command::Syscall, "syscall"},
    {Command::Tsto, "tsto"},       {Command::Tld, "tld"},         {Command::Tan, "tan"},
    {Command::Vnorm, "vnorm"},     {Command::Vadd, "vadd"},       {Command::Vsub, "vsub"},
    {Command::Vmul, "vmul"},       {Command::Vdiv, "vdiv"},       {Command::Vdot, "vdot"},
    {Command::Vlen, "vlen"},       {Command::Vswz, "vswz"},       {Command::Xor, "xor"},
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

struct NumberLiteral {
    union {
        uint64_t uint_val;
        float float_val;
    };
    bool is_float;
};
std::optional<string_view> parse_number(const char* ptr);
NumberLiteral get_number_literal(string_view str);



#define INCLUDE_DATA_TABLE
#include "optable.inc"

static uint64_t s_Line{0};
static string_view s_CurrentFile{ nullptr, nullptr };
static bool s_DebugMode{false};

std::string get_error_location() {
    return std::string{s_CurrentFile.begin, s_CurrentFile.end} + " " + std::to_string(s_Line) + ": ";
}

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

bool consume_whitespace(const char* &ptr, bool break_on_newline = false) {
    if (*ptr != ' ' && *ptr != '\t' && *ptr != '\n' && *ptr != '\r') return false;

    char c;
    do {
        c = *(ptr++);
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
            ptr--;
            break;
        }
        s_Line += c == '\n';
        if (break_on_newline && c == '\n') {
            break;
        }
    } while (*ptr);
    return true;
}

bool consume_comment(const char* &ptr) {
    if (*ptr == ';') {
        while (*(++ptr)) {
            if (*ptr == '\n') {
                s_Line++;
                ptr++;
                return true;
            }
        }
    }
    else if (*ptr == '#') {
        ++ptr;

        while (*ptr && (*ptr == ' ' || *ptr == '\t')) {
            ptr++;
        }
        std::optional<string_view> num = parse_number(ptr);
        if (!num.has_value()) {
            while (*ptr != '\n') {
                ptr++;
            }
            return true;
        }
        ptr = num.value().end;

        NumberLiteral literal = get_number_literal(num.value());
        s_Line = literal.uint_val ;

        while (*ptr && (*ptr == ' ' || *ptr == '\t')) {
            ptr++;
        }
        if (*ptr == '"') {
            s_CurrentFile.begin = (++ptr);
            while (*ptr != '"') {
                ptr++;
            }
            s_CurrentFile.end = (ptr-1);
            ptr++;
        }
        while (*ptr && *ptr != '\n') {
            ptr++;
        }
        ptr++;
        return true;
    }
    else {
        return false;
    }
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
                ptr += s.length();
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

uint32_t get_param_type(CompilerContext& ctx, string_view view, NumberLiteral& literal) {
    if (streq(view.begin, "i8", 2)) {
        return PARAM_TYPE_Size8;
    }
    if (streq(view.begin, "i16", 3)) {
        return PARAM_TYPE_Size16;
    }
    if (streq(view.begin, "i32", 3)) {
        return PARAM_TYPE_Size32;
    }
    if (streq(view.begin, "i64", 3)) {
        return PARAM_TYPE_Size64;
    }
    if (streq(view.begin, "tbl", 4)) {
        return PARAM_TYPE_Table;
    }

    std::string str{view.begin, view.end};

    if (auto where = s_RegisterMap.find(str); where != s_RegisterMap.end()) {
        RegisterID reg = where->second;
        RegisterInfo rinfo = s_RegisterTable[static_cast<size_t>(reg)];

        switch (rinfo.type) {
            case Type::Integer:

                literal.uint_val = rinfo.emitID;

                switch (rinfo.width) {
                    case 1: return PARAM_TYPE_R8;
                    case 2: return PARAM_TYPE_R16;
                    case 4: {
                        if (*str.rbegin() == 'l') {
                            return PARAM_TYPE_R64L;
                        }
                        if (*str.rbegin() == 'h') {
                            return PARAM_TYPE_R64H;
                        }
                        return PARAM_TYPE_R32;
                    }
                    case 8: return PARAM_TYPE_R64;
                    default: throw std::runtime_error(get_error_location() + "Invalid register width");
                }
            case Type::Float:
                return PARAM_TYPE_F32;
            case Type::Vec:
                return PARAM_TYPE_RVec;
            default:
                throw std::runtime_error(get_error_location() + "Invalid register type");
        }
    }

    if (*str.cbegin() == '[' && *str.crbegin() == ']') {
        std::string reg = str.substr(1, str.length() - 2);
        if (auto where = s_RegisterMap.find(reg); where != s_RegisterMap.end()) {
            auto& info = s_RegisterTable[static_cast<size_t>(where->second)];

            if (info.type != Type::Integer || info.width != 4 || !info.generalPurpose) {
                throw std::runtime_error(get_error_location() + reg + " is not allowed for dereferencing. Only general purpose 32 bit registers are allowed.");
            }

            literal.uint_val = info.emitID;
            return PARAM_TYPE_DeR32;
        }
        throw std::runtime_error(get_error_location() + reg + " is not a known register for dereferencing. Only 32 bit registers are allowed.");
    }

    if (isdigit(*str.begin())) {
        literal = get_number_literal(view);
        return PARAM_TYPE_Imm;
    }

    //double check these label values
    /*if (auto wh = ctx.program_labels.find(str); wh != ctx.program_labels.end()) {
        literal.uint_val = wh->second * sizeof(Instruction);
        return PARAM_TYPE_Imm;
    }

    if (auto wh = ctx.data_labels.find(str); wh != ctx.data_labels.end()) {
        literal.uint_val = wh->second ;
        return PARAM_TYPE_Imm;
    }*/

    ctx.unlinked_program_labels.push_back(std::make_pair(ctx.program_section.size() - 1, str));
    literal.uint_val = 0;
    return PARAM_TYPE_Imm;
}

uint32_t parse_param_ids(CompilerContext& ctx, Instruction& instr, std::vector<string_view>& params) {
    const char* ptr = ctx.input;
    uint32_t paramid = 0;
    int param_count = 0;
    do {
        consume_whitespace(ptr);
        if (!*ptr || *ptr == ';') {
            ctx.input = ptr;
            return paramid;
        }
        string_view param{ptr, ptr};
        char ch = *param.end;
        while (ch && ch != ',' && ch != '\n' && ch != ' ' && ch != '\t' && ch != '\r' && ch != ';') {
            ch = *(param.end++);
        }
        param.end--;

        std::string debug_str{param.begin, param.end};

        params.push_back(param);
        NumberLiteral immediate{};
        uint32_t ptype = get_param_type(ctx, param, immediate);
        paramid = (paramid << PARAM_MASK_WIDTH) | ptype;

        switch (ptype) {
            case PARAM_TYPE_None:
            case PARAM_TYPE_Size8:
            case PARAM_TYPE_Size16:
            case PARAM_TYPE_Size32:
            case PARAM_TYPE_Size64:
            case PARAM_TYPE_Table:
                break;
            case PARAM_TYPE_R32:
            case PARAM_TYPE_R8:
            case PARAM_TYPE_R16:
            case PARAM_TYPE_R64:
            case PARAM_TYPE_RVec:
            case PARAM_TYPE_DeR32:
            case PARAM_TYPE_F32:
            case PARAM_TYPE_R64H:
            case PARAM_TYPE_R64L:
                switch (param_count++) {
                    case 0:
                        instr.params[0] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    case 1:
                        instr.params[1] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    case 2:
                        instr.ext_params[0] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    case 3:
                        instr.ext_params[1] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    case 4:
                        instr.ext_params[2] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    case 5:
                        instr.ext_params[3] = static_cast<uint8_t>(immediate.uint_val);
                        break;
                    default:
                        throw std::runtime_error(get_error_location() + "Too many parameters.");
                }
                break;
            case PARAM_TYPE_Imm:
                instr.const_i32 = static_cast<uint32_t>(immediate.uint_val);
                break;
        }

        ptr = param.end;
        if (*ptr != ',') {
            ctx.input = ptr;
            return paramid;
        }
        ptr++;
    } while (true);
}

bool read_and_emit_instruction(CompilerContext& ctx) {
    consume_ignore_whitespace_and_comments(ctx);

    if (s_DebugMode) {
        Instruction& debugPad = ctx.program_section.emplace_back();
        debugPad.opcode = 0x0000;
        debugPad.const_i32 = static_cast<uint32_t>(s_Line);
    }

    Command cmd = get_command(ctx.input);
    if (cmd == Command::Undefined) {
        throw std::runtime_error("Undefined command");
    }
    std::string& cmdName = s_CommandNames.at(cmd);
    Instruction& i = ctx.program_section.emplace_back();

    std::vector<string_view> params{};
    uint32_t param_id = parse_param_ids(ctx, i, params);

    std::unordered_map<uint32_t, TableEntry> *tbl = nullptr;

    if(auto where = TableMap.find(cmdName); where != TableMap.end()){
        tbl = where->second;
    }

    if(tbl == nullptr){
        std::cerr << get_error_location() << cmdName << " is not a recognized command.\n";
        return false;
    }

    if (auto where = tbl->find(param_id); where != tbl->end()) {
        i.opcode = where->second.opcode;
        if (where->second.handler != nullptr) {
            where->second.handler(i, param_id, params);
        }

        if (!s_DebugMode && cmd == Command::Break) {

        }

        return true;
    }
    InvalidOperandParams(i, param_id, params);
    return false;
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
        throw std::runtime_error(get_error_location() + "Expected data,prog,or meta after $section");
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

    throw std::runtime_error(get_error_location() + "Unknown section type: " + std::string{lbl.begin, lbl.end});
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

        try {
            if(read_and_emit_instruction(ctx)){
                continue;
            }
        }
        catch (std::runtime_error const&) {

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
            //ctx.input; This might be needed+incomplete but I'm not sure
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
        //bool fp = false;

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
            //fp = true;
        }
        else if(streq(dec.begin, "i64", 3)){
            size = 8;
        }
        else if(streq(dec.begin, "str", 3)){
            ctx.input = dec.end;
            if(!export_str(ctx)){
                std::cerr << get_error_location() << "Error exporting str data\n";
                return false;
            }
            continue;
        }

        if(size == 0){
            std::cerr << get_error_location() << "Unexpected data declaritor: " << std::string{dec.begin, dec.end} << "\n";
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
                    std::cerr << get_error_location() << "Expected number after ':' in data declaritor.\n";
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
                    std::cerr << get_error_location() << "Floating point value cannot be used as a repeater in a data declaritor.\n";
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
        throw std::runtime_error(get_error_location() + "Undefined label: " + exp);
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
        std::cerr << get_error_location() << "Expected section declaration\n";
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
            else if (argv[i][1] == 'd') {
                s_DebugMode = true;
            }
        }
    }

    s_CurrentFile.begin = argv[1];
    s_CurrentFile.end = s_CurrentFile.begin + strlen(argv[1]);

    std::string output;

    try {
        output = preprocess_input(argv[1]);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    //std::cout << output << "\n";

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


void InvalidOperandParams(Instruction& instr, uint32_t, std::vector<string_view>& view){
    std::cout << get_error_location() << "Invalid operands for instruction: 0x" << std::hex << instr.opcode << std::dec << "\n";

    for (const string_view& sv : view) {
        std::cout << std::string{sv.begin, sv.end} << ", ";
    }
    std::cout << "\b\b  \n";
}

void HandleMovToTblC8(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.params[1] = instr.params[0];
}
void HandleMovToTblR8(Instruction& instr, uint32_t, std::vector<string_view>&){
    uint8_t t = instr.params[0];
    instr.params[1] = instr.params[0];
    instr.params[0] = t;
}
void HandleMovConst48(Instruction& instr, uint32_t, std::vector<string_view>& params){
    NumberLiteral lit = get_number_literal(*params.rbegin());

    instr.const_i32 = static_cast<uint32_t>(lit.uint_val & 0x00000000FFFFFFFF);
    instr.param16 = static_cast<uint16_t>((lit.uint_val >> 32) & 0xFFFF);

    if (streq(params[0].begin, "m1", 2)) {
        instr.opcode++;
    }
}

void HandleTableLoad(Instruction& instr, uint32_t, std::vector<string_view>&){
    uint8_t t = instr.params[0];
    instr.params[0] = instr.params[1];
    instr.params[1] = t;
}
void HandleMathRegExpand(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.ext_params[0] = instr.params[1];
    instr.params[1] = instr.params[0];
}
void HandleMathImmExpand(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.params[1] = instr.params[0];
}
void HandleElseCode(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.params[1] = TGX_JMP_ELSE_FLAG;
}
void HandleElseCodeReg(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.ext_params[0] = instr.params[1];
    instr.params[1] = TGX_JMP_ELSE_FLAG | TGX_JMP_USE_REG_FLAG;
}
void HandleNoElseCode(Instruction& instr, uint32_t, std::vector<string_view>&){
    instr.ext_params[0] = instr.params[1];
    instr.params[1] = 0;
}
void EncodeBreak(Instruction& instr, uint32_t, std::vector<string_view>&) {
    instr.param16 = static_cast<uint16_t>(instr.const_i32);
    instr.const_i32 = static_cast<uint32_t>(s_Line);
}