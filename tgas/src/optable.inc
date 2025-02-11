#ifdef INCLUDE_DATA_TABLE
enum class Command { Undefined, 
Nop,
Mov,
Movc,
Movia,
Movda,
Swap,
Mcpy,
Mcmp,
Mclr,
Push,
Pop,
Tsto,
Tld,
Add,
Sub,
Mul,
Div,
Mod,
Neg,
Inc,
Dec,
Rbr,
Rbl,
Cmp,
Land,
Lor,
Lnot,
And,
Xor,
Or,
Not,
Cmxb,
Sqr,
Abs,
Floor,
Ceil,
Round,
Sqrt,
Vnorm,
Vadd,
Vsub,
Vmul,
Vdiv,
Vdot,
Vlen,
Cos,
Sin,
Tan,
Acos,
Asin,
Atan,
Bcosmxd,
Bsinmxd,
Btanmxd,
Vswz,
Pow,
Log,
Ln,
Epow,
Ldpi,
Lde,
Inv,
Jmp,
Jz,
Jnz,
Jgt,
Jlt,
Jge,
Jle,
Ret,
Hlt,
Int,
Syscall,
Break,
Gqps,
Gqs,
Gqai,
Gqr,
Gqsi,
Gqpc,
Gqpf,
Gqpe,
Trace,
};
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
typedef void(*InstructionHandler)(Instruction& instr, uint32_t paramid, std::vector<string_view>& params);
void InvalidOperandParams(Instruction&, uint32_t, std::vector<string_view>&);
void HandleMovToTblC8(Instruction&, uint32_t, std::vector<string_view>&);
void HandleMovToTblR8(Instruction&, uint32_t, std::vector<string_view>&);
void HandleMovConst48(Instruction&, uint32_t, std::vector<string_view>&);
void HandleTableLoad(Instruction&, uint32_t, std::vector<string_view>&);
void HandleMathRegExpand(Instruction&, uint32_t, std::vector<string_view>&);
void HandleMathImmExpand(Instruction&, uint32_t, std::vector<string_view>&);
void HandleElseCodeReg(Instruction&, uint32_t, std::vector<string_view>&);
void HandleElseCode(Instruction&, uint32_t, std::vector<string_view>&);
void HandleNoElseCode(Instruction&, uint32_t, std::vector<string_view>&);
void EncodeBreak(Instruction&, uint32_t, std::vector<string_view>&);


struct TableEntry { uint16_t opcode; InstructionHandler handler; };

static std::unordered_map<uint32_t, TableEntry> nop_ParamTable {
    { 0x0000, TableEntry{ 0x0000, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> mov_ParamTable {
    { 0x0011, TableEntry{ 0x0001, nullptr } }, /* R32, R32 */
    { 0x0015, TableEntry{ 0x000B, nullptr } }, /* R32, R64H */
    { 0x0016, TableEntry{ 0x000C, nullptr } }, /* R32, R64L */
    { 0x0017, TableEntry{ 0x0016, nullptr } }, /* R32, F32 */
    { 0x001A, TableEntry{ 0x001A, nullptr } }, /* R32, DeR32 */
    { 0x001B, TableEntry{ 0x000F, nullptr } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x0002, nullptr } }, /* R16, R16 */
    { 0x002A, TableEntry{ 0x001C, nullptr } }, /* R16, DeR32 */
    { 0x002B, TableEntry{ 0x0010, nullptr } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x0003, nullptr } }, /* R8, R8 */
    { 0x003A, TableEntry{ 0x001B, nullptr } }, /* R8, DeR32 */
    { 0x003B, TableEntry{ 0x0011, nullptr } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0004, nullptr } }, /* R64, R64 */
    { 0x004A, TableEntry{ 0x001D, nullptr } }, /* R64, DeR32 */
    { 0x004B, TableEntry{ 0x0012, HandleMovConst48 } }, /* R64, Imm */
    { 0x0051, TableEntry{ 0x000D, nullptr } }, /* R64H, R32 */
    { 0x0061, TableEntry{ 0x000E, nullptr } }, /* R64L, R32 */
    { 0x0071, TableEntry{ 0x0018, nullptr } }, /* F32, R32 */
    { 0x0077, TableEntry{ 0x0014, nullptr } }, /* F32, F32 */
    { 0x007A, TableEntry{ 0x0024, nullptr } }, /* F32, DeR32 */
    { 0x007B, TableEntry{ 0x0015, nullptr } }, /* F32, Imm */
    { 0x00A1, TableEntry{ 0x0020, nullptr } }, /* DeR32, R32 */
    { 0x00A2, TableEntry{ 0x001F, nullptr } }, /* DeR32, R16 */
    { 0x00A3, TableEntry{ 0x001E, nullptr } }, /* DeR32, R8 */
    { 0x00A4, TableEntry{ 0x0021, nullptr } }, /* DeR32, R64 */
    { 0x00A7, TableEntry{ 0x0022, nullptr } }, /* DeR32, F32 */
    { 0x00AB, TableEntry{ 0x0005, nullptr } }, /* DeR32, Imm */
    { 0x0193, TableEntry{ 0x000A, nullptr } }, /* R32, Table, R8 */
    { 0x019B, TableEntry{ 0x0009, nullptr } }, /* R32, Table, Imm */
    { 0x0913, TableEntry{ 0x0008, HandleMovToTblR8 } }, /* Table, R32, R8 */
    { 0x091B, TableEntry{ 0x0007, HandleMovToTblC8 } }, /* Table, R32, Imm */
    { 0x0AAB, TableEntry{ 0x0023, nullptr } }, /* DeR32, DeR32, Imm */
    { 0x0ACB, TableEntry{ 0x0006, nullptr } }, /* DeR32, Size8, Imm */
};

static std::unordered_map<uint32_t, TableEntry> movc_ParamTable {
    { 0x0017, TableEntry{ 0x0017, nullptr } }, /* R32, F32 */
    { 0x0071, TableEntry{ 0x0019, nullptr } }, /* F32, R32 */
};

static std::unordered_map<uint32_t, TableEntry> movia_ParamTable {
    { 0x0AAB, TableEntry{ 0x0025, nullptr } }, /* DeR32, DeR32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> movda_ParamTable {
    { 0x0AAB, TableEntry{ 0x0026, nullptr } }, /* DeR32, DeR32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> swap_ParamTable {
    { 0x0011, TableEntry{ 0x0027, nullptr } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x0029, nullptr } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x0028, nullptr } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x002A, nullptr } }, /* R64, R64 */
    { 0x0077, TableEntry{ 0x002B, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> mcpy_ParamTable {
    { 0x0111, TableEntry{ 0x002C, nullptr } }, /* R32, R32, R32 */
};

static std::unordered_map<uint32_t, TableEntry> mcmp_ParamTable {
    { 0x1111, TableEntry{ 0x002D, nullptr } }, /* R32, R32, R32, R32 */
};

static std::unordered_map<uint32_t, TableEntry> mclr_ParamTable {
    { 0x0111, TableEntry{ 0x0030, nullptr } }, /* R32, R32, R32 */
    { 0x0112, TableEntry{ 0x002F, nullptr } }, /* R32, R32, R16 */
    { 0x0113, TableEntry{ 0x002E, nullptr } }, /* R32, R32, R8 */
    { 0x0114, TableEntry{ 0x0031, nullptr } }, /* R32, R32, R64 */
};

static std::unordered_map<uint32_t, TableEntry> push_ParamTable {
    { 0x0000, TableEntry{ 0x0041, nullptr } }, /*  */
    { 0x0001, TableEntry{ 0x0036, nullptr } }, /* R32 */
    { 0x0002, TableEntry{ 0x0034, nullptr } }, /* R16 */
    { 0x0003, TableEntry{ 0x0032, nullptr } }, /* R8 */
    { 0x0004, TableEntry{ 0x0038, nullptr } }, /* R64 */
    { 0x0007, TableEntry{ 0x003A, nullptr } }, /* F32 */
    { 0x000F, TableEntry{ 0x0039, nullptr } }, /* Size64 */
    { 0x00CB, TableEntry{ 0x0033, nullptr } }, /* Size8, Imm */
    { 0x00DB, TableEntry{ 0x0035, nullptr } }, /* Size16, Imm */
    { 0x00EB, TableEntry{ 0x0037, nullptr } }, /* Size32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> pop_ParamTable {
    { 0x0000, TableEntry{ 0x0042, nullptr } }, /*  */
    { 0x0001, TableEntry{ 0x003E, nullptr } }, /* R32 */
    { 0x0002, TableEntry{ 0x003D, nullptr } }, /* R16 */
    { 0x0003, TableEntry{ 0x003C, nullptr } }, /* R8 */
    { 0x0004, TableEntry{ 0x003F, nullptr } }, /* R64 */
    { 0x0007, TableEntry{ 0x0040, nullptr } }, /* F32 */
};

static std::unordered_map<uint32_t, TableEntry> tsto_ParamTable {
    { 0xA933, TableEntry{ 0x0043, nullptr } }, /* DeR32, Table, R8, R8 */
};

static std::unordered_map<uint32_t, TableEntry> tld_ParamTable {
    { 0x9A33, TableEntry{ 0x0044, HandleTableLoad } }, /* Table, DeR32, R8, R8 */
};

static std::unordered_map<uint32_t, TableEntry> add_ParamTable {
    { 0x0011, TableEntry{ 0x0045, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0049, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x0047, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x004B, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x0046, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x004A, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0048, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x004C, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00B6, HandleMathRegExpand } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00B7, HandleMathImmExpand } }, /* F32, Imm */
    { 0x0111, TableEntry{ 0x0045, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0049, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x0047, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x004B, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x0046, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x004A, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0048, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x004C, nullptr } }, /* R64, R64, Imm */
    { 0x0777, TableEntry{ 0x00B6, nullptr } }, /* F32, F32, F32 */
    { 0x077B, TableEntry{ 0x00B7, nullptr } }, /* F32, F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> sub_ParamTable {
    { 0x0011, TableEntry{ 0x004D, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0051, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x004F, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x0053, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x004E, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x0052, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0050, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x0054, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00B8, HandleMathRegExpand } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00B9, HandleMathImmExpand } }, /* F32, Imm */
    { 0x0111, TableEntry{ 0x004D, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0051, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x004F, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x0053, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x004E, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x0052, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0050, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x0054, nullptr } }, /* R64, R64, Imm */
    { 0x0777, TableEntry{ 0x00B8, nullptr } }, /* F32, F32, F32 */
    { 0x077B, TableEntry{ 0x00B9, nullptr } }, /* F32, F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> mul_ParamTable {
    { 0x0011, TableEntry{ 0x0055, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0059, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x0057, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x005B, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x0056, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x005A, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0058, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x005C, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00BA, HandleMathRegExpand } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00BB, HandleMathImmExpand } }, /* F32, Imm */
    { 0x0111, TableEntry{ 0x0055, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0059, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x0057, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x005B, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x0056, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x005A, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0058, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x005C, nullptr } }, /* R64, R64, Imm */
    { 0x0777, TableEntry{ 0x00BA, nullptr } }, /* F32, F32, F32 */
    { 0x077B, TableEntry{ 0x00BB, nullptr } }, /* F32, F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> div_ParamTable {
    { 0x0011, TableEntry{ 0x005D, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0061, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x005F, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x0063, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x005E, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x0062, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0060, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x0064, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00BC, HandleMathRegExpand } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00BD, HandleMathImmExpand } }, /* F32, Imm */
    { 0x0111, TableEntry{ 0x005D, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0061, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x005F, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x0063, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x005E, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x0062, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0060, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x0064, nullptr } }, /* R64, R64, Imm */
    { 0x0777, TableEntry{ 0x00BC, nullptr } }, /* F32, F32, F32 */
    { 0x077B, TableEntry{ 0x00BD, nullptr } }, /* F32, F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> mod_ParamTable {
    { 0x0011, TableEntry{ 0x0065, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0069, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x0067, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x006B, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x0066, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x006A, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0068, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x006C, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00BE, HandleMathRegExpand } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00BF, HandleMathImmExpand } }, /* F32, Imm */
    { 0x0111, TableEntry{ 0x0065, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0069, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x0067, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x006B, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x0066, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x006A, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0068, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x006C, nullptr } }, /* R64, R64, Imm */
    { 0x0777, TableEntry{ 0x00BE, nullptr } }, /* F32, F32, F32 */
    { 0x077B, TableEntry{ 0x00BF, nullptr } }, /* F32, F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> neg_ParamTable {
    { 0x0001, TableEntry{ 0x006D, HandleMathRegExpand } }, /* R32 */
    { 0x0002, TableEntry{ 0x006F, HandleMathRegExpand } }, /* R16 */
    { 0x0003, TableEntry{ 0x006E, HandleMathRegExpand } }, /* R8 */
    { 0x0004, TableEntry{ 0x0070, HandleMathRegExpand } }, /* R64 */
    { 0x0011, TableEntry{ 0x006D, nullptr } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x006F, nullptr } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x006E, nullptr } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x0070, nullptr } }, /* R64, R64 */
    { 0x0077, TableEntry{ 0x00DD, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> inc_ParamTable {
    { 0x0001, TableEntry{ 0x0071, nullptr } }, /* R32 */
    { 0x0002, TableEntry{ 0x0073, nullptr } }, /* R16 */
    { 0x0003, TableEntry{ 0x0072, nullptr } }, /* R8 */
    { 0x0004, TableEntry{ 0x0074, nullptr } }, /* R64 */
};

static std::unordered_map<uint32_t, TableEntry> dec_ParamTable {
    { 0x0001, TableEntry{ 0x0075, nullptr } }, /* R32 */
    { 0x0002, TableEntry{ 0x0077, nullptr } }, /* R16 */
    { 0x0003, TableEntry{ 0x0076, nullptr } }, /* R8 */
    { 0x0004, TableEntry{ 0x0078, nullptr } }, /* R64 */
};

static std::unordered_map<uint32_t, TableEntry> rbr_ParamTable {
    { 0x0011, TableEntry{ 0x0079, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x007D, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x007B, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x007F, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x007A, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x007E, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x007C, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x0080, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0111, TableEntry{ 0x0079, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x007D, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x007B, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x007F, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x007A, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x007E, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x007C, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x0080, nullptr } }, /* R64, R64, Imm */
};

static std::unordered_map<uint32_t, TableEntry> rbl_ParamTable {
    { 0x0011, TableEntry{ 0x0081, HandleMathRegExpand } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x0085, HandleMathImmExpand } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x0083, HandleMathRegExpand } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x0087, HandleMathImmExpand } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x0082, HandleMathRegExpand } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x0086, HandleMathImmExpand } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x0084, HandleMathRegExpand } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x0088, HandleMathImmExpand } }, /* R64, Imm */
    { 0x0111, TableEntry{ 0x0081, nullptr } }, /* R32, R32, R32 */
    { 0x011B, TableEntry{ 0x0085, nullptr } }, /* R32, R32, Imm */
    { 0x0222, TableEntry{ 0x0083, nullptr } }, /* R16, R16, R16 */
    { 0x022B, TableEntry{ 0x0087, nullptr } }, /* R16, R16, Imm */
    { 0x0333, TableEntry{ 0x0082, nullptr } }, /* R8, R8, R8 */
    { 0x033B, TableEntry{ 0x0086, nullptr } }, /* R8, R8, Imm */
    { 0x0444, TableEntry{ 0x0084, nullptr } }, /* R64, R64, R64 */
    { 0x044B, TableEntry{ 0x0088, nullptr } }, /* R64, R64, Imm */
};

static std::unordered_map<uint32_t, TableEntry> cmp_ParamTable {
    { 0x0011, TableEntry{ 0x0089, nullptr } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x008D, nullptr } }, /* R32, Imm */
    { 0x0022, TableEntry{ 0x008B, nullptr } }, /* R16, R16 */
    { 0x002B, TableEntry{ 0x008F, nullptr } }, /* R16, Imm */
    { 0x0033, TableEntry{ 0x008A, nullptr } }, /* R8, R8 */
    { 0x003B, TableEntry{ 0x008E, nullptr } }, /* R8, Imm */
    { 0x0044, TableEntry{ 0x008C, nullptr } }, /* R64, R64 */
    { 0x004B, TableEntry{ 0x0090, nullptr } }, /* R64, Imm */
    { 0x0077, TableEntry{ 0x00C4, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00C5, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> land_ParamTable {
    { 0x0011, TableEntry{ 0x0091, HandleMathRegExpand } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x0093, HandleMathRegExpand } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x0092, HandleMathRegExpand } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x0094, HandleMathRegExpand } }, /* R64, R64 */
    { 0x0111, TableEntry{ 0x0091, nullptr } }, /* R32, R32, R32 */
    { 0x0222, TableEntry{ 0x0093, nullptr } }, /* R16, R16, R16 */
    { 0x0333, TableEntry{ 0x0092, nullptr } }, /* R8, R8, R8 */
    { 0x0444, TableEntry{ 0x0094, nullptr } }, /* R64, R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> lor_ParamTable {
    { 0x0011, TableEntry{ 0x0095, HandleMathRegExpand } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x0097, HandleMathRegExpand } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x0096, HandleMathRegExpand } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x0098, HandleMathRegExpand } }, /* R64, R64 */
    { 0x0111, TableEntry{ 0x0095, nullptr } }, /* R32, R32, R32 */
    { 0x0222, TableEntry{ 0x0097, nullptr } }, /* R16, R16, R16 */
    { 0x0333, TableEntry{ 0x0096, nullptr } }, /* R8, R8, R8 */
    { 0x0444, TableEntry{ 0x0098, nullptr } }, /* R64, R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> lnot_ParamTable {
    { 0x0001, TableEntry{ 0x0099, HandleMathRegExpand } }, /* R32 */
    { 0x0002, TableEntry{ 0x009B, HandleMathRegExpand } }, /* R16 */
    { 0x0003, TableEntry{ 0x009A, HandleMathRegExpand } }, /* R8 */
    { 0x0004, TableEntry{ 0x009C, HandleMathRegExpand } }, /* R64 */
    { 0x0011, TableEntry{ 0x0099, nullptr } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x009B, nullptr } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x009A, nullptr } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x009C, nullptr } }, /* R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> and_ParamTable {
    { 0x0011, TableEntry{ 0x009D, HandleMathRegExpand } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x009F, HandleMathRegExpand } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x009E, HandleMathRegExpand } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x00A0, HandleMathRegExpand } }, /* R64, R64 */
    { 0x0111, TableEntry{ 0x009D, nullptr } }, /* R32, R32, R32 */
    { 0x0222, TableEntry{ 0x009F, nullptr } }, /* R16, R16, R16 */
    { 0x0333, TableEntry{ 0x009E, nullptr } }, /* R8, R8, R8 */
    { 0x0444, TableEntry{ 0x00A0, nullptr } }, /* R64, R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> xor_ParamTable {
    { 0x0011, TableEntry{ 0x00A2, HandleMathRegExpand } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x00A4, HandleMathRegExpand } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x00A3, HandleMathRegExpand } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x00A5, HandleMathRegExpand } }, /* R64, R64 */
    { 0x0111, TableEntry{ 0x00A2, nullptr } }, /* R32, R32, R32 */
    { 0x0222, TableEntry{ 0x00A4, nullptr } }, /* R16, R16, R16 */
    { 0x0333, TableEntry{ 0x00A3, nullptr } }, /* R8, R8, R8 */
    { 0x0444, TableEntry{ 0x00A5, nullptr } }, /* R64, R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> or_ParamTable {
    { 0x0011, TableEntry{ 0x00A6, HandleMathRegExpand } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x00A8, HandleMathRegExpand } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x00A7, HandleMathRegExpand } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x00A9, HandleMathRegExpand } }, /* R64, R64 */
    { 0x0111, TableEntry{ 0x00A6, nullptr } }, /* R32, R32, R32 */
    { 0x0222, TableEntry{ 0x00A8, nullptr } }, /* R16, R16, R16 */
    { 0x0333, TableEntry{ 0x00A7, nullptr } }, /* R8, R8, R8 */
    { 0x0444, TableEntry{ 0x00A9, nullptr } }, /* R64, R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> not_ParamTable {
    { 0x0001, TableEntry{ 0x00AA, HandleMathRegExpand } }, /* R32 */
    { 0x0002, TableEntry{ 0x00AC, HandleMathRegExpand } }, /* R16 */
    { 0x0003, TableEntry{ 0x00AB, HandleMathRegExpand } }, /* R8 */
    { 0x0004, TableEntry{ 0x00AD, HandleMathRegExpand } }, /* R64 */
    { 0x0011, TableEntry{ 0x00AA, nullptr } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x00AC, nullptr } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x00AB, nullptr } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x00AD, nullptr } }, /* R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> cmxb_ParamTable {
    { 0x1111, TableEntry{ 0x00AE, nullptr } }, /* R32, R32, R32, R32 */
    { 0x7777, TableEntry{ 0x00AF, nullptr } }, /* F32, F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> sqr_ParamTable {
    { 0x0011, TableEntry{ 0x00B0, nullptr } }, /* R32, R32 */
    { 0x0044, TableEntry{ 0x00B1, nullptr } }, /* R64, R64 */
};

static std::unordered_map<uint32_t, TableEntry> abs_ParamTable {
    { 0x0001, TableEntry{ 0x00B4, HandleMathRegExpand } }, /* R32 */
    { 0x0002, TableEntry{ 0x00B3, HandleMathRegExpand } }, /* R16 */
    { 0x0003, TableEntry{ 0x00B2, HandleMathRegExpand } }, /* R8 */
    { 0x0004, TableEntry{ 0x00B5, HandleMathRegExpand } }, /* R64 */
    { 0x0011, TableEntry{ 0x00B4, nullptr } }, /* R32, R32 */
    { 0x0022, TableEntry{ 0x00B3, nullptr } }, /* R16, R16 */
    { 0x0033, TableEntry{ 0x00B2, nullptr } }, /* R8, R8 */
    { 0x0044, TableEntry{ 0x00B5, nullptr } }, /* R64, R64 */
    { 0x0077, TableEntry{ 0x00E6, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> floor_ParamTable {
    { 0x0077, TableEntry{ 0x00C0, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> ceil_ParamTable {
    { 0x0077, TableEntry{ 0x00C1, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> round_ParamTable {
    { 0x0077, TableEntry{ 0x00C2, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> sqrt_ParamTable {
    { 0x0077, TableEntry{ 0x00C3, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> vnorm_ParamTable {
    { 0x0008, TableEntry{ 0x00C6, nullptr } }, /* RVec */
};

static std::unordered_map<uint32_t, TableEntry> vadd_ParamTable {
    { 0x0088, TableEntry{ 0x00C7, nullptr } }, /* RVec, RVec */
};

static std::unordered_map<uint32_t, TableEntry> vsub_ParamTable {
    { 0x0088, TableEntry{ 0x00C8, nullptr } }, /* RVec, RVec */
};

static std::unordered_map<uint32_t, TableEntry> vmul_ParamTable {
    { 0x0088, TableEntry{ 0x00C9, nullptr } }, /* RVec, RVec */
};

static std::unordered_map<uint32_t, TableEntry> vdiv_ParamTable {
    { 0x0088, TableEntry{ 0x00CA, nullptr } }, /* RVec, RVec */
};

static std::unordered_map<uint32_t, TableEntry> vdot_ParamTable {
    { 0x0788, TableEntry{ 0x00CB, nullptr } }, /* F32, RVec, RVec */
};

static std::unordered_map<uint32_t, TableEntry> vlen_ParamTable {
    { 0x0078, TableEntry{ 0x00CC, nullptr } }, /* F32, RVec */
};

static std::unordered_map<uint32_t, TableEntry> cos_ParamTable {
    { 0x0077, TableEntry{ 0x00CD, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00CE, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> sin_ParamTable {
    { 0x0077, TableEntry{ 0x00CF, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00D0, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> tan_ParamTable {
    { 0x0077, TableEntry{ 0x00D1, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00D2, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> acos_ParamTable {
    { 0x0077, TableEntry{ 0x00D3, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00D4, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> asin_ParamTable {
    { 0x0077, TableEntry{ 0x00D5, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00D6, nullptr } }, /* F32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> atan_ParamTable {
    { 0x0077, TableEntry{ 0x00D7, nullptr } }, /* F32, F32 */
    { 0x007B, TableEntry{ 0x00D8, nullptr } }, /* F32, Imm */
    { 0x0777, TableEntry{ 0x00D9, nullptr } }, /* F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> bcosmxd_ParamTable {
    { 0x77777, TableEntry{ 0x00DA, nullptr } }, /* F32, F32, F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> bsinmxd_ParamTable {
    { 0x77777, TableEntry{ 0x00DB, nullptr } }, /* F32, F32, F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> btanmxd_ParamTable {
    { 0x77777, TableEntry{ 0x00DC, nullptr } }, /* F32, F32, F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> vswz_ParamTable {
    { 0x0882, TableEntry{ 0x00DE, nullptr } }, /* RVec, RVec, R16 */
};

static std::unordered_map<uint32_t, TableEntry> pow_ParamTable {
    { 0x0777, TableEntry{ 0x00DF, nullptr } }, /* F32, F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> log_ParamTable {
    { 0x0077, TableEntry{ 0x00E0, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> ln_ParamTable {
    { 0x0077, TableEntry{ 0x00E1, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> epow_ParamTable {
    { 0x0077, TableEntry{ 0x00E2, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> ldpi_ParamTable {
    { 0x0007, TableEntry{ 0x00E3, nullptr } }, /* F32 */
};

static std::unordered_map<uint32_t, TableEntry> lde_ParamTable {
    { 0x0007, TableEntry{ 0x00E4, nullptr } }, /* F32 */
};

static std::unordered_map<uint32_t, TableEntry> inv_ParamTable {
    { 0x0077, TableEntry{ 0x00E5, nullptr } }, /* F32, F32 */
};

static std::unordered_map<uint32_t, TableEntry> jmp_ParamTable {
    { 0x0001, TableEntry{ 0x00E7, nullptr } }, /* R32 */
    { 0x000B, TableEntry{ 0x00E8, nullptr } }, /* Imm */
};

static std::unordered_map<uint32_t, TableEntry> jz_ParamTable {
    { 0x0001, TableEntry{ 0x00ED, nullptr } }, /* R32 */
    { 0x000B, TableEntry{ 0x00EA, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00E9, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00E9, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> jnz_ParamTable {
    { 0x0001, TableEntry{ 0x00EE, nullptr } }, /* R32 */
    { 0x000B, TableEntry{ 0x00EC, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00EB, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00EB, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> jgt_ParamTable {
    { 0x0001, TableEntry{ 0x00EF, HandleNoElseCode } }, /* R32 */
    { 0x000B, TableEntry{ 0x00F0, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00EF, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00EF, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> jlt_ParamTable {
    { 0x0001, TableEntry{ 0x00F1, HandleNoElseCode } }, /* R32 */
    { 0x000B, TableEntry{ 0x00F2, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00F1, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00F1, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> jge_ParamTable {
    { 0x0001, TableEntry{ 0x00F3, HandleNoElseCode } }, /* R32 */
    { 0x000B, TableEntry{ 0x00F4, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00F3, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00F3, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> jle_ParamTable {
    { 0x0001, TableEntry{ 0x00F5, HandleNoElseCode } }, /* R32 */
    { 0x000B, TableEntry{ 0x00F6, nullptr } }, /* Imm */
    { 0x0011, TableEntry{ 0x00F5, HandleElseCodeReg } }, /* R32, R32 */
    { 0x001B, TableEntry{ 0x00F5, HandleElseCode } }, /* R32, Imm */
};

static std::unordered_map<uint32_t, TableEntry> ret_ParamTable {
    { 0x0000, TableEntry{ 0x010B, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> hlt_ParamTable {
    { 0x0000, TableEntry{ 0x010C, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> int_ParamTable {
    { 0x0003, TableEntry{ 0x010D, nullptr } }, /* R8 */
    { 0x000B, TableEntry{ 0x010E, nullptr } }, /* Imm */
};

static std::unordered_map<uint32_t, TableEntry> syscall_ParamTable {
    { 0x000B, TableEntry{ 0x0115, nullptr } }, /* Imm */
};

static std::unordered_map<uint32_t, TableEntry> break_ParamTable {
    { 0x0000, TableEntry{ 0x0116, EncodeBreak } }, /*  */
    { 0x000B, TableEntry{ 0x0116, EncodeBreak } }, /* Imm */
};

static std::unordered_map<uint32_t, TableEntry> gqps_ParamTable {
    { 0x0000, TableEntry{ 0x0117, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqs_ParamTable {
    { 0x0000, TableEntry{ 0x0118, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqai_ParamTable {
    { 0x0000, TableEntry{ 0x0119, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqr_ParamTable {
    { 0x0000, TableEntry{ 0x011A, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqsi_ParamTable {
    { 0x0011, TableEntry{ 0x011B, nullptr } }, /* R32, R32 */
};

static std::unordered_map<uint32_t, TableEntry> gqpc_ParamTable {
    { 0x0000, TableEntry{ 0x011C, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqpf_ParamTable {
    { 0x0000, TableEntry{ 0x011D, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> gqpe_ParamTable {
    { 0x0000, TableEntry{ 0x011E, nullptr } }, /*  */
};

static std::unordered_map<uint32_t, TableEntry> trace_ParamTable {
    { 0x000B, TableEntry{ 0x00A1, nullptr } }, /* Imm */
};

static std::unordered_map<std::string, std::unordered_map<uint32_t, TableEntry>*> TableMap {
 { "nop", &nop_ParamTable },
 { "mov", &mov_ParamTable },
 { "movc", &movc_ParamTable },
 { "movia", &movia_ParamTable },
 { "movda", &movda_ParamTable },
 { "swap", &swap_ParamTable },
 { "mcpy", &mcpy_ParamTable },
 { "mcmp", &mcmp_ParamTable },
 { "mclr", &mclr_ParamTable },
 { "push", &push_ParamTable },
 { "pop", &pop_ParamTable },
 { "tsto", &tsto_ParamTable },
 { "tld", &tld_ParamTable },
 { "add", &add_ParamTable },
 { "sub", &sub_ParamTable },
 { "mul", &mul_ParamTable },
 { "div", &div_ParamTable },
 { "mod", &mod_ParamTable },
 { "neg", &neg_ParamTable },
 { "inc", &inc_ParamTable },
 { "dec", &dec_ParamTable },
 { "rbr", &rbr_ParamTable },
 { "rbl", &rbl_ParamTable },
 { "cmp", &cmp_ParamTable },
 { "land", &land_ParamTable },
 { "lor", &lor_ParamTable },
 { "lnot", &lnot_ParamTable },
 { "and", &and_ParamTable },
 { "xor", &xor_ParamTable },
 { "or", &or_ParamTable },
 { "not", &not_ParamTable },
 { "cmxb", &cmxb_ParamTable },
 { "sqr", &sqr_ParamTable },
 { "abs", &abs_ParamTable },
 { "floor", &floor_ParamTable },
 { "ceil", &ceil_ParamTable },
 { "round", &round_ParamTable },
 { "sqrt", &sqrt_ParamTable },
 { "vnorm", &vnorm_ParamTable },
 { "vadd", &vadd_ParamTable },
 { "vsub", &vsub_ParamTable },
 { "vmul", &vmul_ParamTable },
 { "vdiv", &vdiv_ParamTable },
 { "vdot", &vdot_ParamTable },
 { "vlen", &vlen_ParamTable },
 { "cos", &cos_ParamTable },
 { "sin", &sin_ParamTable },
 { "tan", &tan_ParamTable },
 { "acos", &acos_ParamTable },
 { "asin", &asin_ParamTable },
 { "atan", &atan_ParamTable },
 { "bcosmxd", &bcosmxd_ParamTable },
 { "bsinmxd", &bsinmxd_ParamTable },
 { "btanmxd", &btanmxd_ParamTable },
 { "vswz", &vswz_ParamTable },
 { "pow", &pow_ParamTable },
 { "log", &log_ParamTable },
 { "ln", &ln_ParamTable },
 { "epow", &epow_ParamTable },
 { "ldpi", &ldpi_ParamTable },
 { "lde", &lde_ParamTable },
 { "inv", &inv_ParamTable },
 { "jmp", &jmp_ParamTable },
 { "jz", &jz_ParamTable },
 { "jnz", &jnz_ParamTable },
 { "jgt", &jgt_ParamTable },
 { "jlt", &jlt_ParamTable },
 { "jge", &jge_ParamTable },
 { "jle", &jle_ParamTable },
 { "ret", &ret_ParamTable },
 { "hlt", &hlt_ParamTable },
 { "int", &int_ParamTable },
 { "syscall", &syscall_ParamTable },
 { "break", &break_ParamTable },
 { "gqps", &gqps_ParamTable },
 { "gqs", &gqs_ParamTable },
 { "gqai", &gqai_ParamTable },
 { "gqr", &gqr_ParamTable },
 { "gqsi", &gqsi_ParamTable },
 { "gqpc", &gqpc_ParamTable },
 { "gqpf", &gqpf_ParamTable },
 { "gqpe", &gqpe_ParamTable },
 { "trace", &trace_ParamTable },
};

static std::unordered_map<char, std::vector<std::string>> s_Commands = {
    { 'n', { "nop", "neg", "not" } },
    { 'm', { "mov", "movc", "movia", "movda", "mcpy", "mcmp", "mclr", "mul", "mod" } },
    { 's', { "swap", "sub", "sqr", "sqrt", "sin", "syscall" } },
    { 'p', { "push", "pop", "pow" } },
    { 't', { "tsto", "tld", "tan", "trace" } },
    { 'a', { "add", "and", "abs", "acos", "asin", "atan" } },
    { 'd', { "div", "dec" } },
    { 'i', { "inc", "inv", "int" } },
    { 'r', { "rbr", "rbl", "round", "ret" } },
    { 'c', { "cmp", "cmxb", "ceil", "cos" } },
    { 'l', { "land", "lor", "lnot", "log", "ln", "ldpi", "lde" } },
    { 'x', { "xor" } },
    { 'o', { "or" } },
    { 'f', { "floor" } },
    { 'v', { "vnorm", "vadd", "vsub", "vmul", "vdiv", "vdot", "vlen", "vswz" } },
    { 'b', { "bcosmxd", "bsinmxd", "btanmxd", "break" } },
    { 'e', { "epow" } },
    { 'j', { "jmp", "jz", "jnz", "jgt", "jlt", "jge", "jle" } },
    { 'h', { "hlt" } },
    { 'g', { "gqps", "gqs", "gqai", "gqr", "gqsi", "gqpc", "gqpf", "gqpe" } },
};
static std::unordered_map<std::string, Command> s_CommandIDs = {
 { "nop", Command::Nop }, 
 { "mov", Command::Mov }, 
 { "movc", Command::Movc }, 
 { "movia", Command::Movia }, 
 { "movda", Command::Movda }, 
 { "swap", Command::Swap }, 
 { "mcpy", Command::Mcpy }, 
 { "mcmp", Command::Mcmp }, 
 { "mclr", Command::Mclr }, 
 { "push", Command::Push }, 
 { "pop", Command::Pop }, 
 { "tsto", Command::Tsto }, 
 { "tld", Command::Tld }, 
 { "add", Command::Add }, 
 { "sub", Command::Sub }, 
 { "mul", Command::Mul }, 
 { "div", Command::Div }, 
 { "mod", Command::Mod }, 
 { "neg", Command::Neg }, 
 { "inc", Command::Inc }, 
 { "dec", Command::Dec }, 
 { "rbr", Command::Rbr }, 
 { "rbl", Command::Rbl }, 
 { "cmp", Command::Cmp }, 
 { "land", Command::Land }, 
 { "lor", Command::Lor }, 
 { "lnot", Command::Lnot }, 
 { "and", Command::And }, 
 { "xor", Command::Xor }, 
 { "or", Command::Or }, 
 { "not", Command::Not }, 
 { "cmxb", Command::Cmxb }, 
 { "sqr", Command::Sqr }, 
 { "abs", Command::Abs }, 
 { "floor", Command::Floor }, 
 { "ceil", Command::Ceil }, 
 { "round", Command::Round }, 
 { "sqrt", Command::Sqrt }, 
 { "vnorm", Command::Vnorm }, 
 { "vadd", Command::Vadd }, 
 { "vsub", Command::Vsub }, 
 { "vmul", Command::Vmul }, 
 { "vdiv", Command::Vdiv }, 
 { "vdot", Command::Vdot }, 
 { "vlen", Command::Vlen }, 
 { "cos", Command::Cos }, 
 { "sin", Command::Sin }, 
 { "tan", Command::Tan }, 
 { "acos", Command::Acos }, 
 { "asin", Command::Asin }, 
 { "atan", Command::Atan }, 
 { "bcosmxd", Command::Bcosmxd }, 
 { "bsinmxd", Command::Bsinmxd }, 
 { "btanmxd", Command::Btanmxd }, 
 { "vswz", Command::Vswz }, 
 { "pow", Command::Pow }, 
 { "log", Command::Log }, 
 { "ln", Command::Ln }, 
 { "epow", Command::Epow }, 
 { "ldpi", Command::Ldpi }, 
 { "lde", Command::Lde }, 
 { "inv", Command::Inv }, 
 { "jmp", Command::Jmp }, 
 { "jz", Command::Jz }, 
 { "jnz", Command::Jnz }, 
 { "jgt", Command::Jgt }, 
 { "jlt", Command::Jlt }, 
 { "jge", Command::Jge }, 
 { "jle", Command::Jle }, 
 { "ret", Command::Ret }, 
 { "hlt", Command::Hlt }, 
 { "int", Command::Int }, 
 { "syscall", Command::Syscall }, 
 { "break", Command::Break }, 
 { "gqps", Command::Gqps }, 
 { "gqs", Command::Gqs }, 
 { "gqai", Command::Gqai }, 
 { "gqr", Command::Gqr }, 
 { "gqsi", Command::Gqsi }, 
 { "gqpc", Command::Gqpc }, 
 { "gqpf", Command::Gqpf }, 
 { "gqpe", Command::Gqpe }, 
 { "trace", Command::Trace }, 
};
static std::unordered_map<Command, std::string> s_CommandNames = {
 { Command::Nop, "nop" }, 
 { Command::Mov, "mov" }, 
 { Command::Movc, "movc" }, 
 { Command::Movia, "movia" }, 
 { Command::Movda, "movda" }, 
 { Command::Swap, "swap" }, 
 { Command::Mcpy, "mcpy" }, 
 { Command::Mcmp, "mcmp" }, 
 { Command::Mclr, "mclr" }, 
 { Command::Push, "push" }, 
 { Command::Pop, "pop" }, 
 { Command::Tsto, "tsto" }, 
 { Command::Tld, "tld" }, 
 { Command::Add, "add" }, 
 { Command::Sub, "sub" }, 
 { Command::Mul, "mul" }, 
 { Command::Div, "div" }, 
 { Command::Mod, "mod" }, 
 { Command::Neg, "neg" }, 
 { Command::Inc, "inc" }, 
 { Command::Dec, "dec" }, 
 { Command::Rbr, "rbr" }, 
 { Command::Rbl, "rbl" }, 
 { Command::Cmp, "cmp" }, 
 { Command::Land, "land" }, 
 { Command::Lor, "lor" }, 
 { Command::Lnot, "lnot" }, 
 { Command::And, "and" }, 
 { Command::Xor, "xor" }, 
 { Command::Or, "or" }, 
 { Command::Not, "not" }, 
 { Command::Cmxb, "cmxb" }, 
 { Command::Sqr, "sqr" }, 
 { Command::Abs, "abs" }, 
 { Command::Floor, "floor" }, 
 { Command::Ceil, "ceil" }, 
 { Command::Round, "round" }, 
 { Command::Sqrt, "sqrt" }, 
 { Command::Vnorm, "vnorm" }, 
 { Command::Vadd, "vadd" }, 
 { Command::Vsub, "vsub" }, 
 { Command::Vmul, "vmul" }, 
 { Command::Vdiv, "vdiv" }, 
 { Command::Vdot, "vdot" }, 
 { Command::Vlen, "vlen" }, 
 { Command::Cos, "cos" }, 
 { Command::Sin, "sin" }, 
 { Command::Tan, "tan" }, 
 { Command::Acos, "acos" }, 
 { Command::Asin, "asin" }, 
 { Command::Atan, "atan" }, 
 { Command::Bcosmxd, "bcosmxd" }, 
 { Command::Bsinmxd, "bsinmxd" }, 
 { Command::Btanmxd, "btanmxd" }, 
 { Command::Vswz, "vswz" }, 
 { Command::Pow, "pow" }, 
 { Command::Log, "log" }, 
 { Command::Ln, "ln" }, 
 { Command::Epow, "epow" }, 
 { Command::Ldpi, "ldpi" }, 
 { Command::Lde, "lde" }, 
 { Command::Inv, "inv" }, 
 { Command::Jmp, "jmp" }, 
 { Command::Jz, "jz" }, 
 { Command::Jnz, "jnz" }, 
 { Command::Jgt, "jgt" }, 
 { Command::Jlt, "jlt" }, 
 { Command::Jge, "jge" }, 
 { Command::Jle, "jle" }, 
 { Command::Ret, "ret" }, 
 { Command::Hlt, "hlt" }, 
 { Command::Int, "int" }, 
 { Command::Syscall, "syscall" }, 
 { Command::Break, "break" }, 
 { Command::Gqps, "gqps" }, 
 { Command::Gqs, "gqs" }, 
 { Command::Gqai, "gqai" }, 
 { Command::Gqr, "gqr" }, 
 { Command::Gqsi, "gqsi" }, 
 { Command::Gqpc, "gqpc" }, 
 { Command::Gqpf, "gqpf" }, 
 { Command::Gqpe, "gqpe" }, 
 { Command::Trace, "trace" }, 
};
#endif //INCLUDE_DATA_TABLE