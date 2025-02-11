#ifdef INCLUDE_DATA_TABLE
typedef struct { uint16_t opcode; uint32_t param_id; const char* name; } Mnemonic;

#define DISASSEMBLER_TABLE_LEN 279
static Mnemonic s_DisassemblerTable[] = {
    (Mnemonic){ .opcode = 0x0000, .param_id = 0, .name = "nop" },
    (Mnemonic){ .opcode = 0x0001, .param_id = 17, .name = "mov" },
    (Mnemonic){ .opcode = 0x0002, .param_id = 34, .name = "mov" },
    (Mnemonic){ .opcode = 0x0003, .param_id = 51, .name = "mov" },
    (Mnemonic){ .opcode = 0x0004, .param_id = 68, .name = "mov" },
    (Mnemonic){ .opcode = 0x0005, .param_id = 171, .name = "mov" },
    (Mnemonic){ .opcode = 0x0006, .param_id = 2763, .name = "mov" },
    (Mnemonic){ .opcode = 0x0007, .param_id = 2331, .name = "mov" },
    (Mnemonic){ .opcode = 0x0008, .param_id = 2323, .name = "mov" },
    (Mnemonic){ .opcode = 0x0009, .param_id = 411, .name = "mov" },
    (Mnemonic){ .opcode = 0x000A, .param_id = 403, .name = "mov" },
    (Mnemonic){ .opcode = 0x000B, .param_id = 21, .name = "mov" },
    (Mnemonic){ .opcode = 0x000C, .param_id = 22, .name = "mov" },
    (Mnemonic){ .opcode = 0x000D, .param_id = 81, .name = "mov" },
    (Mnemonic){ .opcode = 0x000E, .param_id = 97, .name = "mov" },
    (Mnemonic){ .opcode = 0x000F, .param_id = 27, .name = "mov" },
    (Mnemonic){ .opcode = 0x0010, .param_id = 43, .name = "mov" },
    (Mnemonic){ .opcode = 0x0011, .param_id = 59, .name = "mov" },
    (Mnemonic){ .opcode = 0x0012, .param_id = 75, .name = "mov" },
    (Mnemonic){ .opcode = 0x0014, .param_id = 119, .name = "mov" },
    (Mnemonic){ .opcode = 0x0015, .param_id = 123, .name = "mov" },
    (Mnemonic){ .opcode = 0x0016, .param_id = 23, .name = "mov" },
    (Mnemonic){ .opcode = 0x0017, .param_id = 23, .name = "movc" },
    (Mnemonic){ .opcode = 0x0018, .param_id = 113, .name = "mov" },
    (Mnemonic){ .opcode = 0x0019, .param_id = 113, .name = "movc" },
    (Mnemonic){ .opcode = 0x001A, .param_id = 26, .name = "mov" },
    (Mnemonic){ .opcode = 0x001B, .param_id = 58, .name = "mov" },
    (Mnemonic){ .opcode = 0x001C, .param_id = 42, .name = "mov" },
    (Mnemonic){ .opcode = 0x001D, .param_id = 74, .name = "mov" },
    (Mnemonic){ .opcode = 0x001E, .param_id = 163, .name = "mov" },
    (Mnemonic){ .opcode = 0x001F, .param_id = 162, .name = "mov" },
    (Mnemonic){ .opcode = 0x0020, .param_id = 161, .name = "mov" },
    (Mnemonic){ .opcode = 0x0021, .param_id = 164, .name = "mov" },
    (Mnemonic){ .opcode = 0x0022, .param_id = 167, .name = "mov" },
    (Mnemonic){ .opcode = 0x0023, .param_id = 2731, .name = "mov" },
    (Mnemonic){ .opcode = 0x0024, .param_id = 122, .name = "mov" },
    (Mnemonic){ .opcode = 0x0025, .param_id = 2731, .name = "movia" },
    (Mnemonic){ .opcode = 0x0026, .param_id = 2731, .name = "movda" },
    (Mnemonic){ .opcode = 0x0027, .param_id = 17, .name = "swap" },
    (Mnemonic){ .opcode = 0x0028, .param_id = 51, .name = "swap" },
    (Mnemonic){ .opcode = 0x0029, .param_id = 34, .name = "swap" },
    (Mnemonic){ .opcode = 0x002A, .param_id = 68, .name = "swap" },
    (Mnemonic){ .opcode = 0x002B, .param_id = 119, .name = "swap" },
    (Mnemonic){ .opcode = 0x002C, .param_id = 273, .name = "mcpy" },
    (Mnemonic){ .opcode = 0x002D, .param_id = 4369, .name = "mcmp" },
    (Mnemonic){ .opcode = 0x002E, .param_id = 275, .name = "mclr" },
    (Mnemonic){ .opcode = 0x002F, .param_id = 274, .name = "mclr" },
    (Mnemonic){ .opcode = 0x0030, .param_id = 273, .name = "mclr" },
    (Mnemonic){ .opcode = 0x0031, .param_id = 276, .name = "mclr" },
    (Mnemonic){ .opcode = 0x0032, .param_id = 3, .name = "push" },
    (Mnemonic){ .opcode = 0x0033, .param_id = 203, .name = "push" },
    (Mnemonic){ .opcode = 0x0034, .param_id = 2, .name = "push" },
    (Mnemonic){ .opcode = 0x0035, .param_id = 219, .name = "push" },
    (Mnemonic){ .opcode = 0x0036, .param_id = 1, .name = "push" },
    (Mnemonic){ .opcode = 0x0037, .param_id = 235, .name = "push" },
    (Mnemonic){ .opcode = 0x0038, .param_id = 4, .name = "push" },
    (Mnemonic){ .opcode = 0x0039, .param_id = 15, .name = "push" },
    (Mnemonic){ .opcode = 0x003A, .param_id = 7, .name = "push" },
    (Mnemonic){ .opcode = 0x003C, .param_id = 3, .name = "pop" },
    (Mnemonic){ .opcode = 0x003D, .param_id = 2, .name = "pop" },
    (Mnemonic){ .opcode = 0x003E, .param_id = 1, .name = "pop" },
    (Mnemonic){ .opcode = 0x003F, .param_id = 4, .name = "pop" },
    (Mnemonic){ .opcode = 0x0040, .param_id = 7, .name = "pop" },
    (Mnemonic){ .opcode = 0x0041, .param_id = 0, .name = "push" },
    (Mnemonic){ .opcode = 0x0042, .param_id = 0, .name = "pop" },
    (Mnemonic){ .opcode = 0x0043, .param_id = 43315, .name = "tsto" },
    (Mnemonic){ .opcode = 0x0044, .param_id = 39475, .name = "tld" },
    (Mnemonic){ .opcode = 0x0045, .param_id = 273, .name = "add" },
    (Mnemonic){ .opcode = 0x0046, .param_id = 819, .name = "add" },
    (Mnemonic){ .opcode = 0x0047, .param_id = 546, .name = "add" },
    (Mnemonic){ .opcode = 0x0048, .param_id = 1092, .name = "add" },
    (Mnemonic){ .opcode = 0x0049, .param_id = 283, .name = "add" },
    (Mnemonic){ .opcode = 0x004A, .param_id = 827, .name = "add" },
    (Mnemonic){ .opcode = 0x004B, .param_id = 555, .name = "add" },
    (Mnemonic){ .opcode = 0x004C, .param_id = 1099, .name = "add" },
    (Mnemonic){ .opcode = 0x004D, .param_id = 273, .name = "sub" },
    (Mnemonic){ .opcode = 0x004E, .param_id = 819, .name = "sub" },
    (Mnemonic){ .opcode = 0x004F, .param_id = 546, .name = "sub" },
    (Mnemonic){ .opcode = 0x0050, .param_id = 1092, .name = "sub" },
    (Mnemonic){ .opcode = 0x0051, .param_id = 283, .name = "sub" },
    (Mnemonic){ .opcode = 0x0052, .param_id = 827, .name = "sub" },
    (Mnemonic){ .opcode = 0x0053, .param_id = 555, .name = "sub" },
    (Mnemonic){ .opcode = 0x0054, .param_id = 1099, .name = "sub" },
    (Mnemonic){ .opcode = 0x0055, .param_id = 273, .name = "mul" },
    (Mnemonic){ .opcode = 0x0056, .param_id = 819, .name = "mul" },
    (Mnemonic){ .opcode = 0x0057, .param_id = 546, .name = "mul" },
    (Mnemonic){ .opcode = 0x0058, .param_id = 1092, .name = "mul" },
    (Mnemonic){ .opcode = 0x0059, .param_id = 283, .name = "mul" },
    (Mnemonic){ .opcode = 0x005A, .param_id = 827, .name = "mul" },
    (Mnemonic){ .opcode = 0x005B, .param_id = 555, .name = "mul" },
    (Mnemonic){ .opcode = 0x005C, .param_id = 1099, .name = "mul" },
    (Mnemonic){ .opcode = 0x005D, .param_id = 273, .name = "div" },
    (Mnemonic){ .opcode = 0x005E, .param_id = 819, .name = "div" },
    (Mnemonic){ .opcode = 0x005F, .param_id = 546, .name = "div" },
    (Mnemonic){ .opcode = 0x0060, .param_id = 1092, .name = "div" },
    (Mnemonic){ .opcode = 0x0061, .param_id = 283, .name = "div" },
    (Mnemonic){ .opcode = 0x0062, .param_id = 827, .name = "div" },
    (Mnemonic){ .opcode = 0x0063, .param_id = 555, .name = "div" },
    (Mnemonic){ .opcode = 0x0064, .param_id = 1099, .name = "div" },
    (Mnemonic){ .opcode = 0x0065, .param_id = 273, .name = "mod" },
    (Mnemonic){ .opcode = 0x0066, .param_id = 819, .name = "mod" },
    (Mnemonic){ .opcode = 0x0067, .param_id = 546, .name = "mod" },
    (Mnemonic){ .opcode = 0x0068, .param_id = 1092, .name = "mod" },
    (Mnemonic){ .opcode = 0x0069, .param_id = 283, .name = "mod" },
    (Mnemonic){ .opcode = 0x006A, .param_id = 827, .name = "mod" },
    (Mnemonic){ .opcode = 0x006B, .param_id = 555, .name = "mod" },
    (Mnemonic){ .opcode = 0x006C, .param_id = 1099, .name = "mod" },
    (Mnemonic){ .opcode = 0x006D, .param_id = 17, .name = "neg" },
    (Mnemonic){ .opcode = 0x006E, .param_id = 51, .name = "neg" },
    (Mnemonic){ .opcode = 0x006F, .param_id = 34, .name = "neg" },
    (Mnemonic){ .opcode = 0x0070, .param_id = 68, .name = "neg" },
    (Mnemonic){ .opcode = 0x0071, .param_id = 1, .name = "inc" },
    (Mnemonic){ .opcode = 0x0072, .param_id = 3, .name = "inc" },
    (Mnemonic){ .opcode = 0x0073, .param_id = 2, .name = "inc" },
    (Mnemonic){ .opcode = 0x0074, .param_id = 4, .name = "inc" },
    (Mnemonic){ .opcode = 0x0075, .param_id = 1, .name = "dec" },
    (Mnemonic){ .opcode = 0x0076, .param_id = 3, .name = "dec" },
    (Mnemonic){ .opcode = 0x0077, .param_id = 2, .name = "dec" },
    (Mnemonic){ .opcode = 0x0078, .param_id = 4, .name = "dec" },
    (Mnemonic){ .opcode = 0x0079, .param_id = 273, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007A, .param_id = 819, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007B, .param_id = 546, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007C, .param_id = 1092, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007D, .param_id = 283, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007E, .param_id = 827, .name = "rbr" },
    (Mnemonic){ .opcode = 0x007F, .param_id = 555, .name = "rbr" },
    (Mnemonic){ .opcode = 0x0080, .param_id = 1099, .name = "rbr" },
    (Mnemonic){ .opcode = 0x0081, .param_id = 273, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0082, .param_id = 819, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0083, .param_id = 546, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0084, .param_id = 1092, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0085, .param_id = 283, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0086, .param_id = 827, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0087, .param_id = 555, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0088, .param_id = 1099, .name = "rbl" },
    (Mnemonic){ .opcode = 0x0089, .param_id = 17, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008A, .param_id = 51, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008B, .param_id = 34, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008C, .param_id = 68, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008D, .param_id = 27, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008E, .param_id = 59, .name = "cmp" },
    (Mnemonic){ .opcode = 0x008F, .param_id = 43, .name = "cmp" },
    (Mnemonic){ .opcode = 0x0090, .param_id = 75, .name = "cmp" },
    (Mnemonic){ .opcode = 0x0091, .param_id = 273, .name = "land" },
    (Mnemonic){ .opcode = 0x0092, .param_id = 819, .name = "land" },
    (Mnemonic){ .opcode = 0x0093, .param_id = 546, .name = "land" },
    (Mnemonic){ .opcode = 0x0094, .param_id = 1092, .name = "land" },
    (Mnemonic){ .opcode = 0x0095, .param_id = 273, .name = "lor" },
    (Mnemonic){ .opcode = 0x0096, .param_id = 819, .name = "lor" },
    (Mnemonic){ .opcode = 0x0097, .param_id = 546, .name = "lor" },
    (Mnemonic){ .opcode = 0x0098, .param_id = 1092, .name = "lor" },
    (Mnemonic){ .opcode = 0x0099, .param_id = 17, .name = "lnot" },
    (Mnemonic){ .opcode = 0x009A, .param_id = 51, .name = "lnot" },
    (Mnemonic){ .opcode = 0x009B, .param_id = 34, .name = "lnot" },
    (Mnemonic){ .opcode = 0x009C, .param_id = 68, .name = "lnot" },
    (Mnemonic){ .opcode = 0x009D, .param_id = 273, .name = "and" },
    (Mnemonic){ .opcode = 0x009E, .param_id = 819, .name = "and" },
    (Mnemonic){ .opcode = 0x009F, .param_id = 546, .name = "and" },
    (Mnemonic){ .opcode = 0x00A0, .param_id = 1092, .name = "and" },
    (Mnemonic){ .opcode = 0x00A1, .param_id = 11, .name = "trace" },
    (Mnemonic){ .opcode = 0x00A2, .param_id = 273, .name = "xor" },
    (Mnemonic){ .opcode = 0x00A3, .param_id = 819, .name = "xor" },
    (Mnemonic){ .opcode = 0x00A4, .param_id = 546, .name = "xor" },
    (Mnemonic){ .opcode = 0x00A5, .param_id = 1092, .name = "xor" },
    (Mnemonic){ .opcode = 0x00A6, .param_id = 273, .name = "or" },
    (Mnemonic){ .opcode = 0x00A7, .param_id = 819, .name = "or" },
    (Mnemonic){ .opcode = 0x00A8, .param_id = 546, .name = "or" },
    (Mnemonic){ .opcode = 0x00A9, .param_id = 1092, .name = "or" },
    (Mnemonic){ .opcode = 0x00AA, .param_id = 17, .name = "not" },
    (Mnemonic){ .opcode = 0x00AB, .param_id = 51, .name = "not" },
    (Mnemonic){ .opcode = 0x00AC, .param_id = 34, .name = "not" },
    (Mnemonic){ .opcode = 0x00AD, .param_id = 68, .name = "not" },
    (Mnemonic){ .opcode = 0x00AE, .param_id = 4369, .name = "cmxb" },
    (Mnemonic){ .opcode = 0x00AF, .param_id = 30583, .name = "cmxb" },
    (Mnemonic){ .opcode = 0x00B0, .param_id = 17, .name = "sqr" },
    (Mnemonic){ .opcode = 0x00B1, .param_id = 68, .name = "sqr" },
    (Mnemonic){ .opcode = 0x00B2, .param_id = 51, .name = "abs" },
    (Mnemonic){ .opcode = 0x00B3, .param_id = 34, .name = "abs" },
    (Mnemonic){ .opcode = 0x00B4, .param_id = 17, .name = "abs" },
    (Mnemonic){ .opcode = 0x00B5, .param_id = 68, .name = "abs" },
    (Mnemonic){ .opcode = 0x00B6, .param_id = 1911, .name = "add" },
    (Mnemonic){ .opcode = 0x00B7, .param_id = 1915, .name = "add" },
    (Mnemonic){ .opcode = 0x00B8, .param_id = 1911, .name = "sub" },
    (Mnemonic){ .opcode = 0x00B9, .param_id = 1915, .name = "sub" },
    (Mnemonic){ .opcode = 0x00BA, .param_id = 1911, .name = "mul" },
    (Mnemonic){ .opcode = 0x00BB, .param_id = 1915, .name = "mul" },
    (Mnemonic){ .opcode = 0x00BC, .param_id = 1911, .name = "div" },
    (Mnemonic){ .opcode = 0x00BD, .param_id = 1915, .name = "div" },
    (Mnemonic){ .opcode = 0x00BE, .param_id = 1911, .name = "mod" },
    (Mnemonic){ .opcode = 0x00BF, .param_id = 1915, .name = "mod" },
    (Mnemonic){ .opcode = 0x00C0, .param_id = 119, .name = "floor" },
    (Mnemonic){ .opcode = 0x00C1, .param_id = 119, .name = "ceil" },
    (Mnemonic){ .opcode = 0x00C2, .param_id = 119, .name = "round" },
    (Mnemonic){ .opcode = 0x00C3, .param_id = 119, .name = "sqrt" },
    (Mnemonic){ .opcode = 0x00C4, .param_id = 119, .name = "cmp" },
    (Mnemonic){ .opcode = 0x00C5, .param_id = 123, .name = "cmp" },
    (Mnemonic){ .opcode = 0x00C6, .param_id = 8, .name = "vnorm" },
    (Mnemonic){ .opcode = 0x00C7, .param_id = 136, .name = "vadd" },
    (Mnemonic){ .opcode = 0x00C8, .param_id = 136, .name = "vsub" },
    (Mnemonic){ .opcode = 0x00C9, .param_id = 136, .name = "vmul" },
    (Mnemonic){ .opcode = 0x00CA, .param_id = 136, .name = "vdiv" },
    (Mnemonic){ .opcode = 0x00CB, .param_id = 1928, .name = "vdot" },
    (Mnemonic){ .opcode = 0x00CC, .param_id = 120, .name = "vlen" },
    (Mnemonic){ .opcode = 0x00CD, .param_id = 119, .name = "cos" },
    (Mnemonic){ .opcode = 0x00CE, .param_id = 123, .name = "cos" },
    (Mnemonic){ .opcode = 0x00CF, .param_id = 119, .name = "sin" },
    (Mnemonic){ .opcode = 0x00D0, .param_id = 123, .name = "sin" },
    (Mnemonic){ .opcode = 0x00D1, .param_id = 119, .name = "tan" },
    (Mnemonic){ .opcode = 0x00D2, .param_id = 123, .name = "tan" },
    (Mnemonic){ .opcode = 0x00D3, .param_id = 119, .name = "acos" },
    (Mnemonic){ .opcode = 0x00D4, .param_id = 123, .name = "acos" },
    (Mnemonic){ .opcode = 0x00D5, .param_id = 119, .name = "asin" },
    (Mnemonic){ .opcode = 0x00D6, .param_id = 123, .name = "asin" },
    (Mnemonic){ .opcode = 0x00D7, .param_id = 119, .name = "atan" },
    (Mnemonic){ .opcode = 0x00D8, .param_id = 123, .name = "atan" },
    (Mnemonic){ .opcode = 0x00D9, .param_id = 1911, .name = "atan" },
    (Mnemonic){ .opcode = 0x00DA, .param_id = 489335, .name = "bcosmxd" },
    (Mnemonic){ .opcode = 0x00DB, .param_id = 489335, .name = "bsinmxd" },
    (Mnemonic){ .opcode = 0x00DC, .param_id = 489335, .name = "btanmxd" },
    (Mnemonic){ .opcode = 0x00DD, .param_id = 119, .name = "neg" },
    (Mnemonic){ .opcode = 0x00DE, .param_id = 2178, .name = "vswz" },
    (Mnemonic){ .opcode = 0x00DF, .param_id = 1911, .name = "pow" },
    (Mnemonic){ .opcode = 0x00E0, .param_id = 119, .name = "log" },
    (Mnemonic){ .opcode = 0x00E1, .param_id = 119, .name = "ln" },
    (Mnemonic){ .opcode = 0x00E2, .param_id = 119, .name = "epow" },
    (Mnemonic){ .opcode = 0x00E3, .param_id = 7, .name = "ldpi" },
    (Mnemonic){ .opcode = 0x00E4, .param_id = 7, .name = "lde" },
    (Mnemonic){ .opcode = 0x00E5, .param_id = 119, .name = "inv" },
    (Mnemonic){ .opcode = 0x00E6, .param_id = 119, .name = "abs" },
    (Mnemonic){ .opcode = 0x00E7, .param_id = 1, .name = "jmp" },
    (Mnemonic){ .opcode = 0x00E8, .param_id = 11, .name = "jmp" },
    (Mnemonic){ .opcode = 0x00E9, .param_id = 27, .name = "jz" },
    (Mnemonic){ .opcode = 0x00EA, .param_id = 11, .name = "jz" },
    (Mnemonic){ .opcode = 0x00EB, .param_id = 27, .name = "jnz" },
    (Mnemonic){ .opcode = 0x00EC, .param_id = 11, .name = "jnz" },
    (Mnemonic){ .opcode = 0x00ED, .param_id = 1, .name = "jz" },
    (Mnemonic){ .opcode = 0x00EE, .param_id = 1, .name = "jnz" },
    (Mnemonic){ .opcode = 0x00EF, .param_id = 27, .name = "jgt" },
    (Mnemonic){ .opcode = 0x00F0, .param_id = 11, .name = "jgt" },
    (Mnemonic){ .opcode = 0x00F1, .param_id = 27, .name = "jlt" },
    (Mnemonic){ .opcode = 0x00F2, .param_id = 11, .name = "jlt" },
    (Mnemonic){ .opcode = 0x00F3, .param_id = 27, .name = "jge" },
    (Mnemonic){ .opcode = 0x00F4, .param_id = 11, .name = "jge" },
    (Mnemonic){ .opcode = 0x00F5, .param_id = 27, .name = "jle" },
    (Mnemonic){ .opcode = 0x00F6, .param_id = 11, .name = "jle" },
    (Mnemonic){ .opcode = 0x00F7, .param_id = 44051, .name = "mov" },
    (Mnemonic){ .opcode = 0x00F8, .param_id = 44050, .name = "mov" },
    (Mnemonic){ .opcode = 0x00F9, .param_id = 44049, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FA, .param_id = 44052, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FB, .param_id = 44055, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FC, .param_id = 15041, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FD, .param_id = 10945, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FE, .param_id = 6849, .name = "mov" },
    (Mnemonic){ .opcode = 0x00FF, .param_id = 19137, .name = "mov" },
    (Mnemonic){ .opcode = 0x0100, .param_id = 31425, .name = "mov" },
    (Mnemonic){ .opcode = 0x0101, .param_id = 44307, .name = "mov" },
    (Mnemonic){ .opcode = 0x0102, .param_id = 44306, .name = "mov" },
    (Mnemonic){ .opcode = 0x0103, .param_id = 44305, .name = "mov" },
    (Mnemonic){ .opcode = 0x0104, .param_id = 44308, .name = "mov" },
    (Mnemonic){ .opcode = 0x0105, .param_id = 44311, .name = "mov" },
    (Mnemonic){ .opcode = 0x0106, .param_id = 15057, .name = "mov" },
    (Mnemonic){ .opcode = 0x0107, .param_id = 10961, .name = "mov" },
    (Mnemonic){ .opcode = 0x0108, .param_id = 6865, .name = "mov" },
    (Mnemonic){ .opcode = 0x0109, .param_id = 19153, .name = "mov" },
    (Mnemonic){ .opcode = 0x010A, .param_id = 31441, .name = "mov" },
    (Mnemonic){ .opcode = 0x010B, .param_id = 0, .name = "ret" },
    (Mnemonic){ .opcode = 0x010C, .param_id = 0, .name = "hlt" },
    (Mnemonic){ .opcode = 0x010D, .param_id = 3, .name = "int" },
    (Mnemonic){ .opcode = 0x010E, .param_id = 11, .name = "int" },
    (Mnemonic){ .opcode = 0x0115, .param_id = 11, .name = "syscall" },
    (Mnemonic){ .opcode = 0x0116, .param_id = 11, .name = "break" },
    (Mnemonic){ .opcode = 0x0117, .param_id = 0, .name = "gqps" },
    (Mnemonic){ .opcode = 0x0118, .param_id = 0, .name = "gqs" },
    (Mnemonic){ .opcode = 0x0119, .param_id = 0, .name = "gqai" },
    (Mnemonic){ .opcode = 0x011A, .param_id = 0, .name = "gqr" },
    (Mnemonic){ .opcode = 0x011B, .param_id = 17, .name = "gqsi" },
    (Mnemonic){ .opcode = 0x011C, .param_id = 0, .name = "gqpc" },
    (Mnemonic){ .opcode = 0x011D, .param_id = 0, .name = "gqpf" },
    (Mnemonic){ .opcode = 0x011E, .param_id = 0, .name = "gqpe" },
};

static Mnemonic* SearchOpCode(uint16_t opcode){
    for(int i=0; i<DISASSEMBLER_TABLE_LEN; ++i){

        if(s_DisassemblerTable[i].opcode == opcode) return (s_DisassemblerTable + i);

    }
    return NULL;
}
#endif //INCLUDE_DATA_TABLE