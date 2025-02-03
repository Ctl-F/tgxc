#!/usr/bin/env python3
import sys, os

"""
Input Format:
    %errfunc [name]
    %mask [num-bytes]
    %mask-enum
    [param-name]
    ...
    %end

    [pneumonic] { [param-name] ... } > opcode {, special_func_name}
"""

def process(input_file, output_file):
    mask_width = 2
    mask_enum = []
    mask_enum_open = False
    pneumonics = {}
    errfunc = ""
    lines = []

    with open(input_file, "r") as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
            continue

        if line[0] == '%':
            if line.startswith('%mask-enum'):
                mask_enum_open = True
                continue
            elif line.startswith('%mask'):
                numstr = line[5:].strip()
                try:
                    mask_width = int(numstr)
                    continue
                except ValueError:
                    print("'%s' is invalid for mask setting.", numstr)
                    exit(1)
            elif line.startswith('%end'):
                mask_enum_open = False
                continue
            elif line.startswith("%errfunc"):
                errfunc = line[8:].strip()
                continue

        if mask_enum_open:
            mask_enum.append(line)
        else:
            parts = line.split('>', 1)
            command = parts[0].strip()
            detail = parts[1].strip() if len(parts) > 1 else None

            command_parts = command.split(' ')

            if not command_parts[0] in pneumonics:
                pneumonics[command_parts[0]] = []

            params = []
            for i in range(1, len(command_parts)):
                if not command_parts[i] in mask_enum:
                    print("ParameterType '%s' is not defined [%s]" % (command_parts[i], command))
                    exit(1)

                params.append(command_parts[i])

            if detail is None:
                print("Expected opcode in an opcode definition [%s]" % command_parts[0])
                exit(1)

            bits = detail.split(',', 1)
            opcode = bits[0].strip()
            special_handle = bits[1].strip() if len(bits) > 1 else "nullptr"

            pneumonics[command_parts[0]].append({ "params": params, "opcode": opcode, "special_handle": special_handle })

    header_str = ["#ifdef INCLUDE_DATA_TABLE"]

    bits = "0b%s" % ("1" * mask_width)
    header_str.append("#define PARAM_TYPE_MASK %s" % bits)

    index = 1
    for me in mask_enum:
        header_str.append("#define PARAM_TYPE_%s %s" % (me, index))
        index += 1

    header_str.append("typedef void(*InstructionHandler)(Instruction& instr, uint32_t paramid, std::vector<string_view>& params);")
    header_str.append("void InvalidOperandParams(Instruction&, uint16_t, uint32_t, std::vector<string_view>&);")

    header_str.append("struct TableEntry { uint16_t opcode; InstructionHandler handler };")

    for pneumonic in pneumonics:
        options = pneumonics[pneumonic]

        options.sort(key=lambda opt: len(opt["params"]))

        for option in options:
            paramid = 0

            for param in option["params"]:
                try:
                    paramval = mask_enum.index(param) + 1
                except ValueError:
                    print("Parameter Type %s is not defined (%s)" % (param, pneumonic))

                paramid = (paramid << mask_width) | paramval

            option["paramid"] = paramid

        options.sort(key=lambda opt: opt["paramid"])

        table_len = "0b%s" % ((("1" * mask_width) * len(options[-1]["params"])) if len(options[-1]["params"]) > 0 else 1)
        header_str.append("#define %s_TABLE_LEN %s" % (pneumonic, table_len))

        header_str.append("TableEntry %s_ParamTable[] {" % pneumonic)

        tlen = int(table_len[2:], 2) if table_len != "0b" else 1

        for i in range(tlen):
            for option in options:
                if option["paramid"] == i:
                    header_str.append("{ %s, %s }," % (option["opcode"], option["special_handle"]))
                    break
            else:
                header_str.append("{ %s, %s }," % ( 0xFFFF, "InvalidOperandParams"))

        header_str.append("};\n")

    header_str.append("#endif //INCLUDE_DATA_TABLE")

    with open(output_file, "w") as f:
        f.write("\n".join(header_str))


def print_help():
    print("codegen.py [input-filename] [output-filename]")


def main(args):
    if len(args) < 2:
        print_help()
        exit(0)

    input_file = args[0]
    output_file = args[1]

    if not os.path.exists(input_file):
        print("Input file '%s' not found." % input_file)
        exit(1)

    if os.path.exists(output_file) and not "-y" in args:
        while True:
            a = input("Output file '%s' already exists, do you want to overwrite? (y/n):" % output_file)

            if len(a) == 0:
                continue

            if a[0] == 'Y' or a[0] == 'y':
                break

            if a[0] == 'N' or a[0] == 'n':
                exit(0)

            print("Invalid input")


    process(input_file, output_file)



if __name__ == "__main__":
    main(sys.argv[1:])