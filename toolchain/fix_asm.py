import json
import sys

ignored = [
    '\t.cpu\t',
    '\t.syntax unified'
]

replaced = {
    '\t.globl': '\t.global',
    '\t.p2align': '\t.align',
    '\t.thumb_func': '\t.tc32_func',

    # Assembly
    '\tadcs\t': '\ttaddc\t',
    '\tadd\t': '\ttadd\t',
    '\tadds\t': '\ttadd\t',
    '\tands\t': '\ttand\t',
    '\tasrs\t': '\ttasr\t',
    '\tb\t': '\ttj\t',
    '\tbeq\t': '\ttjeq\t',
    '\tbge\t': '\ttjge\t',
    '\tbgt\t': '\ttjgt\t',
    '\tbhi\t': '\ttjhi\t',
    '\tbhs\t': '\ttjhs\t',
    '\tbics\t': '\ttbclr\t',
    '\tbl\t': '\ttjl\t',
    '\tble\t': '\ttjle\t',
    '\tbls\t': '\ttjls\t',
    '\tblt\t': '\ttjlt\t',
    '\tbmi\t': '\ttjmi\t',
    '\tbne\t': '\ttjne\t',
    '\tbpl\t': '\ttjpl\t',
    '\tbx\t': '\ttjex\t',
    '\tblx\t': '\ttjex\t',
    '\tblo\t': '\ttjlo\t',
    '\tbvc\t': '\ttjvc\t',
    '\tbvs\t': '\ttjvs\t',
    '\tcmp\t': '\ttcmp\t',
    '\tcmn\t': '\ttcmpn\t',
    '\tldm\t': '\ttloadm\t',
    '\tldr\t': '\ttloadr\t',
    '\tldrb\t': '\ttloadrb\t',
    '\tldrh\t': '\ttloadrh\t',
    '\tldrsb\t': '\ttloadrsb\t',
    '\tldrsh\t': '\ttloadrsh\t',
    '\tlsls\t': '\ttshftl\t',
    '\tlsrs\t': '\ttshftr\t',
    '\tmovs\t': '\ttmov\t',
    '\tmov\t': '\ttmov\t',
    '\tmuls\t': '\ttmul\t',
    '\tmvns\t': '\ttmovn\t',
    '\torrs\t': '\ttor\t',
    '\tpop\t': '\ttpop\t',
    '\tpush\t': '\ttpush\t',
    '\trors\t': '\ttrotr\t',
    '\tsbcs\t': '\ttsubc\t',
    '\tstm\t': '\ttstorem\t',
    '\tstr\t': '\ttstorer\t',
    '\tstrb\t': '\ttstorerb\t',
    '\tstrh\t': '\ttstorerh\t',
    '\tsub\t': '\ttsub\t',
    '\tsubs\t': '\ttsub\t',
    '\teors\t': '\ttxor\t',

    # Fuck this instruction
    '\tadr\t': '\ttj .\t@',

    # ???
    '\tmrs\t': '\ttmrs\t',
    '\tsev\t': '\t; sev\t'
}

section_seen = False
last_section_name = ".text"
last_section_extra = ""

inputfile = sys.argv[-1]
with open(inputfile + ".tc32", 'w') as outputfile:
    lines = open(inputfile, 'r').readlines()
    for index in range(len(lines)):
        line = lines[index]
        for kind in ignored:
            if kind in line or kind == line:
                line = '@ ' + line
                break

        for find, replace in replaced.items():
            if find in line or find == line:
                line = line.replace(find, replace)

                break

        if '.section' in line:
            section_seen = True
            last_section_name = line.split('\t')[-1].split(',')[0]
            last_section_extra = ',' + ','.join(line.split('\t')[-1].split(',')[1:]).strip()

            if ".ram_code" not in line and '.note' not in line:
                line = "\t@ " + line

        if '.text' == line.strip():
            line = "\t@ " + line
            section_seen = False
            last_section_name = '.text'
            last_section_extra = ""

        if '.data' == line.strip():
            line = "\t@ " + line
            section_seen = False
            last_section_name = '.data'
            last_section_extra = ""

        if '.fnend' in line:
            section_seen = False

        if '.type' in line and ',%object' in line:
            if ".data" in lines[index+1]:
                section_seen = False
                last_section_name = '.data'
                last_section_extra = ""

            if ".comm" in lines[index+2]:
                section_seen = False
                last_section_name = '.data'
                last_section_extra = ""

            if '.section' in lines[index+1]:
                section_seen = True
                last_section_name = lines[index+1].split('\t')[-1].split(',')[0]
                last_section_extra = ',' + ','.join(lines[index+1].split('\t')[-1].split(',')[1:]).strip()

            if last_section_name != '.ram_code':
                name = line.split('\t')[-1].split(',')[0]
                line += f"\t.section {last_section_name}.{name}{last_section_extra}\n"

        if '.type' in line and ',%function' in line and not section_seen:
            if last_section_name != '.ram_code':
                name = line.split('\t')[-1].split(',')[0]
                line += f"\t.section {last_section_name}.{name}{last_section_extra}\n"

        outputfile.writelines([line])
