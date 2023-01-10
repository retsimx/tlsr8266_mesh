import sys

ignored = [
    '_ZN4core6result13unwrap_failed',
    '_ZN4core9panicking18panic_bounds_check',
    '15copy_from_slice17len_mismatch_fail',
    '_ZN4core5slice5index24slice_end_index_len_fail',
    '_ZN4core3fmt9Formatter3pad'
]

inputfile = sys.argv[-1]
with open(inputfile + ".rc.c", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        if '__attribute_ram_code' in line and '__ATTRIBUTELIST__' in line:
            bits = line.split(';')
            line = bits[0] + ' __attribute__((section(".ram_code")));\n'

        if '__attribute_ram_code' in line:
            line = line.replace('__attribute_ram_code', '')

        for kind in ignored:
            if kind in line:
                line = '// ' + line
                break

        outputfile.writelines([line])
