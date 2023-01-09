import sys
inputfile = sys.argv[-1]
with open(inputfile + ".rc.c", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        if '__attribute_ram_code' in line and '__ATTRIBUTELIST__' in line:
            bits = line.split(';')
            line = bits[0] + ' __attribute__((section(".ram_code")));\n'

        if '_ZN4core9panicking18panic_bounds_check' in line:
            line = '// ' + line

        outputfile.writelines([line])
