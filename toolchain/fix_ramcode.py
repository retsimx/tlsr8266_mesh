import sys

ignored = [
    '_ZN4core6result13unwrap_failed',
    '_ZN4core9panicking18panic_bounds_check',
    '15copy_from_slice17len_mismatch_fail',
    '_ZN4core5slice5index24slice_end_index_len_fail',
    '_ZN4core3fmt9Formatter3pad',
    'struct l_struct_math_KD__KD_fma_KD__KD_Num _ZN4libm4math3fma9normalize17h614a23dc5f280940E(double);',
    'double _ZN4libm4math5k_sin5k_sin17h743f95627beab611E(double, double, uint32_t);',
    'struct l_unnamed_2 _ZN4libm4math8rem_pio28rem_pio217h40920cee4af4bef4E(double);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f32_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17he08e093b84df50afE(uint8_t*, uint32_t);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f64_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17h8acdf53a12542ba2E(uint8_t*, uint32_t);']

inputfile = sys.argv[-1]
with open(inputfile + ".rc.c", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        if '__attribute_ram_code' in line and '__ATTRIBUTELIST__' in line:
            bits = line.split(';')
            line = bits[0] + ' __attribute__((section(".ram_code")));\n'

        if '__attribute_ram_code' in line:
            line = line.replace('__attribute_ram_code', '')

        for kind in ignored:
            if kind in line or kind == line:
                line = '// ' + line
                break

        outputfile.writelines([line])
