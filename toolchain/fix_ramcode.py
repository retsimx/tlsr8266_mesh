import sys

ignored = [
    '_ZN4core6result13unwrap_failed',
    '_ZN4core9panicking18panic_bounds_check',
    '15copy_from_slice17len_mismatch_fail',
    '_ZN4core5slice5index24slice_end_index_len_fail',
    '_ZN4core3fmt9Formatter3pad',
    'struct l_struct_math_KD__KD_fma_KD__KD_Num _ZN4libm4math3fma9normalize17h5e8eb20d4d42e478E(double);',
    'double _ZN4libm4math5k_sin5k_sin17h3494b6221ade2c19E(double, double, uint32_t);',
    'struct l_unnamed_2 _ZN4libm4math8rem_pio28rem_pio217hb10d08a8603bb695E(double);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f32_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17hc8e4ebae52ef0de5E(uint8_t*, uint32_t);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f64_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17hebbc3c59e85a5f00E(uint8_t*, uint32_t);',
    'bool _ZN13tlsr8266_mesh6common23dev_addr_with_mac_match17h65d2e1c9e01ce83aE(uint8_t*, uint32_t);',
    '_ZN4core5slice5index22slice_index_order_fail17he351b61fbe6d5e87E',
    'void _ZN13tlsr8266_mesh10main_light20device_status_update17h4034b73cf581c5e5E(void);',
    '((&llvm_cbe_self->field0))',
    '** l_fptr_',
    'struct l_struct_executor_KD__KD_Executor _ZN13tlsr8266_mesh8executor8Executor3new17h993668f810f035c6E(void);',
    '_ZN42__EC_LT_EC_str_EC_u20_EC_as_EC_u20_EC_core_OC__OC_fmt_OC__OC_Display_EC_GT_EC_3fmt17h35803dd981c70790E'
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
            if kind in line or kind == line:
                line = '// ' + line
                break

        outputfile.writelines([line])
