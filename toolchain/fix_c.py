import json
import sys

ignored = [
    '_ZN4core6result13unwrap_failed',
    '_ZN4core9panicking18panic_bounds_check',
    '15copy_from_slice17len_mismatch_fail',
    '_ZN4core5slice5index24slice_end_index_len_fail',
    '_ZN4core3fmt9Formatter3pad',
    'struct l_struct_math_KD__KD_fma_KD__KD_Num _ZN4libm4math3fma9normalize17h31fb36f1da21fd7aE(double);',
    'double _ZN4libm4math5k_sin5k_sin17h136d508b2c4f044aE(double, double, uint32_t);',
    'struct l_unnamed_2 _ZN4libm4math8rem_pio28rem_pio217h96a376aa3583ae1aE(double);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f32_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17he3636a17c0873269E(uint8_t*, uint32_t);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_char_MC__AC__GC_str_JC__OD_ _ZN39__EC_LT_EC_f64_EC_u20_EC_as_EC_u20_EC_num_traits_OC__OC_Num_EC_GT_EC_14from_str_radix16slice_shift_char17ha2628808556e912aE(uint8_t*, uint32_t);',
    '_ZN4core5slice5index22slice_index_order_fail17he351b61fbe6d5e87E',
    'void _ZN13tlsr8266_mesh10main_light20device_status_update17h4034b73cf581c5e5E(void);',
    '** l_fptr_',
    'struct l_struct_embassy_KD__KD_executor_KD__KD_Executor _ZN13tlsr8266_mesh7embassy8executor8Executor3new17h7b21ae0fe2c7f3ffE(void);',
    '_ZN42__EC_LT_EC_str_EC_u20_EC_as_EC_u20_EC_core_OC__OC_fmt_OC__OC_Display_EC_GT_EC_3fmt17h35803dd981c70790E',
    'bool _ZN13tlsr8266_mesh6common23dev_addr_with_mac_match17he287e6c21f2ecb0fE(uint8_t*, uint32_t);',
    '_ZN4core3fmt9Formatter9write_str17h3d8ad84d80640ea1E',
    '_ZN4core9panicking9panic_fmt17h030ddd32bba63c84E',
    '_ZN4core9panicking5panic17h430c2a43a300922eE',
    '_ZN4core6option13expect_failed17hc880daf17d89e424E',
    ' = ((&llvm_cbe_self->field0));'
]

replaced = {
    '((l_fptr_1*)(void*)__atomic_compare_exchange_4)': '__atomic_compare_exchange_4'
}

inputfile = sys.argv[-1]
with open(inputfile + ".rc.c", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        for kind in ignored:
            if kind in line or kind == line:
                line = '// ' + line
                break

        for find, replace in replaced.items():
            if find in line or find == line:
                line = line.replace(find, replace)

                break

        outputfile.writelines([line])
