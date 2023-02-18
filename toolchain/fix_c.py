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
    'struct l_struct_embassy_KD__KD_executor_KD__KD_Executor _ZN13tlsr8266_mesh7embassy8executor8Executor3new17h52b324020b9e5653E(void);',
    '_ZN42__EC_LT_EC_str_EC_u20_EC_as_EC_u20_EC_core_OC__OC_fmt_OC__OC_Display_EC_GT_EC_3fmt17h35803dd981c70790E',
    'bool _ZN13tlsr8266_mesh6common23dev_addr_with_mac_match17hb725466d08c6dd65E(uint8_t*, uint32_t);',
    '_ZN4core3fmt9Formatter9write_str17h3d8ad84d80640ea1E',
    '_ZN4core9panicking9panic_fmt17h030ddd32bba63c84E',
    '_ZN4core9panicking5panic17h430c2a43a300922eE',
    '_ZN4core6option13expect_failed17hc880daf17d89e424E',
    ' = ((&llvm_cbe_self->field0));',
    'uint128_t _ZN4core3num22__EC_LT_EC_impl_EC_u20_EC_u128_EC_GT_EC_3pow17h08818e8967395174E(uint128_t, uint32_t);',
    'uint8_t _ZN5fixed5bytes5Bytes5index17h135674f650bf8d73E(uint8_t*, uint32_t, uint32_t);',
    'struct l_unnamed_9 _ZN5fixed5bytes5Bytes8split_at17he1a0a3584bd9312dE(uint8_t*, uint32_t, uint32_t);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_u8_MC__AC_bytes_KD__KD_Bytes_JC__OD_ _ZN5fixed5bytes5Bytes11split_first17he1ba8c629709073dE(uint8_t*, uint32_t);',
    'struct l_struct_core_KD__KD_option_KD__KD_Option_MD__IC_bytes_KD__KD_Bytes_MC__AC_u8_JC__OD_ _ZN5fixed5bytes5Bytes10split_last17he3e3d784e84dc11aE(uint8_t*, uint32_t);',
    'struct l_unnamed_7 _ZN5fixed6int25617div_rem_u256_u12817h40abebaac126f157E(uint128_t, uint128_t, uint128_t);',
    'struct l_struct_fixed_KD__KD_helpers_KD__KD_ToFixedHelper _ZN5fixed10int_helper3i3215to_fixed_helper17hd76013291750ecf8E(uint32_t, uint32_t, uint32_t, uint32_t);',
    'struct l_struct_fixed_KD__KD_helpers_KD__KD_ToFixedHelper _ZN70__EC_LT_EC_fixed_OC__OC_FixedI32_EC_LT_EC_Frac_EC_GT_EC__EC_u20_EC_as_EC_u20_EC_fixed_OC__OC_helpers_OC__OC_Sealed_EC_GT_EC_15to_fixed_helper17h60cc98c8df494b6aE(uint32_t, uint32_t, uint32_t);'
]

replaced = {
    '((l_fptr_1*)(void*)__atomic_compare_exchange_4)': '__atomic_compare_exchange_4',
    'uint128_t': 'uint64_t',
    'int128_t': 'int64_t'
}

inputfile = sys.argv[-1]
with open(inputfile + ".rc.c", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        for kind in ignored:
            if kind in line or kind == line:
                line = '// ' + line
                break

        for find, replace in replaced.items():
            while find in line or find == line:
                line = line.replace(find, replace)

                break

        outputfile.writelines([line])
