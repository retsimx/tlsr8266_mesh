#![cfg(target_os = "none")]

//! TC32 reset vector table and low-level startup, formerly `sdk/cstartup_8266.S`.
//!
//! This is a byte-compatible translation of that file: the vector table keeps
//! the same offsets (`__start` at 0x00, version word at 0x02, magic at 0x08,
//! IRQ branch at 0x10, `_bin_size_` at 0x18, `__reset` at 0x20) so the image
//! header read by the staging bootloader is unchanged.
//!
//! Lives in Rust so `BUILD_VERSION` has exactly one definition
//! (`rust/src/version.rs`); the build driver passes it to `ld.lld` as
//! `--defsym=BUILD_VERSION=<n>`, so the `.word` below resolves at link time.
//!
//! # Why five instructions are `.short`
//!
//! The instruction set is standard ARM Thumb-1 (the TC32 binary encoding only
//! permutes the top nibble of each halfword, which the LLVM fork applies when
//! `+tc32` is in the target features) plus five Telink status/IRQ instructions.
//! `rustc`'s embedded LLVM parses `global_asm!` for validation but does not
//! know those five mnemonics, so they are written here as their final TC32
//! encodings:
//!
//! | instruction | TC32 encoding |
//! |---|---|
//! | `mcsr r0` | `0x6bc0` (write CPSR from r0) |
//! | `mrss r0` | `0x6bd8` (read SPSR into r0) |
//! | `mssr r0` | `0x6bd0` (write SPSR from r0) |
//! | `reti {pc}` | `0x6900` (pop PC and restore CPSR from SPSR) |
//!
//! Everything else stays readable Thumb-1 and is assembled by the fork.

use core::arch::global_asm;

// IRQ stack. The symbol `irq_stk` is referenced by name from the assembly below,
// so it must keep the default Rust symbol name.
#[repr(C, align(4))]
struct IrqStack([u32; 0x700 / 4]);

#[no_mangle]
#[used]
static mut irq_stk: IrqStack = IrqStack([0; 0x700 / 4]);

global_asm!(
    r#"
    .equ __LOAD_RAM, 0xc
    .global __LOAD_RAM
    .equ __RAM_START_ADDR, 0x808000
    .global __RAM_START_ADDR
    .equ __RAM_SIZE_MAX, (16 * 1024)
    .global __RAM_SIZE_MAX
    .equ __DATA_ADR_OFFSET, 0
    .global __DATA_ADR_OFFSET
    .equ IRQ_STK_SIZE, 0x700
    .equ FW_DEV_INFO, 0x4

    .section .vectors,"ax",%progbits
    .code 16
    .global __start
    .type __start,%function
__start:
    b __reset
    .word BUILD_VERSION
    .org 0x8
    .word 0x544c4e4b
    .word 0x00880120
    .org 0x10
    b __irq
    .org 0x18
    .word _bin_size_
    .word FW_DEV_INFO

    .org 0x20
    .type __reset,%function
__reset:
    ldr r0, FLL_D
    ldr r1, FLL_D_4
    ldr r2, FLL_D_8
FLL_STK:
    cmp r1, r2
    bge FLL_STK_END
    str r0, [r1, #0]
    adds r1, #4
    b FLL_STK
FLL_STK_END:
    ldr r0, DAT0
    .short 0x6bc0            /* mcsr r0: select IRQ stack */
    ldr r0, DAT0_8
    mov sp, r0
    ldr r0, DAT0_4
    .short 0x6bc0            /* mcsr r0: back to SVC mode */
    ldr r0, DAT0_12
    mov sp, r0

    movs r0, #0
    ldr r1, DAT0_16
    ldr r2, DAT0_20
ZERO:
    cmp r1, r2
    bge ZERO_END
    str r0, [r1, #0]
    adds r1, #4
    b ZERO
ZERO_END:
    ldr r1, DAT0_28
    ldr r2, DAT0_32
ZERO_TAG:
    cmp r1, r2
    bge ZERO_TAG_END
    str r0, [r1, #0]
    adds r1, #4
    b ZERO_TAG
ZERO_TAG_END:
    ldr r1, DAT0_24
    ldr r0, DAT0_36
    strb r0, [r1, #0]
    adds r0, #1
    strb r0, [r1, #1]

    ldr r1, DATA_I
    ldr r2, DATA_I_4
    ldr r3, DATA_I_8
COPY_DATA:
    cmp r2, r3
    bge COPY_DATA_END
    ldr r0, [r1, #0]
    str r0, [r2, #0]
    adds r1, #4
    adds r2, #4
    b COPY_DATA
COPY_DATA_END:
    bl main_entrypoint
END:
    b END

    .balign 4
DAT0:
DAT0_0:
    .word 0x12
DAT0_4:
    .word 0x13
DAT0_8:
    .word (irq_stk + IRQ_STK_SIZE)
DAT0_12:
    .word (__RAM_START_ADDR + 16 * 1024)
DAT0_16:
    .word _start_bss_
DAT0_20:
    .word _end_bss_
DAT0_24:
    .word 0x80060c
DAT0_28:
    .word _ictag_start_
DAT0_32:
    .word _ictag_end_
DAT0_36:
    .word _ramcode_size_div_256_
DATA_I:
DATA_I_0:
    .word _dstored_
DATA_I_4:
    .word _start_data_
DATA_I_8:
    .word _end_data_
FLL_D:
FLL_D_0:
    .word 0xffffffff
FLL_D_4:
    .word _start_data_
FLL_D_8:
    .word (__RAM_START_ADDR + 16 * 1024)
FLL_D_12:
    .word _rstored_

    .align 4
    .global __irq
    .type __irq,%function
__irq:
    push {{lr}}
    push {{r0-r7}}
    .short 0x6bd8            /* mrss r0: read saved status */
    mov r1, r8
    mov r2, r9
    mov r3, r10
    mov r4, r11
    mov r5, r12
    push {{r0-r5}}
    bl irq_handler
    pop {{r0-r5}}
    mov r8, r1
    mov r9, r2
    mov r10, r3
    mov r11, r4
    mov r12, r5
    .short 0x6bd0            /* mssr r0: restore saved status */
    pop {{r0-r7}}
    .short 0x6900            /* reti {{pc}} */

    /* Pad the vector section to 16 bytes, matching the old startup object so
       the flash layout is unchanged. */
    .balign 16
"#
);
