#![cfg(target_os = "none")]

//! TC32 reset vector table and low-level startup, formerly `sdk/cstartup_8266.S`.
//!
//! This is a byte-compatible translation of the vendor startup: the image header
//! and the reset/IRQ entry points keep the same offsets and instruction
//! encodings, so the bootloader and the flash layout are unchanged.
//!
//! # Image header (`.vectors`, offsets 0x00-0x1F)
//!
//! | Offset | Contents |
//! |-------:|----------|
//! | `0x00` | `b __reset` (2-byte Thumb branch) |
//! | `0x02` | `BUILD_VERSION`, injected by `ld.lld --defsym` from `rust/src/version.rs` |
//! | `0x08` | `"TLNK"` magic (`0x544c4e4b`) |
//! | `0x0c` | `0x00880120`: RAM destination/entry hint used by the boot ROM |
//! | `0x10` | `b __irq` |
//! | `0x18` | `_bin_size_`: bytes the boot ROM copies from flash |
//! | `0x1c` | `FW_DEV_INFO` (`0x4`) |
//! | `0x20` | `__reset` |
//!
//! # RAM at reset
//!
//! | Range | Use |
//! |---|---|
//! | `0x808000` + `_ramcode_size_` | image prefix copied to SRAM (the rest runs from flash through the cache) |
//! | `0x808600`-`0x8086ff` | IC tag/cache control region written by `SETIC` |
//! | `_start_bss_`-`_end_bss_` | zeroed `.bss`, which contains `irq_stk` |
//! | `irq_stk`..`irq_stk+0x700` | IRQ-mode stack (top written into the IRQ-mode SP) |
//! | `_end_bss_`-`0x80c000` | SVC-mode/main stack, grown down from the top of RAM |
//!
//! # Why five instructions are `.short`
//!
//! The instruction set is standard ARM Thumb-1 (the TC32 binary encoding only
//! permutes the top nibble of each halfword, which the LLVM fork applies when
//! `+tc32` is in the target features) plus five Telink status/IRQ instructions.
//! `rustc`'s embedded LLVM parses `global_asm!` for validation but does not know
//! those five mnemonics, so they are written here as their final TC32 encodings:
//!
//! | instruction | TC32 encoding |
//! |---|---|
//! | `mcsr r0` | `0x6bc0` (write CPSR from r0) |
//! | `mrss r0` | `0x6bd8` (read SPSR into r0) |
//! | `mssr r0` | `0x6bd0` (write SPSR from r0) |
//! | `reti {pc}` | `0x6900` (pop PC and restore CPSR from SPSR) |
//!
//! Everything else stays readable Thumb-1 and is assembled by the fork.
//!
//! # Version word
//!
//! `BUILD_VERSION` has exactly one definition (`rust/src/version.rs`); the build
//! driver passes it to `ld.lld` as `--defsym=BUILD_VERSION=<n>`, so the `.word`
//! in the header resolves at link time.

use core::arch::global_asm;

// IRQ stack. The symbol `irq_stk` is referenced by name from the assembly below,
// so it must keep the default Rust symbol name. It lands in `.bss`, which the
// reset path zeroes before any interrupt can be taken.
#[repr(C, align(4))]
struct IrqStack([u32; 0x700 / 4]);

#[no_mangle]
#[used]
static mut irq_stk: IrqStack = IrqStack([0; 0x700 / 4]);

global_asm!(
    r#"
    /* ==================================================================
     * Constants shared with the linker script (sdk/boot.link)
     * ================================================================== */

    /* Number of 256-byte blocks the boot ROM preloads into SRAM. */
    .equ __LOAD_RAM, 0xc
    .global __LOAD_RAM

    /* Base of the 16 KiB SRAM window. */
    .equ __RAM_START_ADDR, 0x808000
    .global __RAM_START_ADDR

    /* Total SRAM size; the main stack grows down from the top of it. */
    .equ __RAM_SIZE_MAX, (16 * 1024)
    .global __RAM_SIZE_MAX

    /* No separate non-retention RAM region on the 8266. */
    .equ __DATA_ADR_OFFSET, 0
    .global __DATA_ADR_OFFSET

    /* IRQ-mode stack size (matches the rust/src/startup.rs IrqStack array). */
    .equ IRQ_STK_SIZE, 0x700

    /* FW_DEV_INFO: DEV_INFO_USER = 0 in sdk/version.h at the time of writing. */
    .equ FW_DEV_INFO, 0x4

    /* ==================================================================
     * Image header, offsets 0x00-0x1F (see the module docs for the map)
     * ================================================================== */
    .section .vectors,"ax",%progbits
    .code 16
    .global __start
    .type __start,%function
__start:
    b __reset                       /* reset vector */
    .word BUILD_VERSION             /* image version, patched by ld.lld --defsym */
    .org 0x8                        /* bytes 0x06-0x07 are header padding */
    .word 0x544c4e4b                /* "TLNK" */
    .word 0x00880120                /* RAM destination/entry hint */
    .org 0x10
    b __irq                         /* IRQ vector */
    .org 0x18
    .word _bin_size_                /* provided by sdk/boot.link */
    .word FW_DEV_INFO

    /* ==================================================================
     * Reset path
     * ================================================================== */
    .org 0x20
    .type __reset,%function
__reset:
    /* 1. Fill free SRAM with 0xffffffff. The main stack later grows down
     *    from 0x80c000, so the deepest non-0xffffffff word is the stack
     *    high-water mark (used by docs/swire.md and the HIL checks). */
    ldr r0, FLL_D                   /* fill value */
    ldr r1, FLL_D_4                 /* first word of free SRAM */
    ldr r2, FLL_D_8                 /* top of SRAM (exclusive) */
FLL_STK:
    cmp r1, r2
    bge FLL_STK_END
    str r0, [r1, #0]
    adds r1, #4
    b FLL_STK
FLL_STK_END:

    /* 2. Give each ARM mode its own stack before anything can take an
     *    interrupt. `mcsr` writes CPSR, switching the active mode; the
     *    subsequent `mov sp, ...` targets that mode's banked SP. */
    ldr r0, DAT0                    /* CPSR value for IRQ mode (0x12) */
    .short 0x6bc0                   /* mcsr r0: switch to IRQ mode */
    ldr r0, DAT0_8                  /* irq_stk + IRQ_STK_SIZE */
    mov sp, r0
    ldr r0, DAT0_4                  /* CPSR value for SVC mode (0x13) */
    .short 0x6bc0                   /* mcsr r0: back to SVC mode */
    ldr r0, DAT0_12                 /* 0x80c000: top of SRAM */
    mov sp, r0

    /* 3. Zero .bss (globals, executor storage and irq_stk itself). */
    movs r0, #0
    ldr r1, DAT0_16                 /* _start_bss_ */
    ldr r2, DAT0_20                 /* _end_bss_ */
ZERO:
    cmp r1, r2
    bge ZERO_END
    str r0, [r1, #0]
    adds r1, #4
    b ZERO
ZERO_END:

    /* 4. Zero the IC tag/cache region reserved by the linker script. */
    ldr r1, DAT0_28                 /* _ictag_start_ */
    ldr r2, DAT0_32                 /* _ictag_end_ */
ZERO_TAG:
    cmp r1, r2
    bge ZERO_TAG_END
    str r0, [r1, #0]
    adds r1, #4
    b ZERO_TAG
ZERO_TAG_END:

    /* 5. Program the instruction-cache tag register so the preloaded
     *    `_ramcode_size_div_256_` blocks are cached. This is the vendor
     *    sequence: write the block count, then count + 1. */
    ldr r1, DAT0_24                 /* 0x80060c: IC tag control register */
    ldr r0, DAT0_36                 /* _ramcode_size_div_256_ */
    strb r0, [r1, #0]
    adds r0, #1
    strb r0, [r1, #1]

    /* 6. Copy initialised data from its flash load address to its RAM
     *    virtual address (the linker script uses AT() for this). */
    ldr r1, DATA_I                  /* _dstored_: load address in flash */
    ldr r2, DATA_I_4                /* _start_data_: RAM destination */
    ldr r3, DATA_I_8                /* _end_data_ */
COPY_DATA:
    cmp r2, r3
    bge COPY_DATA_END
    ldr r0, [r1, #0]
    str r0, [r2, #0]
    adds r1, #4
    adds r2, #4
    b COPY_DATA
COPY_DATA_END:

    /* 7. Enter Rust. `main_entrypoint` is `#[no_mangle]` in lib.rs. */
    bl main_entrypoint
END:
    b END                           /* main_entrypoint must not return */

    /* ==================================================================
     * Literal pools. Each label is one word, so callers can load a fixed
     * offset without relying on assembler-generated `label + n` syntax.
     * ================================================================== */
    .balign 4
DAT0:                               /* reset-path constants */
DAT0_0:
    .word 0x12                      /* CPSR: IRQ mode */
DAT0_4:
    .word 0x13                      /* CPSR: SVC mode */
DAT0_8:
    .word (irq_stk + IRQ_STK_SIZE)  /* IRQ stack top */
DAT0_12:
    .word (__RAM_START_ADDR + 16 * 1024) /* main stack top (0x80c000) */
DAT0_16:
    .word _start_bss_
DAT0_20:
    .word _end_bss_
DAT0_24:
    .word 0x80060c                  /* IC tag control register */
DAT0_28:
    .word _ictag_start_
DAT0_32:
    .word _ictag_end_
DAT0_36:
    .word _ramcode_size_div_256_
DATA_I:                             /* .data copy window */
DATA_I_0:
    .word _dstored_
DATA_I_4:
    .word _start_data_
DATA_I_8:
    .word _end_data_
FLL_D:                              /* stack high-water fill window */
FLL_D_0:
    .word 0xffffffff
FLL_D_4:
    .word _start_data_              /* first word above .data */
FLL_D_8:
    .word (__RAM_START_ADDR + 16 * 1024)
FLL_D_12:
    .word _rstored_                 /* unused here; kept for layout parity */

    /* ==================================================================
     * IRQ entry/exit.
     *
     * The core has no link-register banking for this: `__irq` must save the
     * interrupted context itself. Saved layout on the IRQ stack (grows down):
     *
     *   lr                      ( 4 bytes)
     *   r0-r7                   (32 bytes)
     *   CPSR/SPSR + r8-r12      (24 bytes)   <- 6 words pushed together
     *
     * That is 60 bytes; `sdk/stack_analysis.py` adds the same preamble when it
     * budgets the IRQ stack. `reti` pops PC and restores CPSR from SPSR in one
     * instruction, which a plain `pop` into PC cannot do.
     * ================================================================== */
    .align 4
    .global __irq
    .type __irq,%function
__irq:
    push {{lr}}                     /* return address */
    push {{r0-r7}}                  /* low registers used by the handler */
    .short 0x6bd8                   /* mrss r0: read saved status (SPSR) */
    mov r1, r8                      /* stash high registers in low ones so */
    mov r2, r9                      /* they can be pushed by Thumb-1 */
    mov r3, r10
    mov r4, r11
    mov r5, r12
    push {{r0-r5}}                  /* saved status + r8-r12 */
    bl irq_handler                  /* Rust handler, plain AAPCS call */
    pop {{r0-r5}}                   /* restore status + r8-r12 */
    mov r8, r1
    mov r9, r2
    mov r10, r3
    mov r11, r4
    mov r12, r5
    .short 0x6bd0                   /* mssr r0: write saved status (SPSR) */
    pop {{r0-r7}}                   /* restore low registers */
    .short 0x6900                   /* reti {{pc}}: pop PC, restore CPSR */

    /* Pad the vector section to 16 bytes, matching the old startup object so
     * the flash layout is unchanged. */
    .balign 16
"#
);
