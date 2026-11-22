#![cfg(target_os = "none")]

//! TC32 reset vector table, pre-cache stub and the IRQ entry thunk.
//!
//! Only three things must be assembly:
//!
//! * the **image header** at fixed offsets `0x00`-`0x1f`;
//! * the **pre-cache stub**: mode-stack setup (`mcsr`, TC32-only), the stack
//!   watermark fill, and the instruction-cache tag programming, all of which
//!   must run from the SRAM prefix before any flash-resident code executes;
//! * the **IRQ frame**, which needs `mrss`/`mssr` and `reti`.
//!
//! Everything else is Rust in [`startup_init`]: it clears `.bss` and copies
//! `.data` using the compiler's `memclr`/`memcpy` helpers.
//!
//! The watermark fill deliberately stays in the stub: it paints up to the top
//! of SRAM, which is exactly where the stack lives, so it must run before the
//! core uses its stack at all. A Rust callee doing the fill would overwrite its
//! own frame.
//!
//! # Image header (`.vectors`, offsets `0x00`-`0x1f`)
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
//! # Why four instructions are raw `.short` values
//!
//! The instruction set is standard ARM Thumb-1 (the TC32 binary encoding only
//! permutes the top nibble of each halfword, which the LLVM fork applies when
//! `+tc32` is in the target features) plus five Telink status/IRQ instructions.
//! `rustc`'s embedded LLVM validates `global_asm!` but does not know those five
//! mnemonics, so they are written as their final TC32 encodings:
//!
//! | instruction | TC32 encoding |
//! |---|---|
//! | `mcsr r0` | `0x6bc0` (write CPSR from r0) |
//! | `mrss r0` | `0x6bd8` (read SPSR into r0) |
//! | `mssr r0` | `0x6bd0` (write SPSR from r0) |
//! | `reti {pc}` | `0x6900` (pop PC and restore CPSR from SPSR) |

use core::arch::global_asm;
use core::ptr::{copy_nonoverlapping, write_bytes};

// IRQ stack. The symbol `irq_stk` is referenced by name from the assembly below,
// so it must keep the default Rust symbol name. It lives in `.bss`, which
// `startup_init` zeroes before interrupts are enabled. The `UnsafeCell` records
// that the memory is mutated behind Rust's back (by the hardware stack pointer),
// which is what lets this be a plain `static` instead of a `static mut`.
#[repr(C, align(4))]
struct IrqStack(core::cell::UnsafeCell<[u32; 0x700 / 4]>);

// SAFETY: (4) `Sync` wrapper. The hardware IRQ stack is written only by
// interrupt entry/exit assembly through the stack pointer. Rust code never
// reads or writes the bytes, and only its address is ever taken (by
// `global_asm!` and by `probe-tlsr diag`), so no `&IrqStack` can alias live
// stack memory.
unsafe impl Sync for IrqStack {}

#[no_mangle]
#[used]
static irq_stk: IrqStack = IrqStack(core::cell::UnsafeCell::new([0; 0x700 / 4]));

// The assembler reserves the same stack with `.equ IRQ_STK_SIZE, 0x700`
// (`irq_stk + IRQ_STK_SIZE` is loaded as the initial `sp` below); this keeps the
// Rust array and the asm constant from drifting apart silently.
const _: () = assert!(core::mem::size_of::<IrqStack>() == 0x700);

// Absolute symbols provided by `sdk/boot.link`. Taking the address of one
// yields its numeric value.
// SAFETY: (2) linker-provided ABI. These symbols are defined by
// `sdk/boot.link`; none is ever dereferenced - only their addresses are taken
// through `symbol` - so no storage behind the declaration is assumed.
unsafe extern "C" {
    static _dstored_: u8;
    static _start_data_: u8;
    static _end_data_: u8;
    static _start_bss_: u8;
    static _end_bss_: u8;
}

#[inline(always)]
// SAFETY: (2) linker-symbol ABI. Every caller passes one of the extern statics
// above, so `sym` is a linker address and the cast yields it unchanged; the
// result is used only as a raw start/end pointer.
unsafe fn symbol(sym: &u8) -> usize {
    sym as *const u8 as usize
}

/// Finish SRAM bring-up and return to the reset stub.
///
/// Runs with the cache programmed and before `.data` exists; it uses no
/// statics, only raw pointers and the compiler's own `memclr`/`memcpy`
/// helpers.
#[no_mangle]
pub extern "C" fn startup_init() {
    // SAFETY: (2) startup memory bring-up. `startup_init` runs from the reset stub
    // before any Rust code, task or interrupt exists; the ranges are the linker
    // script's symbols and `symbol` maps each to its address, so the `.bss` zeroing
    // and `.data` copy touch exactly the linker-declared regions.
    unsafe {
        // 1. Zero `.bss` (Rust globals, executor storage, `irq_stk`).
        write_bytes(
            symbol(&_start_bss_) as *mut u8,
            0,
            symbol(&_end_bss_) - symbol(&_start_bss_),
        );

        // 2. Copy initialised data from its flash load address to its RAM VMA.
        copy_nonoverlapping(
            symbol(&_dstored_) as *const u8,
            symbol(&_start_data_) as *mut u8,
            symbol(&_end_data_) - symbol(&_start_data_),
        );
    }
}

global_asm!(
    r#"
    /* =====================================================================
     * Linker-shared constants (consumed by sdk/boot.link)
     * ===================================================================== */

    /* 256-byte blocks the boot ROM preloads into SRAM. */
    .equ __LOAD_RAM, 0xc
    .global __LOAD_RAM

    /* Base and size of the SRAM window. */
    .equ __RAM_START_ADDR, 0x808000
    .global __RAM_START_ADDR
    .equ __RAM_SIZE_MAX, (16 * 1024)
    .global __RAM_SIZE_MAX

    /* The 8266 has no separate non-retention RAM region. */
    .equ __DATA_ADR_OFFSET, 0
    .global __DATA_ADR_OFFSET

    /* Must match the IrqStack array above. */
    .equ IRQ_STK_SIZE, 0x700

    /* DEV_INFO_USER is 0, so the header carries only the mode nibble. */
    .equ FW_DEV_INFO, 0x4

    /* =====================================================================
     * Image header: fixed offsets 0x00-0x1f (see the module docs)
     * ===================================================================== */
    .section .vectors,"ax",%progbits
    .code 16
    .global __start
    .type __start,%function
__start:
    b __reset                       /* 0x00: reset vector */
    .word BUILD_VERSION             /* 0x02: patched at link time (--defsym) */
    .org 0x8                        /* 0x06-0x07: header padding */
    .word 0x544c4e4b                /* 0x08: "TLNK" */
    .word 0x00880120                /* 0x0c: RAM destination/entry hint */
    .org 0x10
    b __irq                         /* 0x10: IRQ vector */
    .org 0x18
    .word _bin_size_                /* 0x18: flash bytes to copy (boot.link) */
    .word FW_DEV_INFO               /* 0x1c */

    /* =====================================================================
     * Pre-cache stub. Runs from the SRAM prefix; it installs both mode
     * stacks, paints the stack watermark, and clears + programs the
     * instruction cache before calling flash-resident Rust.
     * ===================================================================== */
    .org 0x20
    .type __reset,%function
__reset:
    /* 1. Install both mode stacks. The IRQ bank must be valid before
     *    interrupts can be taken. `mcsr` selects the mode; `mov sp` then
     *    writes that mode's banked SP. */
    movs r0, #0x12                  /* CPSR: IRQ mode, IRQs masked */
    .short 0x6bc0                   /* mcsr r0 */
    ldr r0, .Lirq_stack_top         /* irq_stk + IRQ_STK_SIZE */
    mov sp, r0

    movs r0, #0x13                  /* CPSR: SVC mode, IRQs masked */
    .short 0x6bc0                   /* mcsr r0 */
    ldr r0, .Lram_top               /* 0x80c000 */
    mov sp, r0

    /* 2. Paint the free SRAM the main stack grows into. The deepest
     *    non-0xffffffff word is the stack high-water mark read by
     *    docs/swire.md and the HIL checks. This must happen before the core
     *    uses its stack: the window ends at the stack top. */
    movs r0, #0
    mvns r0, r0                     /* 0xffffffff */
    ldr r1, .Lstack_fill_start
    ldr r2, .Lram_top
.Lfill_free_sram:
    cmp r1, r2
    bge .Lfill_free_sram_done
    str r0, [r1, #0]
    adds r1, #4
    b .Lfill_free_sram
.Lfill_free_sram_done:

    /* 3. Clear the cache-tag region. */
    movs r0, #0
    ldr r1, .Lic_tag_start
    ldr r2, .Lic_tag_end
.Lzero_ic_tag:
    cmp r1, r2
    bge .Lzero_ic_tag_done
    str r0, [r1, #0]
    adds r1, #4
    b .Lzero_ic_tag
.Lzero_ic_tag_done:

    /* 4. Program the cache tag register: block count, then count + 1. */
    ldr r1, .Lic_tag_register       /* 0x80060c */
    ldr r0, .Lramcode_blocks        /* _ramcode_size_div_256_ */
    strb r0, [r1, #0]
    adds r0, #1
    strb r0, [r1, #1]

    /* 5. Hand the rest to Rust, which now runs from cache-backed flash. */
    bl startup_init                 /* clear .bss, copy .data */
    bl main_entrypoint              /* may never return */
.Lhalt:
    b .Lhalt

    /* Stub literal pool: one word per constant, each with its own label so
     * the code never depends on assembler `label + n` syntax. */
    .balign 4
.Lirq_stack_top:
    .word (irq_stk + IRQ_STK_SIZE)
.Lram_top:
    .word (__RAM_START_ADDR + 16 * 1024)
.Lstack_fill_start:
    .word _end_bss_
.Lic_tag_register:
    .word 0x80060c
.Lic_tag_start:
    .word _ictag_start_
.Lic_tag_end:
    .word _ictag_end_
.Lramcode_blocks:
    .word _ramcode_size_div_256_

    /* =====================================================================
     * IRQ entry and exit.
     *
     * `irq_handler` is an ordinary AAPCS function, so it and everything it
     * calls already preserve the callee-saved registers r4-r11. Only the
     * caller-saved set has to survive the call: r0-r3, r12, lr, plus the
     * saved status. The frame on the IRQ stack is:
     *
     *     r0-r3 + lr         20 bytes
     *     r12                 4 bytes
     *     spsr                4 bytes
     *                       --------
     *                       28 bytes   (`IRQ_PREAMBLE_BYTES` in
     *                                   sdk/stack_analysis.py)
     *
     * `reti` pops PC and restores CPSR from SPSR in one instruction, which a
     * plain `pop` into PC cannot do.
     * ===================================================================== */
    .align 4
    .global __irq
    .type __irq,%function
__irq:
    push {{r0-r3, lr}}              /* caller-saved regs + return address */
    mov r0, r12                     /* stage r12 (r0 is already saved) */
    push {{r0}}                     /* save r12 */
    .short 0x6bd8                   /* mrss r0: read saved status (SPSR) */
    push {{r0}}                     /* save SPSR */
    bl irq_handler                  /* run the Rust handler */
    pop {{r0}}                      /* restore SPSR */
    .short 0x6bd0                   /* mssr r0: write saved status (SPSR) */
    pop {{r0}}                      /* restore r12 */
    mov r12, r0
    pop {{r0-r3}}                   /* restore caller-saved regs */
    .short 0x6900                   /* reti {{pc}}: pop PC, restore CPSR */

    /* Pad `.vectors` to 16 bytes. */
    .balign 16
"#
);
