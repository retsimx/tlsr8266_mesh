# TLSR8266 BLE Mesh — Hardware & Software Technical Reference

> **Revision:** 1.1  
> **Target SoC:** Telink TLSR8266F512  
> **System clock:** 32 MHz (192 MHz PLL ÷ 6)  
> **RF:** 2.4 GHz BLE 1 Mbps GFSK

---

## Table of Contents

1. [Overview](#1-overview)
2. [Register Reference](#2-register-reference)
3. [System Initialisation Sequence](#3-system-initialisation-sequence)
4. [RF Subsystem](#4-rf-subsystem)
5. [DMA Configuration](#5-dma-configuration)
6. [Interrupt System](#6-interrupt-system)
7. [BLE Link Layer State Engine](#7-ble-link-layer-state-engine)
8. [Advertising](#8-advertising)
9. [Connection Management](#9-connection-management)
10. [BLE Channel Selection](#10-ble-channel-selection)
11. [Packet Format Reference](#11-packet-format-reference)
12. [GATT Attribute Table](#12-gatt-attribute-table)
13. [Pairing Protocol](#13-pairing-protocol)
14. [Mesh Advertising](#14-mesh-advertising)
15. [OTA Protocol](#15-ota-protocol)
16. [AES / Cryptography Hardware](#16-aes--cryptography-hardware)
17. [Flash Storage](#17-flash-storage)
18. [Timing Reference](#18-timing-reference)
19. [Mesh Protocol Overview](#19-mesh-protocol-overview)
20. [RF Hardware Configuration for Mesh](#20-rf-hardware-configuration-for-mesh)
21. [BLE / Mesh Coexistence — State Machine](#21-ble--mesh-coexistence--state-machine)
22. [Mesh Packet Format](#22-mesh-packet-format)
23. [Mesh Command Opcode Reference](#23-mesh-command-opcode-reference)
24. [Mesh Node Status Propagation](#24-mesh-node-status-propagation)
25. [Mesh Provisioning (Over-Mesh Re-pairing)](#25-mesh-provisioning-over-mesh-re-pairing)
26. [Over-Mesh OTA Firmware Update](#26-over-mesh-ota-firmware-update)
27. [Complete System Timing Reference (Mesh)](#27-complete-system-timing-reference-mesh)



---


## 1. Overview

### 1.1 Chip Architecture

The TLSR8266F512 is a Telink 32-bit RISC MCU with an integrated 2.4 GHz radio. Key blocks:

| Block | Description |
|-------|-------------|
| CPU | 32-bit RISC core, single-issue |
| Flash | 512 KB on-chip SPI NOR |
| SRAM | ~16 KB |
| PLL | 192 MHz, divided to 32 MHz system clock |
| RF | 2.4 GHz GFSK/FSK transceiver, BLE 1 Mbps |
| AES | Hardware AES-128 encrypt/decrypt engine |
| DMA | 6-channel DMA controller (channels 0–5) |
| Timers | 3 general-purpose timers (TMR0/1/2) + watchdog |
| GPIO | 6 ports (PA–PF), 48 pins total |
| PWM | Multi-channel PWM for LED control |
| UART | 1 UART with DMA support |
| ADC | SAR ADC |
| USB | Full-speed USB 2.0 |

### 1.2 Memory Map

All registers are memory-mapped starting at base address `0x800000`. Register addresses in this document are given as offsets from this base (i.e. register `0x640` is at physical address `0x800640`).

| Region | Offset Range | Description |
|--------|-------------|-------------|
| SPI | 0x00C–0x00D | Master SPI controller |
| UART | 0x090–0x09F | UART serial |
| Reset/Clock | 0x060–0x07F | System reset, clock gating, power management |
| ADC | 0x069–0x06D | ADC/DMIC clock control |
| Analog | 0x0B8–0x0BA | Indirect analog register interface |
| RF Modem | 0x400–0x4FF | RF transceiver, PLL, AGC, CRC |
| DMA/MAC | 0x500–0x52F | DMA channels 0–5, FIFO control |
| AES | 0x540–0x55F | AES-128 crypto engine |
| GPIO | 0x580–0x5CF | GPIO ports PA–PF |
| Timer | 0x620–0x63F | Timers 0/1/2 + watchdog |
| IRQ | 0x640–0x64F | Interrupt controller |
| System Tick | 0x740–0x74F | Free-running 32-bit tick counter |
| PWM | 0x780–0x7BF | PWM channels |
| RF Control | 0xF00–0xF2F | RF state machine, scheduling, RF IRQ |

### 1.3 Clock Tree

```
  ┌──────────────┐
  │ 16 MHz Xtal  │──► 192 MHz PLL ──► ÷6 ──► 32 MHz SYSTEM CLOCK
  └──────────────┘                              │
                                                ├──► CPU core
                                                ├──► System tick counter (reg 0x740)
                                                ├──► Timer 0/1/2
                                                ├──► RF baseband
                                                ├──► DMA controller
                                                ├──► AES engine
                                                └──► Peripherals (UART, SPI, PWM, ...)
```

**Key timing constants:**

| Symbol | Value | Meaning |
|--------|-------|---------|
| `CLOCK_PLL_CLOCK` | 192,000,000 Hz | PLL output |
| `CLOCK_SYS_CLOCK_HZ` | 32,000,000 Hz | System clock |
| `CLOCK_SYS_CLOCK_1US` | 32 ticks | Ticks per microsecond |
| `CLOCK_SYS_CLOCK_1MS` | 32,000 ticks | Ticks per millisecond |
| `CLOCK_SYS_CLOCK_1S` | 32,000,000 ticks | Ticks per second |
| `CLOCK_MAX_US` | 134,217,727 µs | Max representable span (~134 s) |

### 1.4 Modules Involved in BLE Operation

The firmware implements a combined BLE peripheral + proprietary mesh stack. Major software layers:

1. **MCU HAL** — register access, clock, IRQ, DMA, AES, analog, GPIO, flash
2. **RF Driver** — RF init, channel setting, TX/RX scheduling, power control, access code/CRC
3. **Link Layer** — advertising state machine, connection management, channel selection
4. **Attribute Protocol** — GATT server, ATT request dispatch
5. **Pairing** — challenge-response authentication, session key derivation, mesh provisioning
6. **Mesh Layer** — mesh packet relay, node status tracking, online status broadcast
7. **OTA** — over-the-air firmware update protocol
8. **Application** — light control, group/device address management


## 2. Register Reference

All addresses are offsets from `REG_BASE_ADDR = 0x800000`.

### 2.1 Interrupt Controller (0x640–0x64F)

| Register | Offset | Width | Access | Description |
|----------|--------|-------|--------|-------------|
| `reg_irq_mask` | 0x640 | 32 | R/W | Interrupt source enable mask |
| `reg_irq_en` | 0x643 | 8 | R/W | Global interrupt enable (1 = enabled) |
| `reg_irq_pri` | 0x644 | 32 | R/W | Interrupt priority |
| `reg_irq_src` | 0x648 | 32 | R/W1C | Interrupt source status (write 1 to clear) |
| `reg_irq_src3` | 0x64A | 8 | R/W1C | Interrupt source 3 |

**`reg_irq_mask` (FLD_IRQ) bit definitions:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `TMR0_EN` | Timer 0 interrupt |
| 1 | `TMR1_EN` | Timer 1 interrupt |
| 2 | `TMR2_EN` | Timer 2 interrupt |
| 3 | `USB_PWDN_EN` | USB power-down |
| 4 | `DMA_EN` | DMA interrupt |
| 5 | `DAM_FIFO_EN` | DMA FIFO interrupt |
| 6 | `SBC_MAC_EN` | SBC MAC interrupt |
| 7 | `HOST_CMD_EN` | Host command interrupt |
| 8 | `EP0_SETUP_EN` | USB EP0 setup |
| 9 | `EP0_DAT_EN` | USB EP0 data |
| 10 | `EP0_STA_EN` | USB EP0 status |
| 11 | `SET_INTF_EN` | USB set interface |
| 12 | `IRQ4_EN` | IRQ4 |
| 13 | `ZB_RT_EN` | ZigBee/BLE real-time (RF IRQ) |
| 14 | `SW_EN` | Software interrupt |
| 15 | `AN_EN` | Analog interrupt |
| 16 | `USB_250US_EN` | USB 250 µs frame |
| 17 | `USB_RST_EN` | USB reset |
| 18 | `GPIO_EN` | GPIO interrupt |
| 19 | `PM_EN` | Power management |
| 20 | `SYSTEM_TIMER` | System tick timer match |
| 21 | `GPIO_RISC0_EN` | GPIO RISC0 |
| 22 | `GPIO_RISC1_EN` | GPIO RISC1 |
| 23 | `GPIO_RISC2_EN` | GPIO RISC2 |

**Default IRQ mask after `irq_init()`:** `TMR1_EN | ZB_RT_EN` = bits 1 + 13 = `0x2002`

### 2.2 RF Control Registers (0xF00–0xF2F)

| Register | Offset | Width | Access | Description |
|----------|--------|-------|--------|-------------|
| `reg_rf_mode_control` | 0xF00 | 8 | R/W | RF mode command (0x82=BRX, 0x85=SRX2TX, 0x87=STX2RX) |
| `reg_rf_sn` | 0xF01 | 8 | R/W | RF sequence number |
| `reg_rf_txrx_state` | 0xF02 | 8 | R/W | TX/RX state machine (0x45 = TRX_OFF) |
| `reg_rf_tx_wail_settle_time` | 0xF04 | 32 | R/W | TX wait/settle timing |
| `reg_rf_sys_timer_config` | 0xF0A | 16 | R/W | RF system timer interval (init: 700) |
| `reg_rf_mode` | 0xF16 | 8 | R/W | RF mode flags (bit 2 = cmd_schedule enable) |
| `reg_rf_sched_tick` | 0xF18 | 32 | R/W | Scheduled trigger tick |
| `reg_rf_irq_mask` | 0xF1C | 16 | R/W | RF interrupt mask |
| `reg_rf_irq_status` | 0xF20 | 16 | R/W1C | RF interrupt status |
| `reg_rf_event_clear` | 0xF28 | 32 | W | RF event clear |
| `reg_rf_timing_config` | 0xF2C | 16 | R/W | RF timing configuration (init: 0xC00) |

**`reg_rf_irq_mask` / `reg_rf_irq_status` (FLD_RF_IRQ_MASK) bit definitions:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `IRQ_RX` | Packet received |
| 1 | `IRQ_TX` | Packet transmitted |
| 2 | `IRQ_RX_TIMEOUT` | RX timeout |
| 5 | `IRQ_CMD_DONE` | Command done |
| 7 | `IRQ_RETRY_HIT` | Retry limit hit |

### 2.3 RF Modem Registers (0x400–0x4FF)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_rf_tx_mode1` | 0x400 | 8 | TX mode register 1 |
| `reg_rf_tx_mode` | 0x400 | 16 | TX mode (DMA_EN, CRC_EN, bandwidth, PN, FEC, interleave) |
| `reg_rf_access_code` | 0x408 | 32 | Access code for packet filtering |
| `reg_rf_channel` | 0x40D | 8 | RF channel number |
| `reg_rf_rx_sense_thr` | 0x422 | 8 | RX sensitivity threshold |
| `reg_rf_rx_mode` | 0x428 | 8 | RX mode (EN, 1M/2M mode, low-IF, LPF) |
| `reg_rf_rx_pilot` | 0x42B | 8 | Pilot/SFD configuration |
| `reg_rf_rx_rssi_offset` | 0x439 | 8 | RSSI offset calibration |
| `reg_rf_rx_status` | 0x443 | 8 | RX state machine status |
| `reg_rnd_number` | 0x448 | 16 | Hardware random number |
| `reg_rf_crc` | 0x44C | 32 | CRC initial value (24-bit) |
| `reg_rf_chn_rssi` | 0x458 | 8 | Channel RSSI measurement |
| `reg_rf_rx_gain_agc` | 0x480 | 32 | AGC gain table (indexed, 7 entries × 4 bytes) |

**`reg_rf_tx_mode` (0x400) bit definitions:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `DMA_EN` | Enable DMA for TX packet transfers |
| 1 | `CRC_EN` | Enable CRC generation |
| 3 | (modem) | Additional modem parameter; `TBL_RF_INI` sets 0x400 to `0x0B` then `0x0F`; final post-init value is `0x0F` |
| 2, 4–15 | (reserved) | Set via `TBL_RF_INI`; do not modify independently |

**`reg_rf_rx_mode` (FLD_RF_RX_MODE) bit definitions:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `EN` | RX enable |
| 1 | `MODE_1M` | 1 Mbps mode |
| 2 | `MODE_2M` | 2 Mbps mode |
| 3 | `LOW_IF` | Low-IF mode |
| 4 | `BYPASS_DCOC` | Bypass DC offset cancellation |
| 5 | `MAN_FINE_TUNE` | Manual fine tune |
| 6 | `SINGLE_CAL` | Single calibration |
| 7 | `LOW_PASS_FILTER` | Low-pass filter enable |

**`reg_rf_rx_status` (FLD_RF_RX_STATE) RX state machine values:**

| Value | State |
|-------|-------|
| 0 | IDLE |
| 1 | SET_GAIN |
| 2 | CIC_SETTLE |
| 3 | LPF_SETTLE |
| 4 | PE (preamble extraction) |
| 5 | SYN_START |
| 6 | GLOB_SYN |
| 7 | GLOB_LOCK |
| 8 | LOCAL_SYN |
| 9 | LOCAL_LOCK |
| 10 | ALIGN |
| 11 | ADJUST |
| 12 | DEMOD |
| 13 | FOOTER |

### 2.4 PLL Registers (0x4D0–0x4EF)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_pll_rx_coarse_tune` | 0x4D0 | 16 | RX PLL coarse tuning |
| `reg_pll_rx_fine_tune` | 0x4D4 | 16 | RX PLL fine tuning |
| `reg_pll_rx_fine_div_tune` | 0x4D6 | 16 | RX frequency setting (written during channel set) |
| `reg_pll_tx_coarse_tune` | 0x4D8 | 16 | TX PLL coarse tuning |
| `reg_pll_tx_fine_tune` | 0x4DC | 16 | TX PLL fine tuning |
| `reg_pll_rx_frac` | 0x4E0 | 32 | RX PLL fractional |
| `reg_pll_tx_frac` | 0x4E4 | 32 | TX PLL fractional |
| `reg_pll_ctrl` | 0x4E8 | 32 | PLL control (VCO, prescaler, modulation, SD) |
| `reg_pll_ctrl_a` | 0x4EB | 8 | PLL control A (tick enables, always-on, manual mode) |

### 2.5 DMA Registers (0x500–0x52F)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_dma0_addr` | 0x500 | 16 | DMA0 address (Ethernet RX) |
| `reg_dma0_ctrl` | 0x502 | 16 | DMA0 control |
| `reg_dma1_addr` | 0x504 | 16 | DMA1 address (Ethernet TX) |
| `reg_dma2_addr` | 0x508 | 16 | DMA2 address (**RF RX**) |
| `reg_dma2_ctrl` | 0x50A | 16 | DMA2 control |
| `reg_dma3_addr` | 0x50C | 16 | DMA3 address (**RF TX**) |
| `reg_dma3_ctrl` | 0x50E | 16 | DMA3 control |
| `reg_dma4_addr` | 0x510 | 16 | DMA4 address |
| `reg_dma5_addr` | 0x514 | 16 | DMA5 address |
| `reg_dma_chn_en` | 0x520 | 8 | DMA channel enable |
| `reg_dma_chn_irq_msk` | 0x521 | 8 | DMA channel IRQ mask |
| `reg_dma_tx_rdy0` | 0x524 | 8 | DMA TX ready 0 |
| `reg_dma_rx_rdy0` | 0x526 | 8 | DMA RX ready 0 / IRQ source |
| `reg_dma_tx_rptr` | 0x52A | 8 | DMA TX read pointer |
| `reg_dma_tx_wptr` | 0x52B | 8 | DMA TX write pointer |

**DMA channel aliases:**

| Alias | Maps To | Purpose |
|-------|---------|---------|
| `reg_dma_rf_rx_addr` | DMA2 addr | RF receive buffer |
| `reg_dma_rf_rx_ctrl` | DMA2 ctrl | RF receive control |
| `reg_dma_rf_tx_addr` | DMA3 addr | RF transmit buffer |
| `reg_dma_rf_tx_ctrl` | DMA3 ctrl | RF transmit control |

### 2.6 Timer Registers (0x620–0x63F)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_tmr_ctrl` | 0x620 | 32 | Timer master control (enable, mode, watchdog) |
| `reg_tmr_sta` | 0x623 | 8 | Timer interrupt status (write 1 to clear) |
| `reg_tmr0_capt` | 0x624 | 32 | Timer 0 capture/compare value |
| `reg_tmr1_capt` | 0x628 | 32 | Timer 1 capture/compare value |
| `reg_tmr2_capt` | 0x62C | 32 | Timer 2 capture/compare value |
| `reg_tmr0_tick` | 0x630 | 32 | Timer 0 current value |
| `reg_tmr1_tick` | 0x634 | 32 | Timer 1 current value |
| `reg_tmr2_tick` | 0x638 | 32 | Timer 2 current value |

**`reg_tmr_ctrl` (FLD_TMR) bit fields:**

| Bits | Symbol | Description |
|------|--------|-------------|
| 0 | `TMR0_EN` | Timer 0 enable |
| 1–2 | `TMR0_MODE` | Timer 0 mode |
| 3 | `TMR1_EN` | Timer 1 enable |
| 4–5 | `TMR1_MODE` | Timer 1 mode |
| 6 | `TMR2_EN` | Timer 2 enable |
| 7–8 | `TMR2_MODE` | Timer 2 mode |
| 9–22 | `TMR_WD_CAPT` | Watchdog capture/timeout value |
| 23 | `TMR_WD_EN` | Watchdog enable |

**`reg_tmr_sta` (FLD_TMR_STA) bit fields:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `TMR0` | Timer 0 interrupt pending |
| 1 | `TMR1` | Timer 1 interrupt pending |
| 2 | `TMR2` | Timer 2 interrupt pending |
| 3 | `WD` | Watchdog interrupt pending |

**Watchdog timeout coefficient:** `WATCHDOG_TIMEOUT_COEFF = 18`  
**Watchdog init timeout:** 2000 ms → register value = `2000 × 32000 >> 18`

### 2.7 System Tick Registers (0x740–0x74F)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_system_tick` | 0x740 | 32 | Free-running 32-bit tick counter (32 MHz) |
| `reg_system_tick_irq` | 0x744 | 32 | Tick compare value — triggers `SYSTEM_TIMER` IRQ on match |
| `reg_system_wakeup_tick` | 0x748 | 32 | Wakeup tick value |
| `reg_system_tick_mode` | 0x74C | 8 | Tick mode |
| `reg_system_tick_ctrl` | 0x74F | 8 | Tick control (START=bit 0, STOP=bit 1) |

### 2.8 Clock and Reset Registers (0x060–0x07F)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_rst_clk0` | 0x060 | 32 | Combined reset + clock enable |
| `reg_clk_sel` | 0x066 | 8 | Clock source and divider selection |
| `reg_wakeup_en` | 0x06E | 8 | Wakeup source enable |
| `reg_pwdn_ctrl` | 0x06F | 8 | Power-down control (bit 5 = reboot, bit 7 = sleep) |
| `reg_fhs_sel` | 0x070 | 8 | Clock source (bit 0 = 192M PLL, bit 1 = 32M OSC) |

**`reg_clk_sel` (FLD_CLK_SEL) bit fields:**

| Bits | Symbol | Description |
|------|--------|-------------|
| 0–4 | `DIV` | Clock divider (set to 6 for 192÷6=32 MHz) |
| 5–7 | `SRC` | Clock source (1 = SEL_HS_DIV from PLL) |

### 2.9 Analog Control Registers (0x0B8–0x0BA)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_ana_addr` | 0x0B8 | 8 | Analog register target address |
| `reg_ana_data` | 0x0B9 | 8 | Analog register data |
| `reg_ana_ctrl` | 0x0BA | 8 | Analog control (START, RW, BUSY, RSV) |

**`reg_ana_ctrl` (FLD_ANA) bit fields:**

| Bit | Symbol | Description |
|-----|--------|-------------|
| 0 | `BUSY` | Operation in progress (poll until clear) |
| 4 | `RSV` | Set during reads |
| 5 | `RW` | 1 = write, 0 = read |
| 6 | `START` | Start operation |

**Analog register access procedures:**

The analog interface at 0x0B8–0x0BA is an indirect-access bus. Both read and write sequences must be executed atomically (with interrupts disabled).

**Write procedure:**
```c
void analog_write(uint8_t addr, uint8_t value) {
    REG8(0x800B8) = addr;          // 1. Set target address
    REG8(0x800B9) = value;         // 2. Write data
    REG8(0x800BA) = 0x60;          // 3. Trigger: START (bit 6) | RW (bit 5)
    while (REG8(0x800BA) & 0x01);  // 4. Poll BUSY (bit 0) until clear
    REG8(0x800BA) = 0x00;          // 5. Clear control register
}
```

**Read procedure:**
```c
uint8_t analog_read(uint8_t addr) {
    REG8(0x800B8) = addr;          // 1. Set target address
    REG8(0x800BA) = 0x50;          // 2. Trigger: START (bit 6) | RSV (bit 4)
    while (REG8(0x800BA) & 0x01);  // 3. Poll BUSY (bit 0) until clear
    uint8_t data = REG8(0x800B9);  // 4. Read result
    REG8(0x800BA) = 0x00;          // 5. Clear control register
    return data;
}
```

**Key analog register addresses (indirect):**

| Addr | Description |
|------|-------------|
| 0x04 | PA bias control (power table entry b) |
| 0x06 | SAR ADC power (0 = off) |
| 0x3F | Deep-sleep flag |
| 0x8D | RF output power (power table entry d) |
| 0x93 | Per-channel TX power adjustment |
| 0x99 | Gauss filter select |
| 0xA2 | PA gain control (power table entry a) |
| 0xA7 | RF output match (power table entry c) |
| 0x0A–0x14 | GPIO pull-up/pull-down configuration (2 bits per pin) |

### 2.10 AES Registers (0x540–0x55F)

| Register | Offset | Width | Description |
|----------|--------|-------|-------------|
| `reg_aes_ctrl` | 0x540 | 8 | Control: bit 0 = direction (0=encrypt, 1=decrypt); bit 2 = done flag (poll until set) |
| `reg_aes_data` | 0x548 | 32 | Data FIFO — auto-advances on each 32-bit access; write 4 words (source in), read 4 words (result out) |
| `reg_aes_key[0..15]` | 0x550–0x55F | 8 each | AES-128 key bytes, indexed 0–15 |

### 2.11 GPIO Registers (0x580–0x5CF)

Each GPIO port (PA–PF) occupies an 8-byte register block:

| Offset within port | Register | Description |
|---------------------|----------|-------------|
| +0 | `in` | Pin input value (read-only) |
| +1 | `ie` | Input enable (1 = enabled) |
| +2 | `oen` | Output enable (**inverted**: 0 = enabled) |
| +3 | `out` | Output data |
| +4 | `pol` | Polarity |
| +5 | `ds` | Drive strength |
| +6 | `gpio_func` | Function select (1 = GPIO, 0 = peripheral) |
| +7 | `irq_en` | IRQ enable per pin |

**Port base offsets:** PA=0x580, PB=0x588, PC=0x590, PD=0x598, PE=0x5A0, PF=0x5A8


## 3. System Initialisation Sequence

From cold boot to BLE operational. Entry point: `main_entrypoint()` in `lib.rs`.

### 3.1 Boot Sequence (Numbered Steps)

1. **OTA check** — `OtaManager::handle_ota_update()`. If a new firmware image exists at `FLASH_ADR_LIGHT_NEW_FW`, validate and swap it before proceeding.
2. **CPU wakeup init** — `cpu_wakeup_init()`. Configures power rails and basic CPU state.
3. **Create executor** — Instantiate the Embassy async executor.
4. **Spawn main task** — `App::run(spawner)` becomes the main async entry.
5. **Clock init** — `clock_init()`:
   1. Write `0xFF000000 | USB_EN` to `reg_rst_clk0` (0x060) — assert resets, enable USB clock.
   2. Write clock source = `SEL_HS_DIV` (1), divider = 6 to `reg_clk_sel` (0x066) → 192 MHz ÷ 6 = 32 MHz.
   3. Write `reg_tmr_ctrl` (0x620) — enable Timer 0, set watchdog capture = `2000 × 32000 >> 18`, enable watchdog.
6. **DMA init** — `dma_init()`: write 0 to `reg_dma_chn_irq_msk` (0x521) — disable all DMA IRQs.
7. **GPIO init** — `gpio_init()`: for each port PA–PF:
   1. Write `setting1` register (input enable, output enable inverted, initial output).
   2. Write `setting2` register (drive strength, GPIO/peripheral function select).
   3. Write analog registers 0x0A–0x14 for pull-up/pull-down configuration (2 bits per pin via `analog_write()`).
8. **IRQ init** — `irq_init()`: write `0x2002` (`TMR1_EN | ZB_RT_EN`) to `reg_irq_mask` (0x640).
9. **UART init** — `self.uart_manager.init()`.
10. **RF driver init** — `rf_drv_init(true)`:
    1. Read deep-sleep flag from analog register 0x3F, mask with 0x40.
    2. Load all 61 entries from `TBL_RF_INI` — each entry is an `(address, data, command)` tuple written to the appropriate register or analog address (command 0xC3 = digital register write, command 0xC8 = analog register write).
    3. Load 7 AGC table entries from `TBL_AGC` into `reg_rf_rx_gain_agc` (0x480, indexed).
    4. Read power calibration bytes from flash at `FLASH_ADR_MAC + 0x11` and `+ 0x12`. If valid (≠ 0xFF):
       - Set `RF_TP_BASE` from byte at offset 0x11.
       - Calculate `RF_TP_GAIN = ((RF_TP_BASE − 0x19) << 8) / 80` if base > 0x19, else 0.
       - Secondary calibration at offset 0x12 may override gain calculation.
11. **User init** — `user_init()` → `rf_link_slave_init(interval)`:
    1. Call `blc_ll_init_basic_mcu()`:
       - `reg_rf_sys_timer_config` (0xF0A) = 700.
       - DMA2 address → `LIGHT_RX_BUFF[0]`, DMA2 control = 0x104.
       - `reg_dma_chn_irq_msk` = 0.
       - Enable `ZB_RT_EN` in `reg_irq_mask`.
       - Enable system tick mode bit 1 in `reg_system_tick_mode`.
       - `reg_rf_irq_mask` = 0, then `reg_rf_irq_status` = 0xFFFE, then `reg_rf_irq_mask` = `IRQ_RX | IRQ_TX`.
       - Set system tick IRQ with bit 31 set.
       - `reg_rf_timing_config` (0xF2C) = 0xC00.
    2. Set `CURRENT_RF_STATE = Advertising`.
    3. Set `BLE_PERIPHERAL_LINK_STATE = Disconnected`.
    4. Set `MESH_LISTEN_INTERVAL_US = interval × CLOCK_SYS_CLOCK_1US`.
    5. Schedule first system tick IRQ: `tick + 100 ms`.
    6. Write `0x68` to address 0xF04.
    7. Read or generate 6-byte MAC from `FLASH_ADR_MAC`. If flash reads `0xFFFFFFFF`, generate random MAC via `rand()` and write to flash.
    8. Copy MAC into `PKT_ADV.adv_a`.
    9. Configure advertisement data via `rf_link_slave_set_adv()`.
    10. Load pairing keys from flash via `pair_load_key()`.
    11. Set RF access code to `PAIR_AC` (mesh access code).
    12. Set device address in packet templates.
    13. Enable advertising: `BLE_PERIPHERAL_ADVERTISING_ENABLED = true`.
    14. Load group addresses from flash via `retrieve_dev_grp_address()`.
    15. Init mesh node table via `mesh_node_init()`.
    16. Write device info (MAC + build version + CRC16) to memory at 0x808004.
12. **Spawn async tasks** — mesh sender, mesh receiver, light manager, panic checker.
13. **Enable interrupts** — `irq_enable()`: write 1 to `reg_irq_en` (0x643).
14. **Send boot notification** — broadcast `LGT_POWER_ON` mesh message to address 0xFFFF.
15. **Enter main loop** — `loop { wd_clear(); main_loop(); }`.

### 3.2 RF Init Table (TBL_RF_INI) — Full Contents

Command byte: `0xC3` = digital register write, `0xC8` = analog register write.

| # | Address | Data | Cmd | Notes |
|---|---------|------|-----|-------|
| 0 | 0x5B4 | 0x02 | 0xC3 | GPIO config func4 |
| 1 | 0x474 | 0x08 | 0xC3 | RF modem |
| 2 | 0x85 | 0x00 | 0xC8 | Analog |
| 3 | 0x80 | 0x61 | 0xC8 | Analog |
| 4 | 0x06 | 0x00 | 0xC8 | SAR ADC off |
| 5 | 0x8F | 0x30 | 0xC8 | Analog |
| 6 | 0x81 | 0xD8 | 0xC8 | Analog |
| 7 | 0x8F | 0x38 | 0xC8 | Analog |
| 8 | 0x8B | 0xE3 | 0xC8 | Analog |
| 9 | 0x8E | 0x6B | 0xC8 | Analog |
| 10 | 0x8D | 0x67 | 0xC8 | Analog — RF output power |
| 11 | 0x402 | 0x26 | 0xC3 | RF modem |
| 12 | 0x9E | 0xAD | 0xC8 | Analog |
| 13 | 0xA0 | 0x28 | 0xC8 | Analog |
| 14 | 0xA2 | 0x2C | 0xC8 | PA gain control |
| 15 | 0xA3 | 0x10 | 0xC8 | Analog |
| 16 | 0xAC | 0xA7 | 0xC8 | Analog |
| 17 | 0xAA | 0x2E | 0xC8 | Analog |
| 18 | 0x439 | 0x72 | 0xC3 | RSSI offset |
| 19 | 0x400 | 0x0B | 0xC3 | TX mode 1 |
| 20 | 0x42B | 0xF3 | 0xC3 | RX pilot |
| 21 | 0x43B | 0xFC | 0xC3 | RX header |
| 22 | 0x74F | 0x01 | 0xC3 | System tick ctrl (START) |
| 23 | 0xF04 | 0x50 | 0xC3 | TX settle time |
| 24 | 0xF06 | 0x00 | 0xC3 | RF control |
| 25 | 0xF0C | 0x50 | 0xC3 | RF control |
| 26 | 0xF10 | 0x00 | 0xC3 | RF control |
| 27 | 0x400 | 0x0F | 0xC3 | TX mode 1 (update) |
| 28 | 0x42B | 0xF1 | 0xC3 | RX pilot (update) |
| 29 | 0x420 | 0x20 | 0xC3 | RX sense threshold |
| 30 | 0x421 | 0x04 | 0xC3 | RF modem |
| 31 | 0x422 | 0x00 | 0xC3 | RX sense threshold |
| 32 | 0x424 | 0x12 | 0xC3 | RF modem |
| 33 | 0x464 | 0x07 | 0xC3 | RF modem |
| 34 | 0x4CD | 0x04 | 0xC3 | RX DC I/Q |
| 35 | 0x9E | 0x56 | 0xC8 | Analog |
| 36 | 0xA3 | 0xF0 | 0xC8 | Analog |
| 37 | 0xAA | 0x26 | 0xC8 | Analog |
| 38 | 0x404 | 0xF5 | 0xC3 | RF modem |
| 39 | 0x408 | 0x8E | 0xC3 | Access code byte 0 |
| 40 | 0x409 | 0x89 | 0xC3 | Access code byte 1 |
| 41 | 0x40A | 0xBE | 0xC3 | Access code byte 2 |
| 42 | 0x40B | 0xD6 | 0xC3 | Access code byte 3 |
| 43 | 0x401 | 0x08 | 0xC3 | RF modem |
| 44 | 0x430 | 0x12 | 0xC3 | RF modem |
| 45 | 0x43D | 0x71 | 0xC3 | RX peak detect |
| 46 | 0x402 | 0x24 | 0xC3 | RF modem |
| 47 | 0xF04 | 0x68 | 0xC3 | TX settle time (final) |
| 48 | 0x42C | 0x30 | 0xC3 | RX channel DC |
| 49 | 0x4CA | 0x88 | 0xC3 | RX DC |
| 50 | 0x4CB | 0x04 | 0xC3 | RX DC I |
| 51 | 0x42D | 0x33 | 0xC3 | RF modem |
| 52 | 0x433 | 0x00 | 0xC3 | RF modem |
| 53 | 0x434 | 0x01 | 0xC3 | RX peak envelope |
| 54 | 0x43A | 0x77 | 0xC3 | RF modem |
| 55 | 0x43E | 0xC9 | 0xC3 | RF modem |
| 56 | 0x4CD | 0x06 | 0xC3 | RX DC I/Q |
| 57 | 0x4EB | 0x60 | 0xC3 | PLL ctrl A (16 MHz xtal BLE mode) |
| 58 | 0x99 | 0x31 | 0xC8 | Gauss filter select (16 MHz) |
| 59 | 0x82 | 0x34 | 0xC8 | RX ADC clock enable |
| 60 | 0x9E | 0x41 | 0xC8 | DC_MOD 500K |

### 3.3 AGC Table (TBL_AGC)

Written to `reg_rf_rx_gain_agc` (base 0x480), 4 bytes per entry at indices 0–6:

| Index | Offset | Value |
|-------|--------|-------|
| 0 | 0x480 | 0x30333231 |
| 1 | 0x484 | 0x182C3C38 |
| 2 | 0x488 | 0x000C0C1C |
| 3 | 0x48C | 0x00000000 |
| 4 | 0x490 | 0x1B150F0A |
| 5 | 0x494 | 0x322E2721 |
| 6 | 0x498 | 0x00003E38 |


## 4. RF Subsystem

### 4.1 Architecture

The TLSR8266 integrates a 2.4 GHz GFSK transceiver operating at 1 Mbps (BLE). Key parameters:

| Parameter | Value |
|-----------|-------|
| Modulation | GFSK, BT=0.5, 1 Mbps |
| Frequency range | 2402–2480 MHz |
| Channel spacing | 2 MHz |
| TX power | −37 dBm to +8 dBm (12 levels) |
| Access code width | 32 bits |
| CRC | 24-bit |

### 4.2 Channel Plan

BLE uses 40 channels. Three advertising channels and 37 data channels:

**Advertising channels:**

| BLE Channel | Frequency (MHz) | Gain Value | PLL Integral |
|-------------|-----------------|------------|--------------|
| 37 | 2402 | 0x02 | 2402 |
| 38 | 2426 | 0x1A | 2426 |
| 39 | 2480 | 0x50 | 2480 |

**Data channels (0–36):**

Channels 0–10 (low band): `gain = (ch + 2) × 2`, `freq = gain + 2400`  
Channels 11–36 (high band): `gain = (ch + 3) × 2`, `freq = gain + 2400`

Example data channel frequencies: Ch 0 = 2404 MHz, Ch 10 = 2424 MHz, Ch 11 = 2428 MHz, Ch 36 = 2478 MHz.

### 4.3 Channel Setting Procedure (`rf_set_ble_channel`)

1. Write channel number to `reg_rf_channel` (0x40D).
2. Power off SAR ADC: `analog_write(0x06, 0)`.
3. Enable LDO + baseband PLL: write `0x29` to `reg_rf_mode` (0xF16).
4. Disable receiver: write `0x00` to `reg_rf_rx_mode` (0x428).
5. Reset state machine: write `0x45` (TRX_OFF) to `reg_rf_txrx_state` (0xF02).
6. Set frequency: write PLL integral value to `reg_pll_rx_fine_div_tune` (0x4D6).
7. Set per-channel TX power: `analog_write(0x93, power_value)` where `power_value = RF_TP_BASE − ((gain × RF_TP_GAIN + 0x80) >> 8)`.

### 4.4 Power Calibration

Calibration values are stored in flash at `FLASH_ADR_MAC`:

- **Offset 0x11** — `RF_TP_BASE`: base power value (0x00–0xFF, 0xFF = uncalibrated).
- **Offset 0x12** — secondary calibration reference.

`RF_TP_GAIN` is a per-channel correction factor: `((RF_TP_BASE − 0x19) << 8) / 80` if base > 0x19, else 0.

### 4.5 TX Power Table (TBL_RF_POWER)

12 power levels, each defined by 4 analog register values:

| Index | Power | Reg 0xA2 (a) | Reg 0x04 (b) | Reg 0xA7 (c) | Reg 0x8D (d) |
|-------|-------|-------------|-------------|-------------|-------------|
| 0 | +8 dBm | 0x25 | 0x7C | 0x67 | 0x67 |
| 1 | +4 dBm | 0x0A | 0x7C | 0x67 | 0x67 |
| 2 | 0 dBm | 0x06 | 0x74 | 0x43 | 0x61 |
| 3 | −4 dBm | 0x06 | 0x64 | 0xC2 | 0x61 |
| 4 | −10 dBm | 0x06 | 0x64 | 0xC1 | 0x61 |
| 5 | −14 dBm | 0x05 | 0x7C | 0x67 | 0x67 |
| 6 | −20 dBm | 0x03 | 0x7C | 0x67 | 0x67 |
| 7 | −24 dBm | 0x02 | 0x7C | 0x67 | 0x67 |
| 8 | −28 dBm | 0x01 | 0x7C | 0x67 | 0x67 |
| 9 | −30 dBm | 0x00 | 0x7C | 0x67 | 0x67 |
| 10 | −37 dBm (a) | 0x00 | 0x64 | 0x43 | 0x61 |
| 11 | −37 dBm (b) | 0x00 | 0x64 | 0xCB | 0x61 |

Setting power level: `rf_set_power_level_index(index)` writes these 4 values to the corresponding analog registers.

### 4.6 Access Code and CRC Configuration

| Function | Action |
|----------|--------|
| `rf_set_ble_access_code_adv()` | Write `0xD6BE898E` to `reg_rf_access_code` (standard BLE advertising) |
| `rf_set_ble_access_code(ac)` | Write `ac.swap_bytes()` to `reg_rf_access_code` |
| `rf_set_ble_crc_adv()` | Write `0x555555` to `reg_rf_crc` (standard BLE advertising CRC init) |
| `rf_set_ble_crc(crc)` | Write `crc[0] | (crc[1]<<8) | (crc[2]<<16)` to `reg_rf_crc` |

### 4.7 TX/RX Mode Control

| Function | Registers Written | Description |
|----------|-------------------|-------------|
| `rf_stop_trx()` | `reg_rf_txrx_state` = 0x80 | Immediate stop |
| `rf_set_rxmode()` | `reg_rf_rx_mode` = LPF \| EN; `reg_rf_txrx_state` = 0x45 \| BIT(5) | Enable receiver |
| `rf_set_tx_rx_off()` | `reg_rf_mode`=0x29; `reg_rf_rx_mode`=LPF; `reg_rf_txrx_state`=0x45 | Disable both |
| `rf_reset_sn()` | `reg_rf_sn`=0x3F then 0x00 | Reset packet sequence numbers |

### 4.8 Scheduled State Transitions

| Function | Mode Code | Description |
|----------|-----------|-------------|
| `rf_start_stx2rx(addr, tick)` | 0x87 | TX at tick, then auto-switch to RX |
| `rf_start_srx2tx(addr, tick)` | 0x85 | RX at tick, then auto-switch to TX |
| `rf_start_brx(addr, tick)` | 0x82 | Broadcast RX at tick |

**Exact register write sequence for `rf_start_srx2tx(addr, tick)`** (order is critical):

```c
void rf_start_srx2tx(uint32_t packet_addr, uint32_t tick) {
    REG32(0x800F18) = tick;                        // 1. reg_rf_sched_tick — set trigger time
    REG16(0x80050C) = (uint16_t)(packet_addr);     // 2. reg_dma3_addr (reg_dma_rf_tx_addr) — 16-bit addr offset from 0x800000
    REG8(0x800F16) = REG8(0x800F16) | 0x04;        // 3. reg_rf_mode — enable scheduled mode (bit 2)
    REG8(0x800F00) = 0x85;                         // 4. reg_rf_mode_control — issue SRX2TX command
}
```

The same pattern applies to `rf_start_stx2rx` (mode `0x87`) and `rf_start_brx` (mode `0x82`), each setting `reg_dma3_addr` to the TX packet address and `reg_dma2_addr` to the RX buffer address before issuing the command.



## 5. DMA Configuration

### 5.1 Channel Assignments

| DMA Channel | Register Base | Purpose |
|-------------|---------------|---------|
| 0 | 0x500 | Ethernet RX (unused in BLE) |
| 1 | 0x504 | Ethernet TX (unused in BLE) |
| **2** | **0x508** | **RF RX** — receives packets from radio |
| **3** | **0x50C** | **RF TX** — transmits packets to radio |
| 4 | 0x510 | UART TX |
| 5 | 0x514 | UART RX |

### 5.2 BLE RX DMA Configuration

During `blc_ll_init_basic_mcu()`:

1. `reg_dma2_addr` ← address of `LIGHT_RX_BUFF[LIGHT_RX_BUFFER_WRITE_POINTER]` (16-bit offset from `0x800000`).
2. `reg_dma2_ctrl` ← `0x0104` — field breakdown:

| Bits | Description |
|------|-------------|
| [7:0] | Buffer size in units of 16 bytes. `0x04` = 4 × 16 = **64 bytes** per RX buffer slot. |
| [8] | Write-to-memory enable: `1` = hardware writes received data into the buffer address. |
| [15:9] | Reserved. |

Each entry in `LIGHT_RX_BUFF` must be at least **64 bytes** in size.

3. `reg_dma_chn_irq_msk` ← `0x00`.

> **Note:** The RF DMA channels (2 = RX, 3 = TX) are **not** enabled via `reg_dma_chn_en` (0x520). The RF state machine commands (`reg_rf_mode_control`) activate DMA implicitly. `reg_dma_chn_en` is used only for UART DMA channels; **do not set bits 2 or 3**.

### 5.3 RX Buffer Ring Management

The RX subsystem uses a circular buffer array `LIGHT_RX_BUFF[0..LIGHT_RX_BUFF_COUNT]`:

- **`LIGHT_RX_BUFFER_WRITE_POINTER`** — index of buffer currently being filled by DMA.
- On IRQ_RX:
  1. Save current write index as `rx_index`.
  2. Advance: `WRITE_POINTER = (rx_index + 1) % LIGHT_RX_BUFF_COUNT`.
  3. Point DMA2 to new buffer: `reg_dma2_addr = &LIGHT_RX_BUFF[WRITE_POINTER]`.
  4. Process packet from `LIGHT_RX_BUFF[rx_index]`.
  5. Mark processed: `LIGHT_RX_BUFF[rx_index].dma_len = 1`.

### 5.4 TX DMA

TX uses DMA channel 3. The transmit buffer address is set per-packet:

- `rf_start_stx2rx(addr, tick)` → writes `addr` to `reg_dma3_addr`.
- `rf_start_brx(addr, tick)` → writes empty packet addr to `reg_dma3_addr`.

**TX circular buffer management:**

The TX subsystem uses an 8-slot software-managed circular FIFO with hardware read/write pointers:

| Register | Offset | Description |
|----------|--------|-------------|
| `reg_dma_tx_rptr` | 0x52A | Hardware read pointer (hardware decrements as packets are consumed) |
| `reg_dma_tx_wptr` | 0x52B | Software write pointer (software increments when queuing a packet) |

```c
bool is_tx_fifo_ready() {
    uint8_t used = (REG8(0x80052B) - REG8(0x80052A)) & 7;
    return used < 3;  // Ready when fewer than 3 of 8 slots occupied
}

void enqueue_tx_packet(Packet *pkt) {
    if (!is_tx_fifo_ready()) return;
    uint8_t wptr = REG8(0x80052B);
    // Copy pkt to BLE_TX_BUFF[wptr & (TX_FIFO_DEPTH - 1)]
    REG8(0x80052B) = wptr + 1;  // Advance write pointer
}
```

On disconnect, `reg_dma_tx_rptr` is reset to `0x10` (and `reg_dma_tx_wptr` synchronised) to drain any pending packets.

---


## 6. Interrupt System

### 6.1 Interrupt Sources

The init mask enables only two sources:

| Source | Mask Bit | Purpose |
|--------|----------|---------|
| Timer 1 | `TMR1_EN` (bit 1) | Light transition stepping |
| ZB/BLE RT | `ZB_RT_EN` (bit 13) | RF packet TX/RX and state machine |

Additional sources enabled at runtime:

| Source | Mask Bit | Purpose |
|--------|----------|---------|
| System Timer | `SYSTEM_TIMER` (bit 20) | BLE state machine tick (enabled during `blc_ll_init_basic_mcu`) |
| Timer 0 | `TMR0_EN` (bit 0) | 64-bit clock overflow + OTA timeout |

### 6.2 Global Interrupt Control

| Operation | Code |
|-----------|------|
| Enable | `write_reg_irq_en(1)` — returns previous state |
| Disable | `write_reg_irq_en(0)` — returns previous state |
| Restore | `write_reg_irq_en(saved_state)` |

### 6.3 TC32 Interrupt Vector and ISR Calling Convention

The TC32 core uses a fixed interrupt vector table in flash, defined in `sdk/cstartup_8266.S`:

```asm
.org 0x0   tj  __reset          ; Reset vector at flash offset 0x0
.org 0x10  tj  __irq            ; IRQ vector at flash offset 0x10
```

The `__irq` thunk saves all registers, calls `irq_handler`, then restores them:

```asm
__irq:
    tpush   {r14}           ; save LR
    tpush   {r0-r7}         ; save low registers
    tmrss   r0              ; read CPSR into r0
    tmov    r1, r8          ; move high registers to r1-r5
    tmov    r2, r9
    tmov    r3, r10
    tmov    r4, r11
    tmov    r5, r12
    tpush   {r0-r5}         ; save CPSR + r8-r12
    tjl     irq_handler     ; call C/Rust handler
    tpop    {r0-r5}         ; restore CPSR + r8-r12
    tmov    r8,  r1
    tmov    r9,  r2
    tmov    r10, r3
    tmov    r11, r4
    tmov    r12, r5
    tmssr   r0              ; restore CPSR
    tpop    {r0-r7}         ; restore low registers
    treti   {r15}           ; return from IRQ (restores PC from r15)
```

**Implications for a C/Rust firmware implementation:**

- `irq_handler` must be a plain C function with external linkage and the symbol name `irq_handler` (no name-mangling). Do **not** mark it with any ISR attribute — all register save/restore is performed by `__irq`.
- The C function sees a normal calling environment on entry.
- The function **must reside in RAM** (`.ram_code` section) because the flash instruction cache may be disabled during RF operations.
- The startup code allocates a dedicated IRQ stack of **0x800 bytes** (`IRQ_STK_SIZE`) and sets `r13` to the top-of-stack in IRQ mode (`mode = 0x12`). No stack setup is required in `irq_handler` itself.

### 6.4 IRQ Dispatch Flow (`irq_handler`)

`irq_handler()` is placed in `.ram_code`. Execution sequence:

```
irq_handler()
├── Create IrqTracker (sets IS_IRQ_MODE = true; auto-clears on drop/return)
├── Read reg_rf_irq_status (0xF20)
├── If IRQ_TX set:
│   └── handle_rf_transmission_complete()
│       └── Write 0x0002 to reg_rf_irq_status (clear TX bit 1)
├── If IRQ_RX set:
│   └── handle_rf_packet_reception()
│       ├── Advance RX ring buffer write pointer
│       ├── Check reg_rf_rx_status (0x443) — if 0x0B discard (FOOTER error)
│       ├── Update DMA2 address to next buffer slot
│       ├── Write 0x0001 to reg_rf_irq_status (clear RX bit 0)
│       ├── Validate packet:
│       │   ├── dma_len > 0x0E
│       │   ├── dma_len == (rf_len & 0x3F) + 0x11  (length consistency)
│       │   └── *(pkt_base + dma_len + 3) & 0x51 == 0x40  (hardware status footer)
│       ├── Timestamp duplicate check
│       └── Dispatch based on BLE_PERIPHERAL_LINK_STATE:
│           ├── Advertising: handle scan req (cmd=3) or connect req (cmd=5)
│           ├── Mesh: handle_mesh_packet (decrypt, deduplicate, forward)
│           └── Connected/Receiving: handle_ble_connection_data (SN check, timing adjust)
└── handle_system_interrupts()
    ├── SYSTEM_TIMER IRQ (reg_irq_src bit 20):
    │   ├── Clear: write 0x00100000 to reg_irq_src (0x648)
    │   └── Dispatch based on CURRENT_RF_STATE:
    │       ├── Advertising → handle_ble_advertisement_state()
    │       ├── Connected → handle_ble_connected_state()
    │       ├── Receiving → configure_ble_receive_state()
    │       ├── MeshListening → handle_mesh_listening_state()
    │       └── Idle → (no action)
    ├── Timer 0 IRQ (reg_tmr_sta bit 0):
    │   ├── Clear: write 0x01 to reg_tmr_sta (0x623)
    │   ├── Advance 64-bit clock
    │   └── OTA timeout decrement (if active)
    ├── Timer 1 IRQ (reg_tmr_sta bit 1):
    │   ├── Clear: write 0x02 to reg_tmr_sta (0x623)
    │   └── Call light_manager.transition_step()
    └── UART IRQ:
        └── Call uart_manager.check_irq()
```

**Interrupt clear values (write-1-to-clear):**

| IRQ | Register | Address | Value to Write |
|-----|----------|---------|----------------|
| SYSTEM_TIMER | `reg_irq_src` | 0x648 | `0x00100000` (bit 20) |
| Timer 0 | `reg_tmr_sta` | 0x623 | `0x01` (bit 0) |
| Timer 1 | `reg_tmr_sta` | 0x623 | `0x02` (bit 1) |
| RF RX | `reg_rf_irq_status` | 0xF20 | `0x0001` (bit 0) |
| RF TX | `reg_rf_irq_status` | 0xF20 | `0x0002` (bit 1) |

**Packet hardware status footer:**

The RF hardware appends a status byte immediately after the received packet payload in the DMA buffer. Its location and interpretation:

```c
// Location: packet_buffer_base + dma_len + 3
uint8_t status = *((uint8_t *)(packet_addr + dma_len + 3));

// Validity check:
// bit 6 set  → valid packet received
// bit 4 clear → no packet-type error
// bit 0 clear → no CRC error
if ((status & 0x51) != 0x40) { discard_packet(); }
```

The two-stage packet validation is:
1. `reg_rf_rx_status` (register 0x443) — check for `0x0B` (FOOTER state error in RX state machine) before looking at the DMA buffer.
2. DMA buffer status footer — check `(status & 0x51) == 0x40` after reading the packet.


## 7. BLE Link Layer State Engine

The firmware maintains two parallel state variables that together govern RF behaviour:

| Variable | Type | Purpose |
|----------|------|---------|
| `CURRENT_RF_STATE` | `RfOperationState` | Drives the system-timer IRQ dispatch (§6.4) |
| `BLE_PERIPHERAL_LINK_STATE` | `BlePeripheralLinkState` | Drives packet-reception dispatch (§6.4) |

Both are updated together on state transitions and must remain consistent.

### 7.1 `BlePeripheralLinkState` — Peripheral Link State

Controls packet reception handling. Has explicit discriminant values:

| State | Discriminant | Description |
|-------|-------------|-------------|
| `Disconnected` | 0 | No active BLE link; idle |
| `Advertising` | 1 | Broadcasting ADV_IND; accepting SCAN_REQ and CONNECT_IND |
| `Mesh` | 4 | Listening on mesh channel (access code = `PAIR_AC`) |
| `Connected` | 5 | Active BLE connection; connection data packets expected |
| `Receiving` | 7 | Awaiting next connection event packet from master |

### 7.2 `RfOperationState` — RF Operation State

Controls system-timer IRQ dispatch. Compiler-assigned sequential discriminants (0–4):

| State | Value | Description |
|-------|-------|-------------|
| `Idle` | 0 | No RF operation scheduled |
| `Advertising` | 1 | Transmit ADV_IND, cycle channels 37/38/39 |
| `Connected` | 2 | Bridge BLE↔mesh; run connection event |
| `Receiving` | 3 | Configure RX window for next connection event |
| `MeshListening` | 4 | Listen on mesh channel; periodically advertise |

### 7.3 State Transition Diagram

```
                  ┌──────────────────────────────────────────┐
                  │              Power-on / Reset             │
                  └───────────────────┬──────────────────────┘
                                      ▼
                               ┌──────────────┐
                          ┌───►│ MeshListening │◄──────────────────────┐
                          │    └──────┬───────┘                        │
                          │           │ every 4th listen cycle          │
                          │           │ (ADV_INTERVAL2LISTEN = 4)       │
                          │           ▼                                 │
                          │    ┌──────────────┐                        │
                          │    │  Advertising  │                        │
                          │    └──────┬────────┘                       │
                          │           │ CONNECT_IND received            │
                          │           ▼                                 │
                          │    ┌──────────────┐   disconnect/timeout    │
                          └────│   Connected   │────────────────────────┘
                               └──────────────┘
                                       ↕  (Receiving ↔ Connected loop
                                          within a connection)
```

### 7.4 Timing Parameters

| Parameter | Symbol | Default Value | Units |
|-----------|--------|---------------|-------|
| Advertisement interval per channel | — | 1,200 µs (0x4B0 ticks) | µs |
| Mesh listen interval | `MESH_LISTEN_INTERVAL_US` | Configurable (set at init) | sys ticks |
| Min RX window | — | 3,800 µs (0xED8 ticks) | µs |
| Connection interval | `SLAVE_LINK_INTERVAL` | From CONNECT_IND (× 1250 × 32 ticks) | sys ticks |
| Supervision timeout | `BLE_PERIPHERAL_CONNECTION_TIMEOUT_US` | From CONNECT_IND (× 10,000 µs) | µs |
| Scan response delay | `BLE_SCAN_RESPONSE_INTERVAL_US` | 0x92 or 0x93 (varies by xtal) | µs |
| Timing adjust window | — | 700–1100 µs (normal), ±200 µs correction | µs |
| OTA RX timeout | — | 10,000 µs (10 ms) | µs |

### 7.5 Main Loop Structure

The main loop runs outside interrupt context:

```
loop {
    wd_clear()                // Clear watchdog
    rf_link_slave_proc()      // Process mesh pairing + connection parameter updates
    main_loop()               // Application processing (async)
}
```

`rf_link_slave_proc()` calls:
1. `mesh_pair_proc()` — mesh pairing state machine.
2. `update_connect_para()` — delayed connection parameter update after service discovery.

---

## 8. Advertising

### 8.1 PDU Type

The device uses `ADV_IND` (connectable undirected advertising, type 0x00) with scan response support.

### 8.2 Advertising Data Construction

**Advertisement packet layout:**

| Field | Bytes | Value |
|-------|-------|-------|
| `dma_len` | 4 | `data_len + 8` |
| `_type` | 1 | 0x00 (ADV_IND) |
| `rf_len` | 1 | `data_len + 6` (max 0x25 = 37) |
| `adv_a[0..6]` | 6 | Device MAC address |
| `data[0..N]` | ≤31 | AD structures |

**AD structures included:**

| Type | Value | Description |
|------|-------|-------------|
| 0x09 | Complete Local Name | Mesh network name |
| 0xFF | Manufacturer Specific | Mesh-specific data |
| UUID | (inserted at offset 3) | Service UUID |

`MAX_ADV_DATA_LEN = 0x25 (37)` = maximum rf_len.

### 8.3 Channel Sequence

Advertising cycles through three channels in order:

1. Channel 37 (2402 MHz)
2. Channel 38 (2426 MHz)
3. Channel 39 (2480 MHz)

A static counter `ST_PNO` tracks position (0, 1, 2). After all three:
- `ST_PNO` resets to 0.
- `BLE_ADVERTISING_ENABLED = false`.
- State transitions to `MeshListening`.

### 8.4 Advertisement Timing

Each channel advertisement is scheduled 1,200 µs apart:

```
write_reg_system_tick_irq(CLOCK_SYS_CLOCK_1US × 0x4B0 + current_tick)  // 1200 µs
```

After the third channel, a short delay transitions to mesh listening:
- With online status pending: 100 µs delay.
- Without online status: 500 µs delay.

### 8.5 ADV_IND → SCAN_REQ → SCAN_RSP Flow

1. Device transmits `ADV_IND` via `rf_start_stx2rx(PKT_ADV, tick + 10µs)`.
2. Hardware auto-switches to RX after TX completes.
3. If `SCAN_REQ` received (cmd=3):
   - Validate MAC matches `MAC_ID[0..4]`.
   - Stop radio.
   - Build scan response packet:
     - `dma_len` = 0x27, `_type` = 0x04, `rf_len` = 0x25.
     - `adv_a` = MAC address.
     - `handle` = 0xFF1E (mesh handle).
     - `device_address` = current `DEVICE_ADDRESS`.
     - Append `ADV_RSP_PRI_DATA`.
   - Schedule TX: write `reg_rf_sched_tick = rx_time + BLE_SCAN_RESPONSE_INTERVAL_US × 32`.
   - Set `reg_rf_mode_control = 0x85` (SRX2TX mode).
   - Point DMA3 to response packet.

### 8.6 CONNECT_IND Processing

If `CONNECT_IND` received (cmd=5):
- Validate MAC matches.
- Delegate to `rf_link_slave_connect()` (see §9).

---

## 9. Connection Management

### 9.1 CONNECT_IND Processing (`rf_link_slave_connect`)

**Pre-validation:**
1. `BLE_PERIPHERAL_CONNECTION_ENABLED` must be true.
2. Advertiser address in packet must match `MAC_ID`.
3. Connection parameters must pass `check_par_con()`:
   - `6 ≤ interval ≤ 3200` (7.5 ms to 4 s).
   - `0 < wsize ≤ interval`.
   - `10 < timeout ≤ 3200` (100 ms to 32 s).
   - `woffset ≤ interval`.
   - `0 < (hop & 0x1F) ≤ 16`.
   - At least one channel enabled in `chm`.
   - `latency × interval × 2 < timeout`.

**On acceptance:**
1. Stop radio: `rf_stop_trx()`.
2. Calculate window offset: `CLOCK_SYS_CLOCK_1US × 1250 × (woffset + 1)`.
3. Compute timing:
   - `SLAVE_LINK_INTERVAL = interval × 32 × 1250` ticks.
   - `SLAVE_WINDOW_SIZE = (wsize × 1250 + 1100) × 32` ticks (clamped to < `SLAVE_LINK_INTERVAL − 32 × 1250`).
   - `BLE_PERIPHERAL_CONNECTION_TIMEOUT_US = timeout × 10,000` µs.
4. Build channel table: `ble_ll_build_available_channel_table(chm, true)`.
5. Set CRC init: `rf_set_ble_crc([crcinit[0], crcinit[1], crcinit[2]])`.
6. Reset packet SN: `rf_reset_sn()`.
7. Init pairing: `pair_init()`.
8. Set `CURRENT_RF_STATE = Receiving`, `BLE_PERIPHERAL_CONNECTION_INSTANT = 0`.
9. `DEVICE_STATUS_TICK_COUNTER = (interval × 5) >> 2`.
10. Schedule first connection event interrupt.

### 9.2 Connection Event Handling (`handle_ble_connected_state`)

Called every `SYSTEM_TIMER` IRQ when `CURRENT_RF_STATE == Connected`. Sequential steps:

1. `initialize_connection_hardware()` — stop radio, clear RF, configure timing register.
2. `check_connection_supervision_timeout()` — if `(current_tick − SLAVE_CONNECTED_TICK) > timeout × CLOCK_SYS_CLOCK_1US`: disconnect + cleanup. Returns true on timeout.
3. `handle_status_read_timeout()` — clear stale status reads.
4. `handle_ota_operations()` — if OTA active: process OTA, schedule next event, return early.
5. `handle_bridge_operations()` — mesh bridge TX: call `tx_packet_bridge()` if interval allows.
6. `process_mesh_operations()` — status responses, pairing, mesh node flush, status reporting.
7. `manage_connection_event_timing()` — handle missed events (advance `SLAVE_NEXT_CONNECT_TICK` in loop), channel hop, configure RX.
8. `schedule_next_connection_event()` — `write_reg_system_tick_irq(SLAVE_NEXT_CONNECT_TICK)`, `CURRENT_RF_STATE = Receiving`.

### 9.3 Channel Map and Hop Increment

Extracted from `CONNECT_IND`:
- **Channel map** (`chm`): 5 bytes (37 bits), one per data channel.
- **Hop increment** (`hop & 0x1F`): 5-bit value (1–16).

Channel map update via LL_CHANNEL_MAP_IND:
- When `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP == 1` and connection instant matches:
  - Apply new `SLAVE_CHN_MAP`.
  - Rebuild: `ble_ll_build_available_channel_table(new_map, false)`.

### 9.4 Connection Parameter Update

Via LL_CONNECTION_UPDATE_IND:
- When `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP == 2` and instant matches:
  - Apply `BLE_CONN_INTERVAL`, `BLE_CONN_TIMEOUT`.
  - Calculate new window size: `SLAVE_LINK_INTERVAL − CLOCK_SYS_CLOCK_1US × 0x4E2` (1250 ticks ≈ 1.25 ms).
  - Set `BLE_PERIPHERAL_TIMING_UPDATE_TIMESTAMP2_FLAG = true`.

### 9.5 Timing Adjustment PLL (`rf_link_timing_adjust`)

Compensates for crystal drift using one-shot adjustment:

```
timing_error = rx_time − BRIDGE_RECEIVE_TIMING_TICK

if timing_error < 700 µs:
    SLAVE_NEXT_CONNECT_TICK −= 200 µs    // Too early → advance
elif timing_error > 1100 µs:
    SLAVE_NEXT_CONNECT_TICK += 200 µs    // Too late → delay
else:
    (no adjustment — within tolerance)
```

### 9.6 Disconnection (`cleanup_ble_disconnection`)

Triggered by supervision timeout or explicit disconnect:

1. Schedule advertisement: `tick_irq = current_tick + 100 µs`.
2. `CURRENT_RF_STATE = Advertising`.
3. Reset DMA TX pointer to 0x10.
4. `BLE_PERIPHERAL_CONNECTION_ACTIVE = false`.
5. `PAIR_LOGIN_OK = false`.
6. `mesh_report_status_enable(false)`.
7. Load keys: `pair_load_key()`.
8. Set `PAIR_SETTING_FLAG = PairSetted`.
9. Restore RF timing register (0xF04) to 0x68 (or 0x5E for 16 MHz).
10. Clear `NEED_UPDATE_CONNECT_PARA`, `GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP`.


## 10. BLE Channel Selection

### 10.1 Algorithm 1 Implementation

The firmware implements BLE Channel Selection Algorithm #1 (CSA #1) as defined in the Bluetooth specification.

**State variables:**

| Variable | Type | Description |
|----------|------|-------------|
| `BLE_LL_CHANNEL_NUM` | usize | Count of enabled channels (0–37) |
| `BLE_LL_LAST_UNMAPPED_CH` | usize | Last unmapped channel index (0–36) |
| `BLE_LL_CHANNEL_TABLE` | [u8; 40] | Lookup table of enabled channel indices |

### 10.2 Building the Channel Table (`ble_ll_build_available_channel_table`)

Input: 5-byte channel map (`chm`), optional reset flag.

```
BLE_LL_CHANNEL_NUM = 0
if reset: BLE_LL_LAST_UNMAPPED_CH = 0

for chan_id in 0..37:
    byte_idx = chan_id >> 3       // byte index (0–4)
    bit_pos  = chan_id & 0x07     // bit position within byte
    if chm[byte_idx] & (1 << bit_pos) != 0:
        BLE_LL_CHANNEL_TABLE[BLE_LL_CHANNEL_NUM] = chan_id
        BLE_LL_CHANNEL_NUM += 1
```

### 10.3 Selecting the Next Data Channel (`ble_ll_select_next_data_channel`)

Input: channel map, hop increment.

```
1. unmapped = (BLE_LL_LAST_UNMAPPED_CH + hop) % 37
2. BLE_LL_LAST_UNMAPPED_CH = unmapped

3. Check if channel is enabled:
   byte_idx = unmapped >> 3
   bit_pos  = unmapped & 0x07
   if (chm[byte_idx] & (1 << bit_pos)) == 0:
       // Channel disabled — remap
       channel = BLE_LL_CHANNEL_TABLE[unmapped % BLE_LL_CHANNEL_NUM]
   else:
       channel = unmapped

4. return channel
```

This matches the BLE spec: compute unmapped channel via modular hop, then remap to an enabled channel if the unmapped channel is disabled.

---

## 11. Packet Format Reference

All packets are accessed through a 48-byte `Packet` union type. `const_assert!(size_of::<Packet>() == 48)`.

### 11.1 Common L2CAP Header (`PacketL2capHead` — 10 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | `dma_len` | DMA transfer length (includes RF overhead) |
| 4 | 1 | `_type` | Bits [3:0]=PDU type, bit 4=NESN, bit 5=SN, bit 6=MD |
| 5 | 1 | `rf_len` | RF payload length (bits [4:0]) |
| 6 | 2 | `l2cap_len` | L2CAP payload length (little-endian) |
| 8 | 2 | `chan_id` | L2CAP channel ID: 0x04=ATT, 0xFF03=vendor mesh, 0xFFFF=broadcast |

### 11.2 Mesh Packet (`MeshPkt` — 48 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 10 | `head` | L2CAP header (chan_id: 0xFFFF for broadcast, 0xFF03 for direct) |
| 10 | 2 | `src_tx` | Source transmitter address (relay hop address) |
| 12 | 1 | `handle1` | Flags/marker byte |
| 13 | 3 | `sno` | 24-bit sequence number [0][1][2] (little-endian) |
| 16 | 2 | `src_adr` | Original source address (little-endian) |
| 18 | 2 | `dst_adr` | Destination address (little-endian) |
| 20 | 1 | `op` | Operation code |
| 21 | 2 | `vendor_id` | Vendor ID (little-endian) |
| 23 | 10 | `par` | Parameters [0–9] |
| 33 | 5 | `internal_par1` | Internal relay parameters — see §22.2 for field layout |
| 38 | 1 | `ttl` | Time to live |
| 39 | 4 | `internal_par2` | Internal: [1] used as MIC buffer for broadcast |
| 43 | 5 | `no_use` | Unused/padding |

### 11.3 ATT Write Packet (`PacketAttWrite` / `PacketAttCmd` — 43 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 10 | `head` | L2CAP header (chan_id=0x0004) |
| 10 | 1 | `opcode` | ATT opcode (0x52=Write Cmd, 0x12=Write Req) |
| 11 | 1 | `handle` | Attribute handle low byte |
| 12 | 1 | `handle1` | Attribute handle high byte |
| 13 | 30 | `value` | `PacketAttValue`: sno[3] + src[2] + dst[2] + val[23] |

### 11.4 ATT Value Payload (`PacketAttValue` — 30 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 3 | `sno` | Sequence number [0][1][2] |
| 3 | 2 | `src` | Source address [0][1] |
| 5 | 2 | `dst` | Destination address [0][1] |
| 7 | 23 | `val` | Payload (op + vendor_id + params + internal) |

### 11.5 Advertising Packet (`RfPacketAdvIndModuleT` — 43 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | `dma_len` | DMA length |
| 4 | 1 | `_type` | PDU type: RA(1)\|TA(1)\|RFU(2)\|TYPE(4) |
| 5 | 1 | `rf_len` | RF payload length |
| 6 | 6 | `adv_a` | Advertiser address [0–5] |
| 12 | 31 | `data` | Advertisement data [0–30] |

### 11.6 Connection Request (`PacketLlInit`)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | `dma_len` | DMA length |
| 4 | 1 | `_type` | 0x05 (CONNECT_IND) |
| 5 | 1 | `rf_len` | RF payload length |
| 6 | 6 | `adv_a` | Advertiser address |
| 12 | 6 | `scan_a` | Scanner address |
| 18 | 4 | `aa` | Access address for link [0–3] |
| 22 | 3 | `crcinit` | CRC init value [0–2] |
| 25 | 1 | `wsize` | Window size (1.25 ms units) |
| 26 | 2 | `woffset` | Window offset (1.25 ms units) |
| 28 | 2 | `interval` | Connection interval (1.25 ms units) |
| 30 | 2 | `latency` | Slave latency |
| 32 | 2 | `timeout` | Supervision timeout (10 ms units) |
| 34 | 5 | `chm` | Channel map (37 bits) |
| 39 | 1 | `hop` | Hop increment (bits [4:0]) |

### 11.7 Control Packets

**Version Indication (14 bytes):** opcode 0x0C, main_ver=0x08, vendor=VENDOR_ID, sub_ver=0x08.

**Feature Response (13 bytes):** opcode 0x09, 8-byte feature data (byte 0 = 1, rest 0).

**MTU Response (14 bytes):** opcode 0x03, MTU = 0x0017 (23 bytes).

**Error Response (15 bytes):** opcode 0x01, err_opcode, err_handle[2], err_reason.

---

### 11.8 Worked Example: Minimal Mesh Broadcast Packet

The following example constructs a `MeshPkt` to broadcast a `LGT_CMD_LIGHT_ONOFF` (turn-on) command to all nodes (group address `0xFFFF`):

```c
MeshPkt pkt = {0};

// L2CAP / BLE header fields
pkt.head.dma_len    = 40;        // Total payload length
pkt.head.rf_len     = 37;        // Over-air length (dma_len - 3)
pkt.head.l2cap_len  = 29;        // ATT payload (rf_len - 8)
pkt.head.chan_id    = 0x0004;    // BLE L2CAP ATT channel

// Mesh addressing
pkt.sno[0]          = sno & 0xFF;  // 24-bit sequence number, LE
pkt.sno[1]          = (sno >> 8) & 0xFF;
pkt.sno[2]          = (sno >> 16) & 0xFF;
pkt.src_adr         = MY_DEVICE_ADDR;     // 16-bit, LE
pkt.dst_adr         = 0xFFFF;             // broadcast

// Command
pkt.op              = LGT_CMD_LIGHT_ONOFF; // 0xD0
pkt.vendor_id       = VENDOR_ID;           // 0x0211 (LE)
pkt.par[0]          = 1;                   // param: ON

// Internal relay control
pkt.internal_par1[INTERNAL_PAR_STATUS_DATA]        = 0;
pkt.internal_par1[INTERNAL_PAR_PACKET_FORMAT_MODE] = 0;
pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]   = 2; // relay twice
pkt.internal_par1[INTERNAL_PAR_SEND_ACK]           = 0; // no ACK

pkt.ttl = 10;

// Encrypt payload (bytes 10–37) with mesh LTK
aes_att_encryption_packet(MESH_LTK, &pkt);

// Transmit on mesh channel
rf_start_stx(&pkt, get_system_tick() + TX_DELAY_TICKS);
```

---

## 12. GATT Attribute Table

### 12.1 Attribute Structure

```c
struct AttributeT {
    u8  att_num;       // Attribute count (sentinel only)
    u8  uuid_len;      // 2 (16-bit UUID) or 16 (128-bit UUID)
    u8  attr_len;      // Current value length
    u8  attr_max_len;  // Maximum value length
    u8* uuid;          // Pointer to UUID
    u8* p_attr_value;  // Pointer to value buffer
    fn  w;             // Write callback (optional)
    fn  r;             // Read callback (optional)
};
```

### 12.2 Full Attribute Table (29 entries)

| Handle | UUID | Type | Len | Description | Write CB | Read CB |
|--------|------|------|-----|-------------|----------|---------|
| 0 | — | sentinel | 0 | Count = 28 | — | — |
| 1 | 0x2800 | Primary Service | 2 | GAP Service (0x1800) | — | — |
| 2 | 0x2803 | Characteristic | 1 | Device Name properties | — | — |
| 3 | 0x2A00 | Char Value | var | Device Name string | — | — |
| 4 | 0x2901 | User Desc | 7 | "DevName" | — | — |
| 5 | 0x2803 | Characteristic | 1 | Appearance properties | — | — |
| 6 | 0x2A01 | Char Value | 2 | Appearance (0x0000) | — | — |
| 7 | 0x2800 | Primary Service | 2 | Device Info Service (0x180A) | — | — |
| 8 | 0x2803 | Characteristic | 1 | FW Revision properties | — | — |
| 9 | 0x2A26 | Char Value | 4 | Firmware version (BUILD_VERSION) | — | — |
| 10 | 0x2803 | Characteristic | 1 | Manufacturer Name properties | — | — |
| 11 | 0x2A29 | Char Value | var | Manufacturer name (MESH_NAME) | — | — |
| 12 | 0x2803 | Characteristic | 1 | Model Number properties | — | — |
| 13 | 0x2A24 | Char Value | 12 | "model id 123" | — | — |
| 14 | 0x2803 | Characteristic | 1 | HW Revision properties | — | — |
| 15 | 0x2A27 | Char Value | 4 | 0x22222222 | — | — |
| 16 | 128-bit | Primary Service | 16 | Telink SPP Service | — | — |
| 17 | 0x2803 | Characteristic | 1 | Server→Client (R\|W\|Notify) | — | — |
| 18 | 128-bit | Char Value | 4 | Status data (4 bytes) | `mesh_status_write` | — |
| 19 | 0x2902 | CCC Descriptor | 2 | Notification enable (0x01, 0x00) | — | — |
| 20 | 0x2803 | Characteristic | 1 | Client→Server (R\|W\|W_NoRsp) | — | — |
| 21 | 128-bit | Char Value | 16 | Command buffer | `rf_link_slave_data_write` | — |
| 22 | 0x2901 | User Desc | 7 | "Command" | — | — |
| 23 | 0x2803 | Characteristic | 1 | OTA (R\|W_NoRsp) | — | — |
| 24 | 128-bit | Char Value | 16 | OTA data buffer | `rf_link_slave_data_ota` | — |
| 25 | 0x2901 | User Desc | 3 | "OTA" | — | — |
| 26 | 0x2803 | Characteristic | 1 | Pair (R\|W) | — | — |
| 27 | 128-bit | Char Value | 16 | Pairing data buffer | `pair_write` | `pair_read` |
| 28 | 0x2901 | User Desc | 4 | "Pair" | — | — |

### 12.3 Telink SPP Service UUIDs (128-bit)

| UUID | First Byte | Purpose |
|------|-----------|---------|
| Service | 0x10 | `00010203-0405-0607-0809-0A0B0C0D1910` |
| Server→Client | 0x11 | Status notifications |
| Client→Server | 0x12 | Command writes |
| OTA | 0x13 | Firmware updates |
| Pair | 0x14 | Pairing protocol |

### 12.4 GATT Request Handling (`l2cap_att_handler`)

| ATT Opcode | Name | Handler |
|------------|------|---------|
| 0x02 | Exchange MTU Request | Returns MTU=23 |
| 0x04 | Find Information Request | Returns handle-UUID pairs |
| 0x06 | Find By Type Value Request | Returns matching handles |
| 0x08 | Read By Type Request | Returns typed attribute values |
| 0x0A | Read Request | Returns attribute value (or calls read CB) |
| 0x10 | Read By Group Type Request | Returns service groups |
| 0x12 / 0x52 | Write Request / Write Command | Writes attribute (or calls write CB) |

LL Control packets (type=3) are handled in `handle_mtu_exchange_response`:
- Handle 0x08 → Feature Response (opcode 0x09).
- Handle 0x0C → Version Indication (opcode 0x0C).
- Handle 0x02 → Set supervision timeout to 1 s.
- Other → Unknown Response (opcode 0x07).


## 13. Pairing Protocol

### 13.1 Overview

The pairing protocol operates over the GATT **Pair** characteristic (UUID `0x1914`). It establishes a session key for encrypted communication and optionally provisions new mesh network credentials. Two security modes are supported: **Simple** (XOR-based, for compatibility) and **Secure** (AES-based).

### 13.2 Key Material

| Symbol | Size | Description |
|--------|------|-------------|
| `pair_nn` | 16 B | Mesh network name (stored in flash) |
| `pair_pass` | 16 B | Mesh network password (stored in flash, AES-encoded) |
| `pair_ltk` | 16 B | Long-term key for mesh packet encryption |
| `pair_sk` | 16 B | Session key — derived per-session, never stored |
| `pair_randm` | 8 B | Device random challenge (sent to client) |
| `pair_rands` | 8 B | Client random challenge (received from client) |
| `pair_work` | 16 B | Scratch buffer for intermediate computation |

### 13.3 Pairing Opcodes (`pair_write`)

| Opcode | Hex | Direction | Description |
|--------|-----|-----------|-------------|
| `PAIR_OP_EXCHANGE_RANDOM` | `0x01` | Client → Device | Client sends 8-byte random; device records it as `pair_rands`; state → `AwaitingRandom` |
| `PAIR_OP_SET_MESH_NAME` | `0x04` | Client → Device | Encrypted mesh name; state → `ReceivingMeshName` |
| `PAIR_OP_SET_MESH_PASSWORD` | `0x05` | Client → Device | Encrypted mesh password; state → `ReceivingMeshPassword` |
| `PAIR_OP_SET_MESH_LTK` | `0x06` | Client → Device | Encrypted LTK; state → `ReceivingMeshLtk` |
| `PAIR_OP_GET_MESH_LTK` | `0x08` | Client → Device | Request LTK; state → `RequestingLtk` |
| `PAIR_OP_RESET_MESH` | `0x0A` | Client → Device | Reset mesh to defaults |
| `PAIR_OP_VERIFY_CREDENTIALS` | `0x0C` | Client → Device | Credential proof; state → `SessionKeyExchange` |
| `PAIR_OP_DELETE_PAIRING` | `0x0E` | Client → Device | Delete stored credentials; state → `DeletePairing` |

### 13.4 PairState Enum

| Value | Hex | Name | Description |
|-------|-----|------|-------------|
| 0 | `0x00` | `Idle` | No pairing in progress |
| 1 | `0x02` | `AwaitingRandom` | Device sent `pair_randm`; waiting for `pair_rands` |
| 2 | `0x05` | `ReceivingMeshName` | Processing `SET_MESH_NAME` |
| 3 | `0x06` | `ReceivingMeshPassword` | Processing `SET_MESH_PASSWORD` |
| 4 | `0x07` | `ReceivingMeshLtk` | Processing `SET_MESH_LTK` |
| 5 | `0x09` | `RequestingLtk` | Preparing LTK response |
| 6 | `0x0A` | `ResetMesh` | Resetting mesh configuration |
| 7 | `0x0B` | `DeletePairing` | Deleting stored pairing |
| 8 | `0x0C` | `RandomConfirmation` | Sending `pair_randm` to client |
| 9 | `0x0D` | `SessionKeyExchange` | Deriving and exchanging session key |
| 10 | `0x0E` | `Init` | Initialisation state (sends `pair_randm`, transitions to `RandomConfirmation`) |
| 11 | `0x0F` | `Completed` | Operation done; response ready to send |

### 13.5 Authentication Flow (`VERIFY_CREDENTIALS`)

**Simple mode** (when `pair_nn == pair_pass` after XOR, indicating legacy device):

```
SK[i] = pair_nn[i] XOR pair_pass[i]    (i = 0..15)
```

**Secure mode:**

```
1. key_padded  = pair_randm[0..8] || 0x00[0..8]          (16 B)
2. source      = pair_nn[i] XOR pair_pass[i]              (16 B)
3. sk_tmp      = AES_Encrypt(key_padded, source)
4. pair_sk     = sk_tmp[0..8] || pair_rands[0..8]        (16 B)
5. work        = pair_nn[i] XOR pair_pass[i]              (16 B)
6. pair_sk     = AES_Encrypt(work, pair_sk)
```

The device then encrypts a credential proof with `pair_sk` and sends it to the client via `pair_proc()`. The client verifies it by performing the same derivation independently.

### 13.6 `AwaitingRandom` Response

When in `AwaitingRandom`, `pair_proc()` sends the 8-byte device random challenge (`pair_randm`) to the client. State → `RandomConfirmation`.

### 13.7 `ReceivingMeshLtk` Response

```
work[i] = pair_nn[i] XOR pair_pass[i] XOR pair_ltk[i]    (i = 0..15)
response = AES_Encrypt(pair_sk, work)
```
Response is sent to client. State → `Completed`.

### 13.8 LTK Retrieval (`RequestingLtk`)

**Simple mode:** LTK sent in plaintext.

**Secure mode:**
```
work[0..8]   = pair_randm
work[8..16]  = pair_nn[i] XOR pair_pass[i] XOR pair_sk[i]   (i = 0..7)
encrypted_ltk = AES_Encrypt(pair_sk, pair_ltk)
```
`pair_sk` is then restored from `pair_sk_copy`. State → `Completed`.

### 13.9 Packet Encryption

All post-session data PDUs on the Command characteristic (`0x1912`) are AES-CCM encrypted using `pair_sk`. The packet encryption wrapper is `aes_att_encryption_packet()` and decryption is `aes_att_decryption_packet()` — see §16 for the AES hardware procedure.

Mesh packets are encrypted using `pair_ltk` via `pair_enc_packet_mesh()` / `pair_dec_packet_mesh()`.

---

## 14. Mesh Advertising

### 14.1 Overview

The device continuously broadcasts mesh node status via BLE advertisements when not in a BLE connection, or interleaves mesh advertisements with connection events when connected. Mesh advertisements allow nearby devices to learn the state of all nodes in the network.

### 14.2 `MeshNodeStValT` — Node Status Value (4 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 B | `dev_adr` | Mesh node address (1–63) |
| 1 | 1 B | `sn` | Sequence number — increments on each state change |
| 2–3 | 2 B | `par` | Parameters — typically `[lumen, reserved]` |

`MESH_NODE_ST_VAL_LEN = 4`

### 14.3 Mesh Advertising Data Construction (`mesh_node_adv_status`)

The advertising payload is filled with consecutive `MeshNodeStValT` records. The local device's own entry is always placed at index 0. Subsequent slots cycle through known mesh nodes (round-robin via `MESH_NODE_CUR`), up to the buffer capacity. Nodes with `tick == 0` (timed out) are skipped.

### 14.4 Node Tracking and Timeout

`MESH_NODE_ST` holds up to `MESH_NODE_MAX_NUM` entries. Each entry has a `tick` field (16-bit, derived from the upper 16 bits of the 32-bit system tick). A node is marked stale if:

```
current_tick - node.tick > MESH_NODE_TIMEOUT_THRESHOLD
```

Stale nodes are flushed by `mesh_node_flush_status()` (sets `tick = 0`) and their slot is set in `MESH_NODE_MASK` to trigger a status update notification.

### 14.5 Self-Keep-Alive (`mesh_node_keep_alive`)

`mesh_node_keep_alive()` refreshes the device's own entry (index 0):
- Updates `sn` to `DEVICE_NODE_SN`
- Sets `tick` to `(read_reg_system_tick() >> 16) | 1`

### 14.6 Mesh Packet Relay

When a mesh command is received (e.g. from a BLE client or another mesh node), it is re-broadcast over the mesh. The `internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT]` field tracks how many times the packet has been re-broadcast; packets are dropped once the retransmit count reaches the maximum.

---

## 15. OTA Protocol (BLE)

### 15.1 Overview

OTA firmware updates are delivered over the **OTA** GATT characteristic (UUID `0x1913`). Packets are written with `Write Command` (no response). The firmware is written to a staging area in flash, then verified and applied on reboot.

### 15.2 `OtaState` Enum

| Value | Name | Description |
|-------|------|-------------|
| 0 | `Continue` | Update in progress — awaiting more packets |
| 1 | `Ok` | Transfer complete and verified — ready to reboot |
| 2 | `Error` | Transfer failed — abort |
| 3 | `MasterOtaRebootOnly` | Master initiated OTA complete — reboot only |

### 15.3 Flash Storage for OTA

| Parameter | Value |
|-----------|-------|
| Staging base address | `FLASH_ADR_LIGHT_NEW_FW = 0x40000` |
| Maximum firmware size | `FW_SIZE_MAX_K = 128 KB` |
| Sectors erased for OTA | `ceil(128 / 4) = 32 sectors` (each 4 KB) |
| Firmware size field offset | `+0x18` from start of staged firmware |

### 15.4 BLE OTA Packet Format

Each OTA write to the characteristic contains:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0–1 | 2 B | `index` | Packet index (little-endian) |
| 2–(N-3) | N-4 B | `data` | Firmware bytes |
| N-2, N-1 | 2 B | `crc16` | CRC-16 of `[index(2B) || data]` (little-endian) |

Packet data length = 16 bytes (BLE) or 8 bytes (UART/mesh relay).

### 15.5 OTA Completion and Verification

When the last packet is received (`cur_idx == ota_pkt_total`):

1. Read `new_fw_size` from `FLASH_ADR_LIGHT_NEW_FW + 0x18` (4 bytes, little-endian).
2. Compute `ota_pkt_total = ceil(new_fw_size / packet_len)`.
3. Verify that at least 3 packets were received.
4. Read current device firmware version from flash.
5. Compare crystal frequency flag (`VERSION_16MHZ` bit) — mismatch → `OtaState::Error`.
6. Check version ≥ current (unless `--force`) — downgrade → `OtaState::Error`.
7. All checks passed → `OtaState::Ok` → device reboots and applies the staged firmware.

### 15.6 Version and Crystal Check

The firmware version word is stored at `FLASH_ADR_LIGHT_NEW_FW + 0x18`. The bit layout:

| Bit | Name | Description |
|-----|------|-------------|
| 31 | `VERSION_16MHZ` | `1` = firmware requires 16 MHz crystal |
| 30–0 | Version number | Monotonically increasing build number |

The `--force` flag bypasses the version number comparison but **never** bypasses the crystal frequency check.

---

## 16. AES / Cryptography Hardware

### 16.1 Registers

| Register | Address | Width | Description |
|----------|---------|-------|-------------|
| `reg_aes_ctrl` | `0x540` | 8-bit | Control register: bit 0 = direction (0=encrypt, 1=decrypt); bit 2 = done flag (read; set by hardware when operation completes) |
| `reg_aes_data` | `0x548` | 32-bit | Single auto-advancing 32-bit FIFO register. Each write shifts in one word of source data; each read shifts out one word of result data. The address does **not** increment — the register itself advances its internal pointer automatically. |
| `reg_aes_key[i]` | `0x550 + i` | 8-bit | Key byte `i` (write only; 16 bytes, i = 0–15) |

> **Caution:** The `reg_aes_data` register is a FIFO, **not** an addressable array. Do not construct indexed addresses from 0x548. Writing 4 consecutive 32-bit values to address 0x548 loads all 16 bytes of source data; reading 4 consecutive 32-bit values from address 0x548 retrieves the 16-byte result.

### 16.2 Low-Level AES Procedure (`aes_ll_encryption`)

```
Input:  key[16], source[16], direction ∈ {Encrypt=0, Decrypt=1}

1. Write direction bit to reg_aes_ctrl (0x540).

2. Write key bytes in REVERSED order (key[15] → index 0, key[0] → index 15):
     for i in 0..16: reg_aes_key[i] = key[15 - i]

3. Write source data as four big-endian 32-bit words in REVERSE index order.
   The hardware expects word index 3 first, word index 0 last:
     for i in [3, 2, 1, 0]:
         word = u32::from_be_bytes(source[i*4 .. i*4+4])
         write_u32(reg_aes_data /*0x548*/, word)

   (The FIFO auto-advances after each 32-bit write; do not change the address.)

4. Poll reg_aes_ctrl (0x540) until bit 2 (0x04) is set (done flag).

5. Read result as four 32-bit LE words in FORWARD index order:
     for i in 0..4:
         word = read_u32(reg_aes_data /*0x548*/)
         result[i*4]   = (word >>  0) & 0xFF  // little-endian byte extraction
         result[i*4+1] = (word >>  8) & 0xFF
         result[i*4+2] = (word >> 16) & 0xFF
         result[i*4+3] = (word >> 24) & 0xFF

6. Return result[16].
```

**Latency:** Typically completes within a few microseconds at 32 MHz.

### 16.3 ATT-Level Wrappers

| Function | Operation | Post-processing |
|----------|-----------|-----------------|
| `aes_att_encryption(key, src)` | `aes_ll_encryption(Encrypt)` | Byte-reverse the 16-byte result |
| `aes_att_decryption(key, src)` | `aes_ll_encryption(Decrypt)` | Byte-reverse the 16-byte result |
| `encode_password(pwd)` | `aes_att_encryption(GLOBAL_KEY, pwd)` | Password obfuscation for flash storage |
| `decode_password(pwd)` | `aes_att_decryption(GLOBAL_KEY, pwd)` | Password de-obfuscation on load |

The global password encryption key is `MESH_PWD_ENCODE_SK = "0123456789ABCDEF"` (16 ASCII bytes).

### 16.4 Packet Encryption (`aes_att_encryption_packet`)

Implements AES-CCM-like authenticated encryption:

1. **Phase 1 — Counter (CTR) mode decryption/encryption** of packet payload.
2. **Phase 2 — CBC-MAC (MIC) generation** over the plaintext payload using the same key and IV.

The IV is constructed from fields within the packet header. The 4-byte MIC is appended to or stripped from the packet.

---

## 17. Flash Storage

### 17.1 Flash Address Map

| Address | Size | Symbol | Contents |
|---------|------|--------|----------|
| `0x40000` | 128 KB | `FLASH_ADR_LIGHT_NEW_FW` | OTA staging area for new firmware |
| `0x76000` | 4 KB | `FLASH_ADR_MAC` | Device BLE MAC address |
| `0x77000` | 4 KB | `FLASH_ADR_PAIRING` | Mesh credentials (wear-leveled, 64-byte slots) |
| `0x78000` | 4 KB | `FLASH_ADR_LUM` | Luminosity / brightness state |
| `0x79000` | 4 KB | `FLASH_ADR_DEV_GRP_ADR` | Mesh group address list (max 0x1000 B / 2048 entries) |
| `0x7A000` | 4 KB | `FLASH_ADR_RESET_CNT` | Reset counter (used for factory reset detection) |
| `0x7B000` | 4 KB | `FLASH_ADR_PANIC_INFO` | Panic / crash information |

### 17.2 Pairing Sector Layout (`0x77000`, 4 KB)

The pairing sector is divided into **64-byte slots** for wear-leveling. The active slot is tracked by `FLASH_CONFIGURATION_INDEX` (offset within the sector). When the index reaches the end of the sector it wraps and the sector is erased.

Within each 64-byte slot:

| Slot Offset | Size | Symbol | Contents |
|-------------|------|--------|----------|
| `+0x00` | 16 B | `OFFSET_HEADER` | Validity header (`PAIR_VALID_FLAG = 0xFA` at byte 0) |
| `+0x10` | 16 B | `OFFSET_NAME` | Mesh network name (null-padded) |
| `+0x20` | 16 B | `OFFSET_PASSWORD` | Mesh password (AES-encoded with `MESH_PWD_ENCODE_SK`) |
| `+0x30` | 16 B | `OFFSET_LTK` | Long-term key (plaintext) |

A slot is valid if its header byte equals `PAIR_VALID_FLAG (0xFA)`. On load (`pair_load_key`), the sector is scanned from the beginning to find the latest valid slot.

### 17.3 Default Credentials

| Parameter | Default Value |
|-----------|---------------|
| Mesh name | `"out_of_mesh"` |
| Mesh password | `"123"` |
| Mesh LTK | `[0x00, 0x00, ..., 0x00]` (16 zero bytes) |
| Device name (GATT) | `"Telink tLight"` |
| Vendor ID | `0x0211` |

---

## 18. Timing Reference

### 18.1 System Clock

| Parameter | Value | Notes |
|-----------|-------|-------|
| System clock (`CLOCK_SYS_CLOCK_HZ`) | 32 MHz | Post-PLL |
| PLL clock (`CLOCK_PLL_CLOCK`) | 192 MHz | PLL oscillator |
| `CLOCK_SYS_CLOCK_1US` | 32 ticks | Ticks per microsecond |
| `CLOCK_SYS_CLOCK_1MS` | 32,000 ticks | Ticks per millisecond |
| `CLOCK_SYS_CLOCK_1S` | 32,000,000 ticks | Ticks per second |
| `CLOCK_SYS_CLOCK_4S` | 128,000,000 ticks | 4-second overflow guard |
| Max addressable interval (ms) | `u32::MAX / 32000 ≈ 134,217` ms | ~134 s |

### 18.2 BLE Connection Timing

All connection timing is expressed in units of **1250 µs** (one BLE slot = 1250 µs = 40,000 ticks at 32 MHz).

| Parameter | Formula | Example |
|-----------|---------|---------|
| Connection interval | `interval × 1250 µs` | `interval=20` → 25 ms |
| Window offset | `(woffset + 1) × 1250 µs` | `woffset=4` → 6250 µs |
| Window size | `(wsize × 1250 + 1100) µs` | `wsize=1` → 2350 µs |
| Timing correction — early threshold | `700 µs` | Advance by 200 µs |
| Timing correction — late threshold | `1100 µs` | Retard by 200 µs |
| Timing correction step | `200 µs` | Applied per event |
| Connection param update delay | `UPDATE_CONNECT_PARA_DELAY_MS × 1000 µs` | Implementation-specific |
| Latency constraint | `latency × interval × 2 ≤ interval × 8` | Simplified: `latency ≤ 4` |
| Supervision timeout (LL default) | 1000 ms | Set on receiving `0x02` LL control |

### 18.3 Advertising Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| ADV channel sequence | 37 → 38 → 39 | Fixed BLE primary advertising channels |
| IRQ timer source | Timer 1 | `FLD_IRQ::TMR1_EN` |
| RF IRQ source | ZigBee/BLE RT | `FLD_IRQ::ZB_RT_EN` |

### 18.4 OTA Timing

| Parameter | Value |
|-----------|-------|
| BLE OTA packet data size | 16 bytes |
| UART OTA packet data size | 8 bytes |
| UART baud rate | 115,200 |
| UART ACK timeout | 200 ms |
| Max UART retries per packet | 100 |

---

## 19. Mesh Protocol Overview

The TLSR8266 firmware implements a **proprietary flooding mesh** layered on top of the BLE RF hardware. It is entirely distinct from Bluetooth Mesh (SIG specification) and has no compatibility with it. The same 2.4 GHz radio is shared between BLE advertisements, BLE connections, and mesh packet relay, with a cooperative state machine controlling which role is active at any time.

### 19.1 Protocol Philosophy

| Property | Value |
|----------|-------|
| Transport | BLE RF hardware (same radio, same modulation) |
| Topology | Flat flooding — every node repeats every packet |
| Addressing | 1-byte node address (1–254); `0xFF` = broadcast |
| Security | AES-128 encryption per packet when `SECURITY_ENABLE` is set |
| Access code | Derived from mesh name + password; hardware-level network isolation |
| Vendor ID | 16-bit constant (`VENDOR_ID`) included in every mesh command |

All mesh nodes are peers. There is no designated coordinator, router, or gateway. A smartphone connects to whichever node is in range and issues commands; that node relays the command across the mesh via flooding.

### 19.2 Key Constants

| Constant | Value | Location | Meaning |
|----------|-------|----------|---------|
| `SYS_CHN_LISTEN` | `[2, 12, 23, 34]` | `common.rs` | BLE data channels used for mesh RX/TX |
| `SYS_CHN_ADV` | `[0x25, 0x26, 0x27]` | `common.rs` | BLE advertising channels (37, 38, 39) |
| `ADV_INTERVAL2LISTEN_INTERVAL` | `4` | `sdk/light.rs` | Advertise every 4th listen cycle |
| `ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL` | `8` | `sdk/light.rs` | Send online status every 8th cycle |
| `MESH_LISTEN_INTERVAL_US` | `100_000` (100 ms) | `status_management.rs` | Duration of one listen cycle |
| `BRIDGE_MAX_CNT` | `8` | `sdk/light.rs` | Maximum relay hops per packet |
| `ONLINE_STATUS_TIMEOUT` | `3000` ms | `mesh_management.rs` | Inactivity before node marked offline |
| `MESH_PAIR_CMD_INTERVAL` | `500` ms | `mesh.rs` | Interval between mesh-pair credential frames |
| `MESH_PAIR_TIMEOUT` | `10` s | `mesh.rs` | Total time allowed to complete re-pairing |
| `MESH_NODE_MAX_NUM` | (compile-time) | `state.rs` | Maximum nodes tracked in status table |

### 19.3 Channel Strategy

Mesh packets are transmitted on **BLE data channels** (not advertising channels), specifically the four channels `[2, 12, 23, 34]`. This avoids the congested advertising channels (37/38/39) and allows the radio to listen for mesh traffic even when BLE advertising or connection events are not in progress.

```
BLE Advertising channels: 37 (0x25), 38 (0x26), 39 (0x27)  — used by BLE adv only
Mesh data channels:         2,        12,        23,        34 — used by mesh TX and RX
```

When transmitting a mesh packet, the firmware iterates over **all four** mesh channels in a random order, transmitting the packet on each. This ensures reception by all neighbours regardless of which channel they are currently listening on.

---

## 20. RF Hardware Configuration for Mesh

The same RF peripheral used for BLE is reconfigured on-the-fly when switching between BLE and mesh operation. The key differences are:

| Parameter | BLE Advertising | BLE Connected | Mesh |
|-----------|-----------------|---------------|------|
| Access code | Standard BLE preamble AC | Standard BLE connection AC | `PAIR_AC` (derived from credentials) |
| CRC | `rf_set_ble_crc_adv()` | Connection CRC | `rf_set_ble_crc_adv()` |
| Channel | 37 / 38 / 39 | Negotiated data channel | `SYS_CHN_LISTEN[cycle % 4]` |
| RF mode | TX (adv) | RX + TX | RX (listen), then SRX2TX (transmit) |
| Encryption | None at RF layer | BLE LL encryption | AES-128 at application layer |

### 20.1 Mesh RX Configuration

`configure_rf_for_mesh_listening()` (`irq/mesh.rs`):

```
rf_stop_trx()                                   // halt any ongoing operation
rf_set_ble_access_code(PAIR_AC.get())           // filter to this mesh network only
rf_set_ble_crc_adv()                            // use advertising CRC format
rf_set_ble_channel(SYS_CHN_LISTEN[cycle % 4])  // select channel in round-robin
rf_set_rxmode()                                 // enable receiver
BLE_PERIPHERAL_LINK_STATE = Mesh               // update state flag
```

The channel index (`MESH_LISTEN_CYCLE_COUNT`) advances by 1 each listen cycle, causing the device to rotate through all four mesh channels over successive 100 ms windows.

### 20.2 Mesh TX Configuration

`send_mesh_msg_iteration()` (`mesh.rs`) — called from the async mesh TX task:

```
// Pick a random starting channel (XOR of system tick and hardware RNG)
start_chan_idx = (read_reg_system_tick() ^ read_reg_rnd_number()) as usize

for channel_index in start_chan_idx .. start_chan_idx + 4:
    rf_set_tx_rx_off()
    rf_set_ble_access_code(PAIR_AC.get())
    rf_set_ble_crc_adv()
    rf_set_ble_channel(SYS_CHN_LISTEN[channel_index % 4])
    rf_start_srx2tx(packet_addr, now + 30 µs)   // simultaneous RX+TX mode
    Timer::after(600 µs).await                  // wait for TX to complete

rf_set_rxmode()   // return to receive after all channels transmitted
```

The random start channel and 600 µs inter-channel gap ensure that each transmission completes before the next begins, and the random starting point reduces the probability that two nodes collide on the same channel sequence.

### 20.3 Retransmission

After transmitting on all four channels, if the packet's `send_count` is nonzero, the packet is re-queued with:

```
delay = 8000 - (((tick ^ rnd) % 16) * 500)   // 0–8000 µs random backoff
send_count -= 1
```

This adds up to 8 ms of random jitter between retransmissions, preventing synchronised collisions when multiple nodes relay the same packet simultaneously.

### 20.4 Access Code Derivation

`access_code(name, pass)` (`common.rs`):

```
Stage 1 — AES encryption:
    key       = pass[0..16] (zero-padded)
    plaintext = name[0..16] (zero-padded)
    result    = AES-128-ECB(key=pass, plaintext=name)
    candidate = result[0]  (first 32 bits)

Stage 2 — Consecutive-bit limiting:
    Scan the 32-bit candidate for runs of identical bits longer than 6.
    Flip bits to break up runs > 6 consecutive identical bits.

Stage 3 — Bit density validation:
    Using test patterns 0xAAAAAAAA then 0x55555555,
    if fewer than 3 bits match, XOR lower byte with 0xFF.
    This ensures sufficient bit transitions for reliable radio sync.
```

The resulting 32-bit value is stored in `PAIR_AC` and loaded into the RF access code register. All nodes in the same mesh network derive the same access code from the shared name and password — the RF hardware will **reject at silicon level** any packet whose preamble does not match `PAIR_AC`. Devices on different networks are invisible to each other.

---

## 21. BLE / Mesh Coexistence — State Machine

### 21.1 `RfOperationState` Enum

The global `CURRENT_RF_STATE` variable (type `RfOperationState`) controls which handler the system-timer IRQ invokes. This is **distinct** from `BLE_PERIPHERAL_LINK_STATE` (type `BlePeripheralLinkState`) which controls packet-reception dispatch (see §7.1). Both variables coexist and must be kept consistent.

`RfOperationState` has **no explicit discriminant values** — the compiler assigns sequential integers:

| Variant | Compiler Value | Role in system-timer IRQ dispatch |
|---------|---------------|----------------------------------|
| `Idle` | 0 | No RF operation; IRQ does nothing |
| `Advertising` | 1 | Transmit ADV_IND on channels 37/38/39, then enter `MeshListening` |
| `Connected` | 2 | Run BLE connection event; bridge commands to mesh |
| `Receiving` | 3 | Configure RX window for next connection event |
| `MeshListening` | 4 | Listen on mesh channel; periodically enter `Advertising` |

Compare with `BlePeripheralLinkState` which uses **explicit non-sequential discriminants**: Disconnected=0, Advertising=1, Mesh=4, Connected=5, Receiving=7 (values 2, 3, 6 are invalid gaps).

### 21.2 State Transition Diagram

```
                      ┌─────────────────────────────────────────┐
                      │              Power-on / Reset            │
                      └────────────────────┬────────────────────┘
                                           ▼
                                   ┌───────────────┐
                              ┌───►│ MeshListening │◄───────────────────┐
                              │    └───────┬───────┘                    │
                              │            │ every 4th cycle            │
                              │            │ (ADV_INTERVAL2LISTEN = 4)  │
                              │            ▼                            │
                              │    ┌───────────────┐                    │
                              │    │  Advertising  │                    │
                              │    └───────┬───────┘                    │
                              │            │ connection request          │
                              │            │ received                    │
                              │            ▼                            │
                              │    ┌───────────────┐   disconnect       │
                              └────│   Connected   │────────────────────┘
                                   └───────────────┘
```

### 21.3 Timing Ratios

The listen cycle counter (`MESH_LISTEN_CYCLE_COUNT`) increments each 100 ms window:

```
cycle % 4 == 0  →  Advertising window (BLE adv packets transmitted)
cycle % 8 == 0  →  Online status broadcast via mesh (in addition to advertising)
other cycles    →  Pure mesh listening
```

Concrete timing with default 100 ms listen interval:

| Event | Period |
|-------|--------|
| Mesh listen window | 100 ms |
| BLE advertising opportunity | 400 ms |
| Online status broadcast | 800 ms |

### 21.4 Advertising Window Randomisation

When transitioning to the `Advertising` state, the IRQ timer is randomised to prevent all nodes from advertising simultaneously:

```rust
write_reg_system_tick_irq(
    (((read_reg_system_tick() ^ read_reg_rnd_number() as u32) & 0x7fff)
        * CLOCK_SYS_CLOCK_1US
    + read_reg_system_tick_irq()),
);
```

The `& 0x7fff` mask caps the random offset at 32,767 µs (~32 ms), spreading advertising events across a 32 ms window to reduce collision probability in dense deployments.

### 21.5 Connected State — Mesh Bridge

When a BLE connection is active (`Connected` state), the device acts as a bridge: commands received over BLE GATT are translated to mesh packets and relayed across the mesh network via `tx_packet_bridge()`.

```
Smartphone ──(BLE GATT write)──► Connected node
                                         │
                                  app_bridge_cmd_handle()
                                         │
                                  Adaptive relay delay:
                                  relay_time = min(
                                    (elapsed_us + 500) >> 10,
                                    255
                                  )
                                         │
                                  mesh_manager.add_send_mesh_msg()
                                         │
                              ┌──────────┴──────────┐
                              ▼                     ▼
                         Node 0x02            Node 0x03
                              │                     │
                         (relay)               (relay)
                              │                     │
                         Node 0x04 ──── ... ──── Node 0x1F
```

`tx_packet_bridge()` also manages the periodic online status broadcast when the device is connected, using the same `ONLINE_STATUS_INTERVAL2LISTEN_INTERVAL` ratio.

---

## 22. Mesh Packet Format

### 22.1 `MeshPkt` Structure (48 bytes)

```
Offset  Size  Field           Description
──────  ────  ─────────────── ────────────────────────────────────────────
 0       10   head            PacketL2capHead (DMA len, type, rf_len, l2cap, chan_id)
10        2   src_tx          Address of the node that most recently transmitted this packet
12        1   handle1         Flag byte (bit 7 = encrypted when SECURITY_ENABLE)
13        3   sno             Sequence number (24-bit, little-endian)
16        2   src_adr         Originating node address
18        2   dst_adr         Destination address (0xFFFF = broadcast)
20        1   op              Opcode (with 0xC0 ORed in for mesh commands)
21        2   vendor_id       Vendor identifier (VENDOR_ID constant)
23       10   par             Application parameters (command-specific)
33        5   internal_par1   Internal relay parameters (see §22.2)
38        1   ttl             Time-to-live / hop counter
39        4   internal_par2   Internal relay parameters (extended)
43        5   no_use          Padding (must be 48 bytes total for DMA TX address alignment)
```

Total: 48 bytes. Verified at compile time: `const_assert!(size_of::<MeshPkt>() == 48)`.

### 22.2 `internal_par1` Field Layout

| Index | Constant | Meaning |
|-------|----------|---------|
| `[0]` | `INTERNAL_PAR_STATUS_DATA` | Status/response data |
| `[1]` | `INTERNAL_PAR_PACKET_FORMAT_MODE` | Packet format mode indicator |
| `[2]` | `INTERNAL_PAR_RETRANSMIT_COUNT` | Remaining relay hops (decrements at each node) |
| `[3]` | `INTERNAL_PAR_SEND_ACK` | `1` = destination should send an acknowledgement |
| `[4]` | (reserved) | Reserved |

### 22.3 `par` Field (Application Parameters)

The `par[10]` field is overlaid with application-layer fields when viewed as `PacketAttValue.val`:

```
val[0..2]   op[0..2]          Opcode bytes (op | 0xC0, vendor_id_lo, vendor_id_hi)
val[3..12]  params[0..9]      Command-specific parameters
val[13]     PKT_VAL_MAC_APP_TTC           Time-to-complete / status field
val[14]     PKT_VAL_MAC_APP_HOP_COUNT     Mesh hop counter
val[15]     PKT_VAL_MAC_APP_RESPONSE_TYPE Response type (GET_STATUS, GET_GROUP, etc.)
val[16]     PKT_VAL_MAC_APP_LINK_INTERVAL BLE connection link interval
val[17]     PKT_VAL_MAC_APP_RELAY_TIMING  Relay timing (congestion control, units = 1024 µs)
```

### 22.4 Sequence Number

The 24-bit `sno` field is used for **duplicate suppression**. When a node receives a mesh packet, it checks whether it has already processed a packet with this `(src_adr, sno)` pair. If so, it discards the packet. This prevents the flooding from looping indefinitely.

Sequence numbers are generated as:

```rust
cmd_sno = clock_time() + DEVICE_ADDRESS as u32
```

### 22.5 Routing and Flooding

There is no routing table. Every node that receives a packet and passes duplicate-suppression **re-queues it for transmission** on all four mesh channels. Propagation is therefore a flood that terminates naturally via:

1. `INTERNAL_PAR_RETRANSMIT_COUNT` reaching zero (counts down at each relay hop)
2. Duplicate suppression (each node processes each SNO only once)
3. `BRIDGE_MAX_CNT = 8` — hard cap on relay count per packet

---

## 23. Mesh Command Opcode Reference

All mesh opcodes are defined in `rust/src/sdk/light.rs`. Commands are transmitted as `op | 0xC0` in the packet opcode byte to distinguish them from BLE ATT opcodes.

| Constant | Hex | Direction | Description |
|----------|-----|-----------|-------------|
| `LGT_CMD_NOTIFY_MESH` | `0x02` | Node → App | Mesh notification / status event |
| `LGT_CMD_MESH_PAIR` | `0x09` | Node → Node | Credential distribution during re-pairing |
| `LGT_CMD_LIGHT_ONOFF` | `0x10` | App → Node(s) | Turn light on or off |
| `LGT_CMD_LIGHT_STATUS` | `0x1B` | Node → App | Report current light status |
| `LGT_CMD_KICK_OUT` | `0x23` | App → Node | Remove node from mesh (factory reset) |
| `LGT_CMD_START_OTA_REQ` | `0x24` | App → Node | Begin OTA firmware update |
| `LGT_CMD_START_OTA_RSP` | `0x25` | Node → App | Confirm OTA session started |
| `LGT_CMD_TRANSFER_OTA_DATA` | `0x26` | App → Node | OTA data chunk (8 bytes) |
| `LGT_CMD_END_OTA_REQ` | `0x28` | App → Node | Signal end of OTA data |
| `LGT_CMD_END_OTA_RSP` | `0x29` | Node → App | Confirm OTA complete |
| `LGT_CMD_SET_LIGHT` | `0x30` | App → Node(s) | Set light brightness/colour/CT |
| `LGT_CMD_SET_DEV_ADDR` | `0x08` | App → Node | Assign mesh node address |
| `LGT_CMD_CONFIG_DEV_ADDR` | `0x0A` | App → Node | Configure device address with validation |
| `LGT_CMD_SET_MESH_INFO` | `0xC0` | Internal | Reload mesh credentials; clear online status |
| `LGT_CMD_DEL_PAIR` | `0xC1` | App → Node | Delete pairing, revert to default mesh |
| `LGT_CMD_MESH_CMD_NOTIFY` | `0x05` | Internal | Internal notification relay |

Commands directed at `dst_adr = 0xFFFF` target all nodes (broadcast). Commands with a specific `dst_adr` are still flooded by intermediate nodes but only acted upon by the matching node.

---

## 24. Mesh Node Status Propagation

### 24.1 `MeshNodeStValT` (4 bytes)

Each node maintains a 4-byte status record:

```
Offset  Size  Field    Description
──────  ────  ──────── ─────────────────────────────
 0       1    dev_adr  Mesh node address (1–63)
 1       1    sn       Status sequence number
 2       2    par      Parameters: par[0]=lumen, par[1]=reserved
```

### 24.2 `MeshNodeStT` — Full Status Entry (6 bytes)

The runtime table stores each node's status with a tick counter for timeout detection:

```
Offset  Size  Field  Description
──────  ────  ─────  ─────────────────────────────────────────
 0       2    tick   Scaled system tick at last update (0 = offline)
 2       4    val    MeshNodeStValT (dev_adr, sn, par)
```

Tick values are stored as `(clock_time() as u16)` for compact representation. Timeout is detected by comparing the stored tick with the current time using a scaled threshold:

```
timeout_threshold = (CLOCK_SYS_CLOCK_1US × ONLINE_STATUS_TIMEOUT × 1000) >> 16
```

If `current_tick - stored_tick > timeout_threshold`, the node is marked offline (`tick = 0`).

### 24.3 Status Update Algorithm

`mesh_node_update_status()` processes incoming `MeshNodeStValT` records:

```
For each incoming node record:
  1. If dev_adr == self address: skip (no self-update)
  2. Search existing table for matching dev_adr
  3. If found:
     a. Accept update if new_sn - old_sn < 0x3F (forward progress)
        OR if node was offline (tick == 0)
        OR if significant time has elapsed
     b. Update tick, sn, par
     c. Set MESH_NODE_MASK bit for this node (triggers status report)
  4. If not found AND table not full:
     a. Allocate new entry
     b. Initialise tick, sn, par
     c. Set MESH_NODE_MASK bit
```

The 32-bit bitmask `MESH_NODE_MASK` tracks which nodes have pending status updates that need to be reported to a connected BLE master.

### 24.4 Status Report to BLE Master

`mesh_node_report_status()` is called when a BLE master queries node status:

```
For each set bit in MESH_NODE_MASK:
  1. Clear the bit (one-shot reporting)
  2. Copy MeshNodeStValT into output buffer
  3. If tick == 0 (offline), set sn = 0 in report
  4. Advance buffer pointer by MESH_NODE_ST_VAL_LEN (4 bytes)
  5. Stop if output buffer is full (chunked reporting across multiple packets)
```

### 24.5 Online Status Broadcast

`mesh_send_online_status()` is called every 8th listen cycle (every 800 ms):

```
1. Collect current status of all known nodes (mesh_node_adv_status())
2. Build a mesh packet with opcode LGT_CMD_LIGHT_STATUS
3. Inject into mesh TX queue via add_send_mesh_msg()
```

This keeps the entire mesh informed of every node's current lumen level and sequence number. Nodes receiving this broadcast update their local `MESH_NODE_ST` table via `mesh_node_update_status()`.

### 24.6 Timeout and Flush

`mesh_node_flush_status()` is called periodically to detect stale nodes:

```
For node index 1..MESH_NODE_MAX:
  if tick != 0 AND (now - tick) > timeout_threshold:
    tick = 0           // mark offline
    sn   = 0           // clear sequence
    set MESH_NODE_MASK bit  // trigger status report to BLE master
```

A node is removed from the active set after `ONLINE_STATUS_TIMEOUT = 3000 ms` of silence.

---

## 25. Mesh Provisioning (Over-Mesh Re-pairing)

### 25.1 Purpose

When the user adds a new node to an existing mesh via the mobile app, the app sends new network credentials (name, password, LTK) to the currently-connected node. That node relays the credentials across the mesh to all other nodes using the `LGT_CMD_MESH_PAIR` command sequence.

### 25.2 `MeshPairState` Enumeration

| Value | Variant | Description |
|-------|---------|-------------|
| `0` | `MeshPairName1` | First 8 bytes of the new mesh name |
| `1` | `MeshPairName2` | Second 8 bytes of the new mesh name |
| `2` | `MeshPairPwd1` | First 8 bytes of the new password |
| `3` | `MeshPairPwd2` | Second 8 bytes of the new password |
| `4` | `MeshPairLtk1` | First 8 bytes of the new LTK |
| `5` | `MeshPairLtk2` | Second 8 bytes of the new LTK |
| `6` | `MeshPairEffectDelay` | Apply credentials after a delay |
| `7` | `MeshPairEffect` | Apply credentials immediately (receive-only) |
| `8` | `MeshPairDefaultMesh` | Revert to factory default mesh credentials |

### 25.3 Credential Relay Sequence

```
App                  Connected Node              Other Nodes
 │                         │                         │
 │──LGT_CMD_MESH_PAIR ────►│                         │
 │   state=Name1            │──broadcast(Name1)──────►│
 │   data[8]=name[0..8]     │                         │ stores name[0..8]
 │                          │                         │
 │──LGT_CMD_MESH_PAIR ────►│                         │
 │   state=Name2            │──broadcast(Name2)──────►│
 │   data[8]=name[8..16]    │                         │ stores name[8..16]
 │                          │                         │
 │  (repeat for Pwd1, Pwd2, Ltk1, Ltk2)              │
 │                          │                         │
 │──LGT_CMD_MESH_PAIR ────►│                         │
 │   state=EffectDelay      │──broadcast(EffectDelay)►│
 │                          │                         │ waits MESH_PAIR_CMD_INTERVAL×2
 │                          │                         │ then calls save_effect_new_mesh()
 │                          │ calls save_effect_new_mesh()
 │                          │ • pair_save_key()
 │                          │ • rf_set_ble_access_code(new PAIR_AC)
 │                          │ • LGT_CMD_SET_MESH_INFO (clear online table)
```

Timing:
- Each credential frame is sent every `MESH_PAIR_CMD_INTERVAL = 500 ms`
- The entire sequence (6 frames) takes ~3 seconds
- `MESH_PAIR_TIMEOUT = 10 s` — if not completed, `LGT_CMD_MESH_PAIR_TIMEOUT` is fired
- After `MeshPairEffectDelay`, each node independently applies the new credentials after `MESH_PAIR_CMD_INTERVAL × 2 = 1000 ms`, staggering the network transition

### 25.4 `mesh_pair_cb()` — Receiving Credentials

Each node receiving `LGT_CMD_MESH_PAIR` calls `mesh_pair_cb(params)`:

```
params[0]    = MeshPairState variant
params[1..9] = 8 bytes of credential data
```

The state machine assembles the 16-byte name, password, and LTK from the two 8-byte halves. On `MeshPairEffectDelay`, the node schedules a call to `save_effect_new_mesh()` which:

1. Writes new credentials to flash via `pair_save_key()`
2. Regenerates `PAIR_AC` from new name+password
3. Immediately reloads the new access code into the RF hardware
4. Broadcasts `LGT_CMD_SET_MESH_INFO` to clear the network status table
5. Re-initialises the node status table (`mesh_node_init()`)

From this point the node is a member of the new mesh and is invisible to nodes on the old network.

### 25.5 Default Mesh Reversion

`MeshPairDefaultMesh` causes the node to revert to the factory-default mesh credentials (stored separately as `PAIR_CONFIG_MESH_NAME` / `PAIR_CONFIG_MESH_PWD`). The delay (`params[1]` seconds) staggers the transition across the network to prevent all nodes from simultaneously losing contact with the app.

---

## 26. Over-Mesh OTA Firmware Update

When a BLE-connected node initiates OTA update for a remote (non-connected) node, it relays OTA packets through the mesh using the standard mesh flood mechanism.

### 26.1 OTA Command Sequence over Mesh

| Step | Command | Direction | Description |
|------|---------|-----------|-------------|
| 1 | `LGT_CMD_START_OTA_REQ` (`0x24`) | App → Target | Begin OTA session, specify target address |
| 2 | `LGT_CMD_START_OTA_RSP` (`0x25`) | Target → App | Confirm ready |
| 3 | `LGT_CMD_TRANSFER_OTA_DATA` (`0x26`) | App → Target | 8-byte data chunk + offset |
| 4 | `LGT_CMD_END_OTA_REQ` (`0x28`) | App → Target | Signal transfer complete |
| 5 | `LGT_CMD_END_OTA_RSP` (`0x29`) | Target → App | Confirm and schedule reboot |

Each `TRANSFER_OTA_DATA` packet carries 8 bytes of firmware data (compared to 16 bytes in direct BLE OTA, because the 10-byte `par` field is shared with mesh routing metadata).

The OTA packets are injected into the mesh flood like any other command. Intermediate nodes relay them without inspection. Only the target node (`dst_adr` match) processes the OTA payload.

### 26.2 Flash Write Target

The target node writes received OTA data to `0x40000` (OTA staging area, 128 KB max) and validates the image before rebooting into the new firmware (see §17 for full flash layout).

---

## 27. Complete System Timing Reference (Mesh)

### 27.1 Mesh State Machine Cycle

```
Time 0ms       100ms      200ms      300ms      400ms      500ms ...
  │              │          │          │          │          │
  ├── Listen ────┼── Listen─┼── Listen─┼── Adv ───┼── Listen─┼── Listen ──►
  │  (ch 2)      │  (ch 12) │  (ch 23) │ (ch 37-39│  (ch 34) │  (ch 2) ...
  │              │          │          │  BLE adv) │          │
  │              │          │          │           │          │
  └──────────────────────── Online status (every 800ms) ─────►
```

### 27.2 Mesh TX Timing per Packet

```
Per channel (×4 channels):
  rf_set_tx_rx_off()          ~5 µs
  rf_set_ble_access_code()    ~2 µs
  rf_set_ble_crc_adv()        ~2 µs
  rf_set_ble_channel()        ~2 µs
  rf_start_srx2tx()           ~30 µs setup + packet air time
  Timer::after(600 µs)        600 µs guard
  ─────────────────────────────────────
  Per channel total:          ~640 µs

4 channels total:             ~2.56 ms

Retransmit delay (random):   0–8 ms

Per retransmit cycle:         ~2.56 + 0–8 ms
With BRIDGE_MAX_CNT=8:        ≤8 relay hops possible
```

### 27.3 Node Online Status Summary

| Parameter | Value | Derived From |
|-----------|-------|-------------|
| Status broadcast period | 800 ms | `100 ms × 8` |
| Node timeout | 3000 ms | `ONLINE_STATUS_TIMEOUT` |
| Max missed broadcasts before timeout | ~3–4 | `3000 / 800` |
| Re-pairing credential window | ~3 s | 6 × 500 ms frames |
| Re-pairing timeout | 10 s | `MESH_PAIR_TIMEOUT` |
| Post-credential apply delay | 1000 ms | `MESH_PAIR_CMD_INTERVAL × 2` |

---
