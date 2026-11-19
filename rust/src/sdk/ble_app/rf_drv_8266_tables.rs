use crate::sdk::common::compat::TBLCMDSET;

pub const TBL_AGC: [u32; 7] = [
    0x30333231, 0x182C3C38, 0xC0C1C, 0x0, 0x1B150F0A, 0x322E2721, 0x3E38,
];

pub const TBL_RF_INI: [TBLCMDSET; 61] = [
    TBLCMDSET {
        // reg_gpio_config_func4: GPIO function config
        adr: 0x5b4,
        dat: 0x02,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF modem/AGC baseband config between reg_rf_chn_rssi (0x458) and AGC table (0x480); undocumented
        adr: 0x474,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // 1.8V analog RF/bias init (TLSR8266 IP1); undocumented
        adr: 0x85,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // 1.8V analog RF/bias init (TLSR8266 IP1); undocumented
        adr: 0x80,
        dat: 0x61,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x06: "SAR ADC power off", but Telink calls it PLL BG power-down (ambiguous)
        adr: 0x06,
        dat: 0x00,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog calibration/enable strobe (bit3 toggled around the 0x81 xtal write); undocumented
        adr: 0x8f,
        dat: 0x30,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x81: crystal oscillator supply/bias/freq trim (xtal csel)
        adr: 0x81,
        dat: 0xd8,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog calibration/enable strobe (bit3 toggled around the 0x81 xtal write); undocumented
        adr: 0x8f,
        dat: 0x38,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x8b: ADC setting 2
        adr: 0x8b,
        dat: 0xe3,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // 1.8V analog RF/PA support adjacent to 0x8d output power; undocumented
        adr: 0x8e,
        dat: 0x6b,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x8d: RF output power
        adr: 0x8d,
        dat: 0x67,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // RF TX baseband config between reg_rf_tx_mode (0x400) and reg_rf_acc_len (0x405); undocumented
        adr: 0x402,
        dat: 0x26,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // Analog 0x9e: DC modulator (dc_mod)
        adr: 0x9e,
        dat: 0xad,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PA/PLL config (IP1); undocumented
        adr: 0xa0,
        dat: 0x28,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0xa2: PA gain control
        adr: 0xa2,
        dat: 0x2c,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PLL/modulator calibration step; undocumented
        adr: 0xa3,
        dat: 0x10,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PA/PLL config (IP1); undocumented
        adr: 0xac,
        dat: 0xa7,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PLL/modulator calibration step; undocumented
        adr: 0xaa,
        dat: 0x2e,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // reg_rf_rssi_offset: RSSI offset
        adr: 0x439,
        dat: 0x72,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_tx_mode (16-bit): RF TX mode
        adr: 0x400,
        dat: 0x0b,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_pilot: RX pilot
        adr: 0x42b,
        dat: 0xf3,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_hdx: RX header detect
        adr: 0x43b,
        dat: 0xfc,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_system_tick_ctrl: START system tick
        adr: 0x74f,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_tx_wail_settle_time byte 0 (32-bit): TX wait/settle timing
        adr: 0xf04,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // Upper byte of the reg_rf_tx_wail_settle_time word at 0xf04 (the wait/settle
        // field split is not documented)
        adr: 0xf06,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF link-layer timing between reg_rf_sys_timer_config (0xf0a) and reg_rf_mode (0xf16); undocumented
        adr: 0xf0c,
        dat: 0x50,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF link-layer timing between sys_timer_config (0xf0a) and rf_mode (0xf16); undocumented
        adr: 0xf10,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_tx_mode (16-bit): RF TX mode, second write
        adr: 0x400,
        dat: 0x0f,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_pilot: RX pilot, second write
        adr: 0x42b,
        dat: 0xf1,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX modem/threshold config adjacent to reg_rf_rx_sense_thr (0x422); undocumented
        adr: 0x420,
        dat: 0x20,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX modem/threshold config adjacent to reg_rf_rx_sense_thr (0x422); undocumented
        adr: 0x421,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_sense_thr: RX sensitivity threshold
        adr: 0x422,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX modem config between sense_thr (0x422) and rx_auto (0x426); undocumented
        adr: 0x424,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF modem/AGC baseband config between reg_rf_chn_rssi (0x458) and AGC table (0x480); undocumented
        adr: 0x464,
        dat: 0x07,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX DC I/Q offset between dci (0x4cb) and dcq (0x4cf); undocumented
        adr: 0x4cd,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // Analog 0x9e: DC modulator (dc_mod), mid-init update
        adr: 0x9e,
        dat: 0x56,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PLL/modulator calibration step, updated mid-init alongside 0x9e; undocumented
        adr: 0xa3,
        dat: 0xf0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog RF PLL/modulator calibration step, updated mid-init alongside 0x9e; undocumented
        adr: 0xaa,
        dat: 0x26,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // RF TX/sync config between reg_rf_tx_mode (0x400) and reg_rf_access_code (0x408); undocumented
        adr: 0x404,
        dat: 0xf5,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_access_code byte 0
        adr: 0x408,
        dat: 0x8e,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_access_code byte 1
        adr: 0x409,
        dat: 0x89,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_access_code byte 2
        adr: 0x40a,
        dat: 0xbe,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_access_code byte 3
        adr: 0x40b,
        dat: 0xd6,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // High byte of 16-bit reg_rf_tx_mode (0x400): 0x08 = BIT(11) = TX_1M2M_PN_EN (BLE 1M/2M whitening)
        adr: 0x401,
        dat: 0x08,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX demod/channel-filter config between rx_chn_dc (0x42c) and rx_pel (0x434); undocumented
        adr: 0x430,
        dat: 0x12,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_peak: RX peak detect
        adr: 0x43d,
        dat: 0x71,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF TX baseband config between reg_rf_tx_mode (0x400) and reg_rf_acc_len (0x405); undocumented
        adr: 0x402,
        dat: 0x24,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_tx_wail_settle_time byte 0 (32-bit): TX wait/settle timing, final value
        adr: 0xf04,
        dat: 0x68,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_chn_dc: RX channel DC offset
        adr: 0x42c,
        dat: 0x30,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX DC-offset control adjacent to reg_rf_rx_dci (0x4cb); undocumented
        adr: 0x4ca,
        dat: 0x88,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_dci: RX DC offset (I)
        adr: 0x4cb,
        dat: 0x04,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX demod/channel-filter config between rx_chn_dc (0x42c) and rx_pel (0x434); undocumented
        adr: 0x42d,
        dat: 0x33,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX demod/channel-filter config between rx_chn_dc (0x42c) and rx_pel (0x434); undocumented
        adr: 0x433,
        dat: 0x00,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // reg_rf_rx_pel: RX peak envelope
        adr: 0x434,
        dat: 0x01,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX config between reg_rf_rssi_offset (0x439) and reg_rf_rx_hdx (0x43b); undocumented
        adr: 0x43a,
        dat: 0x77,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX peak/gain config adjacent to reg_rf_rx_peak (0x43d); undocumented
        adr: 0x43e,
        dat: 0xc9,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // RF RX DC I/Q offset between dci (0x4cb) and dcq (0x4cf); final value; undocumented
        adr: 0x4cd,
        dat: 0x06,
        cmd: 0xc3,
    },
    // Enable these for BLE to work with 16mhz xtal
    TBLCMDSET {
        // reg_pll_ctrl_a = CAL_DONE_EN | LOCK_EN (loaded only for the xtal-16mhz build)
        adr: 0x4eb,
        dat: 0x60,
        cmd: 0xc3,
    },
    TBLCMDSET {
        // Analog 0x99: gauss filter select, 16M
        adr: 0x99,
        dat: 0x31,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x82: enable RX ADC clock
        adr: 0x82,
        dat: 0x34,
        cmd: 0xc8,
    },
    TBLCMDSET {
        // Analog 0x9e: DC modulator (dc_mod), 500K
        adr: 0x9e,
        dat: 0x41,
        cmd: 0xc8,
    },
];

/**
 * RF power control table structure
 * Contains analog register values to configure RF power settings
 * Each entry defines values for four different analog registers that together control transmission power
 */
#[repr(C, packed)]
pub struct TblRfPowerT {
    pub a: u8, // PA gain control (analog 0xa2 / REGA_RF_PA_GAIN)
    pub b: u8, // PA bias control (analog 0x04 / REGA_RF_PA_BIAS)
    pub c: u8, // RF output match (analog 0xa7 / REGA_RF_OUTPUT_MATCH)
    pub d: u8, // RF output power (analog 0x8d / REGA_RF_OUTPUT_POWER)
}

// Table of predefined RF power settings in descending order from highest to lowest power
// Each entry corresponds to a specific power level the transmitter can operate at
pub const TBL_RF_POWER: [TblRfPowerT; 12] = [
    TblRfPowerT {
        a: 0x25,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x0a,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x74,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc2,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x06,
        b: 0x64,
        c: 0xc1,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x05,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x03,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x02,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x01,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x7c,
        c: 0x67,
        d: 0x67,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0x43,
        d: 0x61,
    },
    TblRfPowerT {
        a: 0x00,
        b: 0x64,
        c: 0xcb,
        d: 0x61,
    },
];
