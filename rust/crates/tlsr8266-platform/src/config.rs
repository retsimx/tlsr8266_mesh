//! Minimal fixed-copy platform and product configuration.

/// Platform-owned timing and enablement configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlatformConfig {
    pub heartbeat_ticks: u32,
    pub mesh_rx_enabled: bool,
    pub uart_enabled: bool,
}

impl PlatformConfig {
    pub const fn new(heartbeat_ticks: u32, mesh_rx_enabled: bool, uart_enabled: bool) -> Self {
        Self {
            heartbeat_ticks,
            mesh_rx_enabled,
            uart_enabled,
        }
    }
}

impl Default for PlatformConfig {
    fn default() -> Self {
        Self::new(1_000, true, true)
    }
}

/// Product-owned radio identity configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProductConfig {
    pub device_address: u16,
    pub vendor_id: u16,
    pub product_id: u16,
}

impl ProductConfig {
    pub const fn new(device_address: u16, vendor_id: u16, product_id: u16) -> Self {
        Self {
            device_address,
            vendor_id,
            product_id,
        }
    }
}

impl Default for ProductConfig {
    fn default() -> Self {
        Self::new(0, 0, 0)
    }
}

/// Compact firmware identity stamped at build composition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FirmwareIdentity {
    pub vendor_id: u16,
    pub product_id: u16,
    pub firmware_version: u32,
}

impl FirmwareIdentity {
    pub const fn new(vendor_id: u16, product_id: u16, firmware_version: u32) -> Self {
        Self {
            vendor_id,
            product_id,
            firmware_version,
        }
    }
}

impl Default for FirmwareIdentity {
    fn default() -> Self {
        Self::new(0, 0, 0)
    }
}
