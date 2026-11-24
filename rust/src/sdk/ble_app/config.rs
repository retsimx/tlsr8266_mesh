//! Board and product configuration consumed by the BLE application layer.
//!
//! `RadioConfig` is the single configuration seam the SDK uses. It removes the
//! upward dependency on the crate-root configuration and version modules, which
//! is what lets the radio layer be extracted into its own crate (C-10).
//!
//! Production builds the struct once in `app.rs` from the crate-root `config`
//! and `version` modules and installs it with [`set_radio_config`] before the
//! radio is started. The literals in [`DEFAULT`] mirror today's factory values,
//! so host unit tests need no set-up and a read that happens before injection
//! still returns exactly the values the firmware reported before this seam
//! existed.

use tlsr8266_hal::common::sync::CriticalSectionMutex;

use crate::sdk::ble_app::ble_ll_pair::seed_pair_config;
use crate::sdk::ble_app::rf_drv_8266::apply_vendor_id;

/// Board and product configuration read by the BLE application layer.
///
/// String fields are copied into fixed-size pairing/vendor buffers by
/// [`set_radio_config`], which zero-pads and clamps them, so no field may be
/// assumed to fit without clamping.
#[derive(Copy, Clone)]
pub struct RadioConfig {
    /// Vendor id written into both private advertising payloads.
    pub vendor_id: u16,
    /// Firmware version word reported at `0x808008` and over GATT.
    pub build_version: u32,
    /// GATT device name; clamped to 19 bytes by the GATT profile.
    pub device_name: &'static [u8],
    /// Mesh network name seeded into the pairing config; clamped to 16 bytes.
    pub default_mesh_name: &'static str,
    /// Mesh network password seeded into the pairing config; clamped to 16 bytes.
    pub default_mesh_pwd: &'static str,
    /// Password-encoding session key; clamped to 16 bytes.
    pub default_mesh_pwd_encode_sk: &'static str,
    /// Mesh long-term key seeded into the pairing config.
    pub default_mesh_ltk: [u8; 16],
    /// Network name advertised while the device is out of a mesh.
    pub out_of_mesh_name: &'static str,
    /// Magic byte marking a valid pairing record in flash.
    pub pair_valid_flag: u8,
    /// Flash sector holding the MAC address.
    pub flash_adr_mac: u32,
    /// Flash sector holding the pairing record.
    pub flash_adr_pairing: u32,
    /// Flash sector holding the device/group addresses.
    pub flash_adr_dev_grp_adr: u32,
}

/// Factory defaults, byte-identical to the crate-root configuration constants.
///
/// These literals deliberately duplicate the crate-root `config` values: that
/// duplication is what removes the last upward import from `sdk/`, and it keeps
/// host tests boilerplate-free. The crate-root `config` module plus the
/// `version` module remain production's source of truth, injected by `app.rs`.
pub(crate) const DEFAULT: RadioConfig = RadioConfig {
    vendor_id: 0x0211,
    build_version: 0,
    device_name: b"Telink tLight",
    default_mesh_name: "out_of_mesh",
    default_mesh_pwd: "123",
    default_mesh_pwd_encode_sk: "0123456789ABCDEF",
    default_mesh_ltk: [
        0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde,
        0xdf,
    ],
    out_of_mesh_name: "out_of_mesh",
    pair_valid_flag: 0xFA,
    flash_adr_mac: 0x76000,
    flash_adr_pairing: 0x77000,
    flash_adr_dev_grp_adr: 0x79000,
};

static RADIO_CONFIG: CriticalSectionMutex<RadioConfig> = CriticalSectionMutex::new(DEFAULT);

/// Copy the active configuration out of the lock and release it immediately.
///
/// `RadioConfig` is `Copy`, so callers get a self-contained snapshot; no
/// interrupt-masking lock is ever held across I/O.
pub(crate) fn radio_config() -> RadioConfig {
    *RADIO_CONFIG.lock()
}

/// Install `cfg` as the active configuration.
///
/// Also pushes the parts of the configuration that live in module-owned state:
/// the vendor id into both private advertising payloads, and the mesh
/// credentials into the pairing configuration buffers. Call this before the
/// radio starts advertising.
pub fn set_radio_config(cfg: RadioConfig) {
    *RADIO_CONFIG.lock() = cfg;
    apply_vendor_id(cfg.vendor_id);
    seed_pair_config(&cfg);
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::sdk::ble_app::ble_ll_pair::{
        PAIR_CONFIG_MESH_LTK, PAIR_CONFIG_MESH_NAME, PAIR_CONFIG_MESH_PWD,
        PAIR_CONFIG_PWD_ENCODE_SK,
    };
    use crate::sdk::ble_app::rf_drv_8266::{ADV_PRI_DATA, ADV_RSP_PRI_DATA};

    /// Restore the factory configuration after a test that injected its own.
    fn restore_default() {
        set_radio_config(DEFAULT);
    }

    #[test]
    fn default_matches_factory_configuration() {
        let cfg = radio_config();

        assert_eq!(cfg.vendor_id, 0x0211);
        assert_eq!(cfg.build_version, 0);
        assert_eq!(cfg.device_name, b"Telink tLight");
        assert_eq!(cfg.default_mesh_name, "out_of_mesh");
        assert_eq!(cfg.default_mesh_pwd, "123");
        assert_eq!(cfg.default_mesh_pwd_encode_sk, "0123456789ABCDEF");
        assert_eq!(
            cfg.default_mesh_ltk,
            [
                0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd,
                0xde, 0xdf,
            ]
        );
        assert_eq!(cfg.out_of_mesh_name, "out_of_mesh");
        assert_eq!(cfg.pair_valid_flag, 0xFA);
        assert_eq!(cfg.flash_adr_mac, 0x76000);
        assert_eq!(cfg.flash_adr_pairing, 0x77000);
        assert_eq!(cfg.flash_adr_dev_grp_adr, 0x79000);
    }

    #[test]
    fn injected_config_round_trips() {
        let mut cfg = radio_config();
        cfg.build_version = 0x1234_5678;
        set_radio_config(cfg);

        let read_back = radio_config();
        restore_default();

        assert_eq!(read_back.build_version, 0x1234_5678);
    }

    #[test]
    fn set_radio_config_propagates_vendor_id_to_both_advertising_payloads() {
        let mut cfg = radio_config();
        cfg.vendor_id = 0xABCD;
        set_radio_config(cfg);

        let (pri_id, pri_uuid) = {
            let adv = ADV_PRI_DATA.lock();
            (adv.manufacture_id, adv.mesh_product_uuid)
        };
        let (rsp_id, rsp_uuid) = {
            let adv = ADV_RSP_PRI_DATA.lock();
            (adv.manufacture_id, adv.mesh_product_uuid)
        };
        restore_default();

        assert_eq!(pri_id, 0xABCD, "ADV_PRI_DATA must carry the vendor id");
        assert_eq!(
            pri_uuid, 0xABCD,
            "ADV_PRI_DATA uuid must follow the vendor id"
        );
        assert_eq!(rsp_id, 0xABCD, "ADV_RSP_PRI_DATA must carry the vendor id");
        assert_eq!(
            rsp_uuid, 0xABCD,
            "ADV_RSP_PRI_DATA uuid must follow the vendor id"
        );

        // Restoring the default config must put the factory vendor id back.
        let factory_vendor_id = radio_config().vendor_id;
        let pri_id = ADV_PRI_DATA.lock().manufacture_id;
        let rsp_id = ADV_RSP_PRI_DATA.lock().manufacture_id;
        assert_eq!(factory_vendor_id, 0x0211);
        assert_eq!(pri_id, 0x0211);
        assert_eq!(rsp_id, 0x0211);
    }

    #[test]
    fn set_radio_config_seeds_all_four_pair_credential_buffers() {
        let mut cfg = radio_config();
        cfg.default_mesh_name = "mesh";
        cfg.default_mesh_pwd = "pw";
        cfg.default_mesh_pwd_encode_sk = "0123456789ABCDEFGHIJ";
        cfg.default_mesh_ltk = [0x5A; 16];
        set_radio_config(cfg);

        let name = *PAIR_CONFIG_MESH_NAME.lock();
        let pwd = *PAIR_CONFIG_MESH_PWD.lock();
        let sk = *PAIR_CONFIG_PWD_ENCODE_SK.lock();
        let ltk = *PAIR_CONFIG_MESH_LTK.lock();
        restore_default();

        let mut expected_name = [0u8; 16];
        expected_name[..4].copy_from_slice(b"mesh");
        let mut expected_pwd = [0u8; 16];
        expected_pwd[..2].copy_from_slice(b"pw");
        // The key is clamped to the 16-byte buffer, then zero-padded.
        let expected_sk = *b"0123456789ABCDEF";

        assert_eq!(name, expected_name, "mesh name must be zero padded");
        assert_eq!(pwd, expected_pwd, "mesh password must be zero padded");
        assert_eq!(sk, expected_sk, "pwd encode sk must be clamped and padded");
        assert_eq!(ltk, [0x5A; 16], "mesh ltk must be copied verbatim");
    }

    #[test]
    fn injected_build_version_is_returned() {
        let mut cfg = radio_config();
        cfg.build_version = 0xDEAD_BEEF;
        set_radio_config(cfg);

        let build_version = radio_config().build_version;
        restore_default();

        assert_eq!(build_version, 0xDEAD_BEEF);
    }
}
