//! The OTA subsystem: every piece of OTA state, policy and logic in the
//! platform.
//!
//! # Layout
//!
//! * [`fsm`] — pure chunk decision table (`u16` wrap and saturating size bound);
//! * [`storage`] — staging-area flash reads/writes, verification, dirty-area
//!   recovery and ready-flag arming;
//! * [`engine`] — the sole owner of runtime state, timeouts, the completion
//!   funnel and the connection-slot servicing;
//! * [`gatt`], [`mesh`], [`uart`] — transport adapters that decode their own
//!   framing into an [`OtaChunk`] and call the engine;
//! * [`boot`] — the RAM-resident boot-time image copier.
//!
//! # Transport neutrality
//!
//! Raw `mesh_protocol::packet_types::Packet` never crosses this module boundary:
//! the GATT adapter stages a typed [`window::StagedOtaSlot`] into the shared
//! packet window, and every transport hands the engine an owned-data-free
//! [`OtaChunk`].
//!
//! # Legacy behaviour
//!
//! This is a port of the deployed firmware OTA implementation, and every
//! documented asymmetry is preserved deliberately: the cursor resets on a gap,
//! on index 0 over a non-empty cursor, on a bad final header, on a malformed
//! frame and on a bad CRC, but **not** on a failed flash verification or on the
//! size-bound overrun. See [`engine::process_chunk`].

pub mod boot;
pub mod engine;
pub mod fsm;
pub mod gatt;
pub mod mesh;
pub mod storage;
pub mod uart;

#[cfg(test)]
pub(crate) mod test_product;

pub use boot::{ota_boot_copy, OtaBootConfig};
pub use engine::{
    finish_and_reboot, is_gatt_ota_in_progress, is_mesh_ota_blocked, is_timeout_expired,
    on_1s_tick, on_link_disconnected, process_chunk, ChunkOutcome, OtaChunk, OtaFrameError,
    OtaTransport,
};
pub use mesh::{ota_data as mesh_ota_data, start_ota as mesh_start_ota};
pub use storage::flash_layout;

use tlsr8266_radio::ConnectionPlan;

use crate::api::Product;

/// Boot-time recovery: erase a staging area left dirty by a reset mid-transfer.
///
/// A half-written image must never be copied out, so the area is cleaned before
/// the ready flag is consulted.
pub fn check_ota_area_startup() {
    if let Ok(false) | Err(()) = storage::is_ota_area_valid() {
        storage::erase_staging_area();
    }
}

/// Service one BLE connection slot for a GATT OTA transfer.
///
/// Returns `Continue` when no transfer is running, `SkipToNextEvent` while one
/// is (the radio must not run bridge/mesh work for this slot) and `Disconnect`
/// once the reboot was latched.
pub fn poll_connection_slot<P: Product>(current_tick: u32) -> ConnectionPlan {
    engine::poll_connection_slot::<P>(current_tick)
}

#[cfg(test)]
mod tests {
    use mesh_protocol::mesh_types::OtaState;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;

    use super::test_product::TestProduct;
    use super::{check_ota_area_startup, engine, gatt, mesh, storage};
    use crate::config::FirmwareIdentity;

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../tests/fixtures/rust_parser.rs"
    ));

    /// Staging base of the deployed layout.
    const STAGING: u32 = 0x40000;
    /// The golden 64-byte image both transports must reproduce.
    const IMAGE_BYTES: usize = 64;

    fn reset() {
        FakeFlash::reset();
        fake_clock::reset();
        engine::reset_for_test();
    }

    fn vectors() -> Parser {
        Parser::parse(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/wire_vectors.txt"
        )))
        .expect("wire vectors parse")
    }

    fn staged_image() -> [u8; IMAGE_BYTES] {
        let mut image = [0u8; IMAGE_BYTES];
        FakeFlash::read_slice(STAGING, &mut image);
        image
    }

    /// A dirty staging area is erased before the ready flag is consulted; a
    /// clean one is left alone.
    #[test]
    fn startup_erases_a_dirty_staging_area_but_not_a_clean_one() {
        FakeFlash::reset();

        check_ota_area_startup();
        assert_eq!(
            FakeFlash::erase_count(STAGING),
            0,
            "a clean staging area must not be erased"
        );
        assert_eq!(storage::is_ota_area_valid(), Ok(true));

        FakeFlash::write_slice(STAGING, &[0x00]);
        assert_eq!(storage::is_ota_area_valid(), Ok(false));

        check_ota_area_startup();
        assert_eq!(
            FakeFlash::erase_count(STAGING),
            1,
            "a half-written image is discarded at boot"
        );
        assert_eq!(storage::is_ota_area_valid(), Ok(true));
    }

    /// The same golden image, staged through two transports with different
    /// chunk widths, must finish `Ok` with byte-identical flash contents.
    #[test]
    fn gatt_and_mesh_stage_the_same_golden_image() {
        let mut fixture = vectors();
        let expected = fixture.bytes("ota.blob").expect("golden blob");
        assert_eq!(expected.len(), IMAGE_BYTES);

        // GATT: four 16-byte chunks plus a final header at index 4.
        reset();
        tlsr8266_radio::set_paired_login(true);
        for index in 0..4u16 {
            let frame = fixture
                .bytes(&format!("ota.ble.packet.{index}"))
                .expect("golden BLE packet");
            assert_eq!(frame.len(), 20);
            gatt::on_write::<TestProduct>(23, &frame);
        }
        let mut gatt_header = [0u8; 4];
        gatt_header[0..2].copy_from_slice(&4u16.to_le_bytes());
        let crc = mesh_protocol::crc::crc16(&gatt_header[0..2]).to_le_bytes();
        gatt_header[2..4].copy_from_slice(&crc);
        gatt::on_write::<TestProduct>(7, &gatt_header);
        gatt::drain_staged_slots::<TestProduct>();

        assert_eq!(engine::finish_status(), OtaState::Ok);
        let gatt_image = staged_image();
        tlsr8266_radio::set_paired_login(false);

        // Mesh: the same blob as eight 8-byte chunks plus a header at index 8.
        reset();
        let identity = FirmwareIdentity::new(0x1234, 0x0001, 0x1122_3344);
        assert!(mesh::start_ota::<TestProduct>(identity, || true).1);
        for index in 0..8u16 {
            let block = fixture
                .bytes(&format!("ota.uart.packet.{index}"))
                .expect("golden mesh block");
            assert_eq!(block.len(), 10);
            assert_eq!(mesh::ota_data::<TestProduct>(&block, false), index);
        }
        let mut mesh_header = [0u8; 10];
        mesh_header[0..2].copy_from_slice(&8u16.to_le_bytes());
        assert_eq!(mesh::ota_data::<TestProduct>(&mesh_header, true), 8);

        assert_eq!(engine::finish_status(), OtaState::Ok);
        let mesh_image = staged_image();

        assert_eq!(gatt_image[..], expected[..]);
        assert_eq!(
            mesh_image, gatt_image,
            "both transports must stage byte-identical images"
        );
    }
}
