//! F-3 compatibility fixtures for firmware-owned flash, advertising and SRAM contracts.

mod compat_fixture {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tests/fixtures/rust_parser.rs"
    ));
}

use crate::light::effects::{adv_private_to_bytes, build_vendor_adv_data};
use compat_fixture::Parser;
use mesh_protocol::crc::crc16;
use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;

#[test]
fn gatt_table_metadata_and_rows_match_fixture() {
    let source = include_str!("../../../tests/fixtures/gatt_compat.txt");
    let mut p = Parser::parse(source).unwrap();

    assert_eq!(p.u8("gatt.table.count").unwrap(), 29);
    assert_eq!(p.u8("gatt.table.max_valid_handle").unwrap(), 28);
    assert_eq!(p.u8("gatt.att.mtu").unwrap(), 23);

    let mut span_sum = 0u32;
    for handle in 0..=28u8 {
        let prefix = format!("gatt.row.{handle:02}");
        let span = p.u8(&format!("{prefix}.span")).unwrap();
        let uuid_len = p.u8(&format!("{prefix}.uuid_len")).unwrap();
        let uuid = p.bytes(&format!("{prefix}.uuid")).unwrap();
        let default_len = p.u8(&format!("{prefix}.default_len")).unwrap();
        let max_len = p.u8(&format!("{prefix}.max_len")).unwrap();
        let behavior = p.str(&format!("{prefix}.behavior")).unwrap();

        if handle == 0 {
            assert_eq!(span, 28);
            assert_eq!(uuid_len, 0);
            assert!(uuid.is_empty());
            assert_eq!(behavior, "sentinel");
        } else {
            assert!(uuid_len == 2 || uuid_len == 16);
            assert_eq!(uuid.len(), uuid_len as usize);
            assert!(!behavior.is_empty());
        }

        if handle == 9 {
            assert_eq!(
                p.str(&format!("{prefix}.value")).unwrap(),
                "build_version_u32_le"
            );
        } else {
            let value = p.bytes(&format!("{prefix}.value")).unwrap();
            assert_eq!(value.len(), default_len as usize);
        }

        assert!(max_len >= default_len);
        span_sum += span as u32;
    }

    assert_eq!(span_sum, 28 + 6 + 9 + 13);
}

#[test]
fn ota_flash_regions_advertising_and_sram_match_fixture() {
    let source = include_str!("../../../tests/fixtures/ota_flash_adv_compat.txt");
    let mut p = Parser::parse(source).unwrap();

    let sector_size = p.u32("flash.sector_size").unwrap();
    let total_size = p.u32("flash.total_size").unwrap();
    let region_count = p.u8("flash.region_count").unwrap();
    let reset_last = p.u32("flash.reset_erase_last_addr").unwrap();
    assert_eq!(sector_size, 0x1000);
    assert_eq!(total_size, 0x80000);
    assert_eq!(region_count, 14);
    assert_eq!(reset_last, 0x7a000);

    let mut cursor = 0u32;
    let mut total_sectors = 0u32;
    for index in 0..14u8 {
        let prefix = format!("flash.region.{index:02}");
        let name = p.str(&format!("{prefix}.name")).unwrap().to_owned();
        let start = p.u32(&format!("{prefix}.start")).unwrap();
        let end = p.u32(&format!("{prefix}.end")).unwrap();
        let sectors = p.u32(&format!("{prefix}.sectors")).unwrap();
        let owner = p.str(&format!("{prefix}.owner")).unwrap().to_owned();
        let policy = p.str(&format!("{prefix}.reset_policy")).unwrap().to_owned();

        assert_eq!(start, cursor, "FlashMap gap or overlap at region {name}");
        assert_eq!(start % sector_size, 0);
        assert_eq!(end % sector_size, 0);
        assert_eq!((end - start) / sector_size, sectors);
        assert_eq!((end - start), sectors * sector_size);
        assert!(!owner.is_empty());
        assert!(!policy.is_empty());
        cursor = end;
        total_sectors += sectors;

        if index == 8 {
            assert_eq!(start, reset_last);
            assert_eq!(policy, "reset_erase_last");
        }
    }
    assert_eq!(cursor, total_size);
    assert_eq!(total_sectors, total_size / sector_size);

    assert_eq!(p.u32("ota.flash_adr_ota_ready_flag").unwrap(), 0x3f000);
    assert_eq!(p.u8("ota.flash_ota_ready_flag").unwrap(), 0xa5);
    assert_eq!(p.u32("ota.flash_adr_light_new_fw").unwrap(), 0x40000);
    assert_eq!(p.u8("ota.timeout_default_s").unwrap(), 30);
    assert_eq!(p.u8("ota.timeout_post_finish_s").unwrap(), 4);
    assert_eq!(p.u8("ota.packet_window.staging_slots").unwrap(), 16);
    assert_eq!(p.u8("ota.packet_window.packet_size").unwrap(), 48);
    assert_eq!(p.u16("ota.packet_window.window_bytes").unwrap(), 768);

    let mac = p.bytes("advertising.canonical.mac").unwrap();
    let mac_prefix = p.bytes("advertising.canonical.mac_prefix").unwrap();
    let dev_addr = p.bytes("advertising.canonical.dev_addr").unwrap();
    let build_version = p.bytes("advertising.canonical.build_version").unwrap();
    assert_eq!(mac, [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
    assert_eq!(mac_prefix, [0x11, 0x22, 0x33, 0x44]);
    assert_eq!(dev_addr, [0x1f, 0x00]);
    assert_eq!(build_version, [0x78, 0x56, 0x34, 0x12]);

    let mac_id = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    let (pri, rsp) = build_vendor_adv_data(mac_id, 0x0211, 0x001f);
    assert_eq!(
        adv_private_to_bytes(&pri),
        p.bytes("advertising.private.bytes").unwrap().as_slice()
    );

    let mut rsp_bytes = [0u8; 29];
    rsp_bytes[0..2].copy_from_slice(&rsp.manufacture_id.to_le_bytes());
    rsp_bytes[2..4].copy_from_slice(&rsp.mesh_product_uuid.to_le_bytes());
    rsp_bytes[4..8].copy_from_slice(&rsp.mac_address.to_le_bytes());
    rsp_bytes[8..10].copy_from_slice(&rsp.product_uuid.to_le_bytes());
    rsp_bytes[10] = rsp.status;
    rsp_bytes[11..13].copy_from_slice(&rsp.device_address.to_le_bytes());
    rsp_bytes[13..29].copy_from_slice(&rsp.rsv);
    assert_eq!(
        rsp_bytes.as_slice(),
        p.bytes("advertising.scan_rsp.private_bytes")
            .unwrap()
            .as_slice()
    );

    assert_eq!(
        p.bytes("advertising.init_flags").unwrap(),
        [0x02, 0x01, 0x05]
    );
    assert_eq!(
        p.bytes("advertising.out_of_mesh.primary_tlv")
            .unwrap()
            .len(),
        26
    );
    assert_eq!(
        p.bytes("advertising.in_mesh.primary_tlv").unwrap().len(),
        23
    );

    assert_eq!(p.u32("sram.ident.mac_addr").unwrap(), 0x808004);
    assert_eq!(p.u32("sram.ident.version_addr").unwrap(), 0x808008);
    assert_eq!(p.u32("sram.ident.crc_addr").unwrap(), 0x80800c);
    let golden = p.bytes("sram.ident.golden_crc_input").unwrap();
    assert_eq!(crc16(&golden), p.u16("sram.ident.golden_crc").unwrap());
}

#[test]
fn factory_reset_erases_exact_sectors_with_reset_count_last() {
    FakeFlash::reset();
    crate::factory_reset::reset_for_test();
    crate::factory_reset::erase_config_sectors();

    let expected = [
        0x77000, 0x78000, 0x79000, 0x7b000, 0x7c000, 0x7d000, 0x7e000, 0x7f000, 0x7a000,
    ];
    assert_eq!(FakeFlash::erase_history(), expected);

    for addr in expected {
        assert_eq!(FakeFlash::erase_count(addr), 1);
    }
    assert_eq!(FakeFlash::erase_count(0x76000), 0);
    assert_eq!(FakeFlash::erase_count(0x40000), 0);

    let reset_last = *FakeFlash::erase_history().last().unwrap();
    assert_eq!(reset_last, 0x7a000);
}

#[test]
fn domain_fixtures_have_closed_world_key_counts_and_no_implementation_symbols() {
    const FILES: [&str; 3] = [
        include_str!("../../../tests/fixtures/gatt_compat.txt"),
        include_str!("../../../tests/fixtures/mesh_uart_compat.txt"),
        include_str!("../../../tests/fixtures/ota_flash_adv_compat.txt"),
    ];
    const COUNTS: [usize; 3] = [269, 105, 161];

    for (source, expected_count) in FILES.iter().zip(COUNTS.iter()) {
        let mut p = Parser::parse(source).unwrap();
        assert_eq!(p.key_count(), *expected_count);
        p.consume_all();
        assert!(p.unconsumed_keys().is_empty());

        for forbidden in [
            "rf_link_",
            "crate::",
            "PacketAtt",
            "MeshNodeSt",
            "C-table",
            "callback",
        ] {
            assert!(
                !source.contains(forbidden),
                "fixture leaked implementation symbol `{forbidden}`"
            );
        }
    }
}
