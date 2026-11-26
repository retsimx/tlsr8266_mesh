//! `crypto` namespace: the AES-128 block table, the authenticated-packet
//! vectors and the command-encryption vectors, all driven through the real
//! HAL crypto path (`SoftwareAes` on host, hardware AES on device).

use super::{exact, Parser};
use tlsr8266_hal::mcu::crypto::fake_crypto;
use tlsr8266_hal::mcu::crypto::{
    aes_att_decryption, aes_att_decryption_packet, aes_att_encryption, aes_att_encryption_packet,
    MicMismatch,
};

pub(super) struct AesBlock {
    key: [u8; 16],
    plain: [u8; 16],
    cipher: [u8; 16],
}

/// `aes.block.N` key/plain/cipher triples, read without asserting.
pub(super) fn read_aes_blocks(p: &mut Parser) -> Vec<AesBlock> {
    let mut blocks = Vec::new();
    let mut index = 1;

    while p.contains(&format!("aes.block.{index}.key")) {
        let key = exact(
            &format!("aes.block.{index}.key"),
            p.bytes(&format!("aes.block.{index}.key")),
        );
        let plain = exact(
            &format!("aes.block.{index}.plain"),
            p.bytes(&format!("aes.block.{index}.plain")),
        );
        let cipher = exact(
            &format!("aes.block.{index}.cipher"),
            p.bytes(&format!("aes.block.{index}.cipher")),
        );

        blocks.push(AesBlock { key, plain, cipher });
        index += 1;
    }

    blocks
}

pub(super) struct AesPacket {
    key: [u8; 16],
    ivm: [u8; 8],
    plain: Vec<u8>,
    cipher: Vec<u8>,
    mic: [u8; 2],
}

/// `aes.packet.N` authenticated-packet vectors, read without asserting.
pub(super) fn read_aes_packets(p: &mut Parser) -> Vec<AesPacket> {
    let mut packets = Vec::new();
    let mut index = 1;

    while p.contains(&format!("aes.packet.{index}.key")) {
        let key = exact(
            &format!("aes.packet.{index}.key"),
            p.bytes(&format!("aes.packet.{index}.key")),
        );
        let ivm = exact(
            &format!("aes.packet.{index}.ivm"),
            p.bytes(&format!("aes.packet.{index}.ivm")),
        );
        let plain = p.bytes(&format!("aes.packet.{index}.plain"));
        let cipher = p.bytes(&format!("aes.packet.{index}.cipher"));
        let mic = exact(
            &format!("aes.packet.{index}.mic"),
            p.bytes(&format!("aes.packet.{index}.mic")),
        );

        packets.push(AesPacket {
            key,
            ivm,
            plain,
            cipher,
            mic,
        });
        index += 1;
    }

    packets
}

pub(super) struct CmdAction {
    session_key: [u8; 16],
    nonce: [u8; 8],
    plain_cmd: Vec<u8>,
    encrypted_cmd: Vec<u8>,
}

/// `cmd_action.N` command-encryption vectors, read without asserting.
///
/// Wire form is `plain_cmd = sno(3) | mac_slot(2) | payload(15)`; the fixture's
/// `encrypted_cmd` is the same 20 bytes after `CommandAction.generate_data`
/// wrote the 2-byte MAC into 3..5 and CTR-encrypted the 15-byte payload.
pub(super) fn read_cmd_actions(p: &mut Parser) -> Vec<CmdAction> {
    let mut actions = Vec::new();
    let mut index = 1;

    while p.contains(&format!("cmd_action.{index}.session_key")) {
        let session_key = exact(
            &format!("cmd_action.{index}.session_key"),
            p.bytes(&format!("cmd_action.{index}.session_key")),
        );
        let nonce = exact(
            &format!("cmd_action.{index}.nonce"),
            p.bytes(&format!("cmd_action.{index}.nonce")),
        );
        let plain_cmd = p.bytes(&format!("cmd_action.{index}.plain_cmd"));
        let encrypted_cmd = p.bytes(&format!("cmd_action.{index}.encrypted_cmd"));

        actions.push(CmdAction {
            session_key,
            nonce,
            plain_cmd,
            encrypted_cmd,
        });
        index += 1;
    }

    actions
}

/// Proves the AES block table both ways through the **real** crypto path.
///
/// `aes_att_encryption` / `aes_att_decryption` are called unstubbed, so this is
/// a value test of the whole primitive: on host the HAL's `SoftwareAes` runs a
/// real FIPS-197 AES-128 with the TLSR8266 byte-reversal parity, and on device
/// the hardware AES engine runs. Any byte-order or parity mistake anywhere in
/// that chain (the outer `result.reverse()`, the reversed key/data the chip
/// expects, the S-box, the round schedule) makes one of the 8 committed
/// `(key, plain) -> cipher` triples fail in one of the two directions.
#[test]
fn fixture_aes_blocks() {
    fake_crypto::reset();
    let blocks = read_aes_blocks(&mut Parser::new());
    assert!(
        !blocks.is_empty(),
        "the fixture has no `aes.block.*` vectors"
    );

    for block in &blocks {
        assert_eq!(
            aes_att_encryption(&block.key, &block.plain),
            block.cipher,
            "aes_att_encryption mismatch for fixture block {:02x?}",
            block.plain
        );
        assert_eq!(
            aes_att_decryption(&block.key, &block.cipher),
            block.plain,
            "aes_att_decryption mismatch for fixture block {:02x?}",
            block.cipher
        );
    }
}

/// Value parity for the authenticated-packet path: the committed
/// `aes.packet.*` vectors drive the real CTR + CBC-MAC code end to end, and the
/// decryption direction must both recover the plaintext and reject a corrupted
/// MIC.
#[test]
fn fixture_aes_packets() {
    fake_crypto::reset();
    let mut p = Parser::new();
    let packets = read_aes_packets(&mut p);
    assert!(
        !packets.is_empty(),
        "the fixture has no `aes.packet.*` vectors"
    );

    for packet in &packets {
        let mut ciphertext = packet.plain.clone();
        let mut mic = [0u8; 2];
        aes_att_encryption_packet(&packet.key, &packet.ivm, &mut mic, &mut ciphertext);
        assert_eq!(ciphertext, packet.cipher, "CTR ciphertext mismatch");
        assert_eq!(mic, packet.mic, "CBC-MAC MIC mismatch");

        let mut recovered = packet.cipher.clone();
        assert_eq!(
            aes_att_decryption_packet(&packet.key, &packet.ivm, &packet.mic, &mut recovered),
            Ok(()),
            "decryption rejected a valid MIC"
        );
        assert_eq!(recovered, packet.plain, "decrypted plaintext mismatch");

        let mut corrupted_mic = packet.mic;
        corrupted_mic[0] ^= 0xff;
        let mut ignored = packet.cipher.clone();
        assert_eq!(
            aes_att_decryption_packet(&packet.key, &packet.ivm, &corrupted_mic, &mut ignored),
            Err(MicMismatch),
            "decryption accepted a corrupted MIC"
        );
    }
}

/// Value parity for `CommandAction.generate_data`: the committed
/// `cmd_action.*` vectors drive the real CTR + CBC-MAC code over the 15-byte
/// command payload, and the decryption direction must recover the original
/// plaintext. `SoftwareAes` is the active transport after `fake_crypto::reset`.
#[test]
fn fixture_cmd_actions() {
    fake_crypto::reset();
    let mut p = Parser::new();
    let actions = read_cmd_actions(&mut p);
    assert!(
        !actions.is_empty(),
        "the fixture has no `cmd_action.*` vectors"
    );

    for action in &actions {
        assert_eq!(action.plain_cmd.len(), 20, "`plain_cmd` must be 20 bytes");
        assert_eq!(
            action.encrypted_cmd.len(),
            20,
            "`encrypted_cmd` must be 20 bytes"
        );

        let mut command = action.plain_cmd.clone();
        let (head, payload) = command.split_at_mut(5);
        aes_att_encryption_packet(&action.session_key, &action.nonce, &mut head[3..5], payload);
        assert_eq!(
            command, action.encrypted_cmd,
            "command encryption mismatch for fixture vector {:02x?}",
            action.plain_cmd
        );

        let (head, payload) = command.split_at_mut(5);
        aes_att_decryption_packet(&action.session_key, &action.nonce, &head[3..5], payload)
            .expect("the encrypted cmd_action vector must decrypt");
        assert_eq!(
            &payload[..],
            &action.plain_cmd[5..],
            "decrypted payload mismatch for fixture vector {:02x?}",
            action.plain_cmd
        );
    }
}
