//! Probe-to-host responses, tags `0x00..=0x05` plus `0x80 | code`.

use heapless::Vec;

use super::{expect_len, ProtoError, MAX_DATA_PAYLOAD};

/// Probe-to-host responses, tags `0x00..=0x05` plus `0x80 | code`.
// `Data` is a fixed-size inline buffer by contract: the crate is no-heap, so
// there is no `Box` to move the payload behind.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Response {
    /// Tag `0x00`: command accepted.
    Ack,
    /// Tag `0x01`: liveness reply.
    Pong {
        /// Firmware version.
        fw_ver: u16,
        /// Protocol version, see [`crate::PROTOCOL_VERSION`].
        proto_ver: u8,
    },
    /// Tag `0x02`: chip identification.
    ChipId {
        /// Chip revision.
        rev: u8,
        /// Chip ID.
        chip_id: u16,
    },
    /// Tag `0x03`: single byte read result.
    Byte(u8),
    /// Tag `0x04`: block read result, `0..=1024` bytes.
    Data(Vec<u8, MAX_DATA_PAYLOAD>),
    /// Tag `0x05`: SPI NOR JEDEC ID.
    JedecId([u8; 3]),
    /// Tag `0x80 | code`: failure, empty body.
    Error(ProtoError),
}

impl Response {
    /// Wire tag of this response.
    pub fn tag(&self) -> u8 {
        match self {
            Self::Ack => 0x00,
            Self::Pong { .. } => 0x01,
            Self::ChipId { .. } => 0x02,
            Self::Byte(_) => 0x03,
            Self::Data(_) => 0x04,
            Self::JedecId(_) => 0x05,
            Self::Error(err) => 0x80 | err.code(),
        }
    }

    /// Encoded body length of this response, tag and CRC excluded.
    fn body_len(&self) -> usize {
        match self {
            Self::Ack | Self::Error(_) => 0,
            Self::Pong { .. } => 3,
            Self::ChipId { .. } => 3,
            Self::Byte(_) => 1,
            Self::Data(data) => data.len(),
            Self::JedecId(_) => 3,
        }
    }

    /// Encodes the body (no tag, no CRC) into `out` and returns its length.
    ///
    /// `Data` length is bounded by its [`MAX_DATA_PAYLOAD`] capacity. Returns
    /// [`ProtoError::InvalidArg`] if `out` is too small.
    pub fn encode_body(&self, out: &mut [u8]) -> Result<usize, ProtoError> {
        let total = self.body_len();
        if out.len() < total {
            return Err(ProtoError::InvalidArg);
        }
        match self {
            Self::Ack | Self::Error(_) => {}
            Self::Pong { fw_ver, proto_ver } => {
                out[..2].copy_from_slice(&fw_ver.to_le_bytes());
                out[2] = *proto_ver;
            }
            Self::ChipId { rev, chip_id } => {
                out[0] = *rev;
                out[1..3].copy_from_slice(&chip_id.to_le_bytes());
            }
            Self::Byte(byte) => out[0] = *byte,
            Self::Data(data) => out[..total].copy_from_slice(data.as_slice()),
            Self::JedecId(id) => out[..3].copy_from_slice(id),
        }
        Ok(total)
    }

    /// Decodes the exact body bytes of a response carrying `tag`.
    ///
    /// Unknown tags, bodies of the wrong length, and unknown `Error` codes are
    /// [`ProtoError::InvalidFrame`].
    pub fn decode_body(tag: u8, body: &[u8]) -> Result<Self, ProtoError> {
        match tag {
            0x00 => {
                expect_len(body, 0)?;
                Ok(Self::Ack)
            }
            0x01 => {
                expect_len(body, 3)?;
                Ok(Self::Pong {
                    fw_ver: u16::from_le_bytes([body[0], body[1]]),
                    proto_ver: body[2],
                })
            }
            0x02 => {
                expect_len(body, 3)?;
                Ok(Self::ChipId {
                    rev: body[0],
                    chip_id: u16::from_le_bytes([body[1], body[2]]),
                })
            }
            0x03 => {
                expect_len(body, 1)?;
                Ok(Self::Byte(body[0]))
            }
            0x04 => {
                let mut data = Vec::new();
                data.extend_from_slice(body)
                    .map_err(|_| ProtoError::InvalidFrame)?;
                Ok(Self::Data(data))
            }
            0x05 => {
                expect_len(body, 3)?;
                Ok(Self::JedecId([body[0], body[1], body[2]]))
            }
            0x81..=0x87 => {
                expect_len(body, 0)?;
                Ok(Self::Error(ProtoError::from_code(tag & 0x7f)))
            }
            _ => Err(ProtoError::InvalidFrame),
        }
    }
}
