use core::marker::PhantomData;

use crate::ble::connection::GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP;
use crate::gatt::service_table::{
    AttError, AttrRole, AttrUuid, GattBackend, GattReadKind, GattReadOutcome, GattReadRequest,
    GattWriteKind, GattWriteOutcome, GattWriteRequest, LegacyGattBackend, ProfileView,
    LEGACY_PROFILE,
};
use crate::rf::drv_8266::PKT_ERR_RSP;
use mesh_protocol::packet_types::{
    Packet, PacketAttMtu, PacketAttReadRsp, PacketAttWriteRsp, PacketL2capHead,
};
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_pac::read_reg_system_tick;

/// Legacy numeric opcode names kept for compatibility with existing host fixtures.
/// New packet dispatch is typed and matches the complete opcode byte directly.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GattOp {
    AttOpErrorRsp = 0x01,
    AttOpExchangeMtuReq = 0x02,
    AttOpExchangeMtuRsp = 0x03,
    AttOpFindInfoReq = 0x04,
    AttOpFindInfoRsp = 0x05,
    AttOpFindByTypeValueReq = 0x06,
    AttOpFindByTypeValueRsp = 0x07,
    AttOpReadByTypeReq = 0x08,
    AttOpReadByTypeRsp = 0x09,
    AttOpReadReq = 0x0a,
    AttOpReadRsp = 0x0b,
    AttOpReadBlobReq = 0x0c,
    AttOpReadBlobRsp = 0x0d,
    AttOpReadMultiReq = 0x0e,
    AttOpReadMultiRsp = 0x0f,
    AttOpReadByGroupTypeReq = 0x10,
    AttOpReadByGroupTypeRsp = 0x11,
    AttOpWriteReq = 0x12,
    AttOpWriteRsp = 0x13,
    AttOpPrepareWriteReq = 0x16,
    AttOpPrepareWriteRsp = 0x17,
    AttOpExecuteWriteReq = 0x18,
    AttOpExecuteWriteRsp = 0x19,
    AttOpHandleValueNoti = 0x1b,
    AttOpHandleValueInd = 0x1d,
    AttOpHandleValueCfm = 0x1e,
    AttOpWriteCmd = 0x52,
}

impl TryFrom<u8> for GattOp {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Self::AttOpErrorRsp),
            0x02 => Ok(Self::AttOpExchangeMtuReq),
            0x03 => Ok(Self::AttOpExchangeMtuRsp),
            0x04 => Ok(Self::AttOpFindInfoReq),
            0x05 => Ok(Self::AttOpFindInfoRsp),
            0x06 => Ok(Self::AttOpFindByTypeValueReq),
            0x07 => Ok(Self::AttOpFindByTypeValueRsp),
            0x08 => Ok(Self::AttOpReadByTypeReq),
            0x09 => Ok(Self::AttOpReadByTypeRsp),
            0x0a => Ok(Self::AttOpReadReq),
            0x0b => Ok(Self::AttOpReadRsp),
            0x0c => Ok(Self::AttOpReadBlobReq),
            0x0d => Ok(Self::AttOpReadBlobRsp),
            0x0e => Ok(Self::AttOpReadMultiReq),
            0x0f => Ok(Self::AttOpReadMultiRsp),
            0x10 => Ok(Self::AttOpReadByGroupTypeReq),
            0x11 => Ok(Self::AttOpReadByGroupTypeRsp),
            0x12 => Ok(Self::AttOpWriteReq),
            0x13 => Ok(Self::AttOpWriteRsp),
            0x16 => Ok(Self::AttOpPrepareWriteReq),
            0x17 => Ok(Self::AttOpPrepareWriteRsp),
            0x18 => Ok(Self::AttOpExecuteWriteReq),
            0x19 => Ok(Self::AttOpExecuteWriteRsp),
            0x1b => Ok(Self::AttOpHandleValueNoti),
            0x1d => Ok(Self::AttOpHandleValueInd),
            0x1e => Ok(Self::AttOpHandleValueCfm),
            0x52 => Ok(Self::AttOpWriteCmd),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AttRequest<'a> {
    ExchangeMtu,
    FindInfo {
        start: u16,
        end: u16,
    },
    FindByTypeValue {
        start: u16,
        end: u16,
        uuid: [u8; 2],
        target_value: u16,
    },
    ReadByType {
        start: u16,
        end: u16,
        uuid: &'a [u8],
        uuid128: bool,
    },
    Read {
        handle: u16,
    },
    ReadByGroupType {
        start: u16,
        end: u16,
        uuid: [u8; 2],
    },
    Write {
        handle: u16,
        kind: GattWriteKind,
        l2cap_len: u16,
        payload: &'a [u8],
    },
}

impl<'a> AttRequest<'a> {
    pub fn parse(packet: &'a Packet) -> Option<Self> {
        if packet.head().chan_id != 4 {
            return None;
        }

        let att = packet.att_val();
        let l2cap_len = packet.head().l2cap_len;

        match att.opcode {
            0x02 if l2cap_len >= 3 => Some(Self::ExchangeMtu),
            0x04 if l2cap_len >= 5 => {
                let start = u16::from_le_bytes([att.handle, att.handle1]);
                let end = u16::from_le_bytes([att.value[0], att.value[1]]);
                if start == 0 || start > end {
                    return Some(Self::FindInfo { start, end });
                }
                Some(Self::FindInfo { start, end })
            }
            0x06 if l2cap_len >= 9 => {
                let start = u16::from_le_bytes([att.handle, att.handle1]);
                let end = u16::from_le_bytes([att.value[0], att.value[1]]);
                let uuid = [att.value[2], att.value[3]];
                let target_value = u16::from_le_bytes([att.value[4], att.value[5]]);
                Some(Self::FindByTypeValue {
                    start,
                    end,
                    uuid,
                    target_value,
                })
            }
            0x08 if l2cap_len >= 7 => {
                let start = u16::from_le_bytes([att.handle, att.handle1]);
                let end = u16::from_le_bytes([att.value[0], att.value[1]]);
                let uuid128 = l2cap_len >= 21;
                let uuid_len = if uuid128 { 16 } else { 2 };
                let uuid = att.value.get(2..2 + uuid_len)?;
                Some(Self::ReadByType {
                    start,
                    end,
                    uuid,
                    uuid128,
                })
            }
            0x0a if l2cap_len >= 3 => {
                let handle = u16::from_le_bytes([att.handle, att.handle1]);
                if handle == 0 {
                    return None;
                }
                Some(Self::Read { handle })
            }
            0x10 if l2cap_len >= 7 => {
                let start = u16::from_le_bytes([att.handle, att.handle1]);
                let end = u16::from_le_bytes([att.value[0], att.value[1]]);
                let uuid = [att.value[2], att.value[3]];
                Some(Self::ReadByGroupType { start, end, uuid })
            }
            0x12 | 0x52 => {
                let handle = u16::from_le_bytes([att.handle, att.handle1]);
                let kind = if att.opcode == 0x12 {
                    GattWriteKind::Request
                } else {
                    GattWriteKind::Command
                };
                if l2cap_len < 3 {
                    return None;
                }
                let payload_len = (l2cap_len as usize).saturating_sub(3).min(att.value.len());
                let payload = att.value.get(..payload_len)?;
                Some(Self::Write {
                    handle,
                    kind,
                    l2cap_len,
                    payload,
                })
            }
            _ => None,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AttResponse {
    Silent,
    Deferred,
    ExchangeMtu,
    FindInfo {
        payload: [u8; 22],
        payload_len: u8,
    },
    FindByTypeValue {
        payload: [u8; 22],
        payload_len: u8,
    },
    ReadByType {
        payload: [u8; 22],
        payload_len: u8,
    },
    Read {
        payload: [u8; 22],
        payload_len: u8,
    },
    ReadByGroupType {
        payload: [u8; 22],
        payload_len: u8,
    },
    Write,
    Error {
        request_opcode: u8,
        handle: u16,
        error: AttError,
    },
}

fn response_template() -> Packet {
    Packet {
        att_read_rsp: PacketAttReadRsp {
            head: PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 0,
                chan_id: 0,
            },
            opcode: 0,
            value: [0; 22],
        },
    }
}

fn encode_discovery_response(opcode: u8, payload: &[u8]) -> Option<Packet> {
    if payload.len() > 22 {
        return None;
    }
    let mut response = response_template();
    let payload_len = payload.len();
    response.att_read_rsp_mut().value[..payload_len].copy_from_slice(payload);
    let l2cap_len = (payload_len as u16).saturating_add(1);
    {
        let head = response.head_mut();
        head.dma_len = l2cap_len as u32 + 6;
        head._type = 2;
        head.rf_len = (l2cap_len as u8).saturating_add(4);
        head.l2cap_len = l2cap_len;
        head.chan_id = 4;
    }
    response.att_read_rsp_mut().opcode = opcode;
    Some(response)
}

fn encode_att_response(response: AttResponse) -> Option<Packet> {
    match response {
        AttResponse::Silent | AttResponse::Deferred => None,
        AttResponse::ExchangeMtu => Some(Packet {
            att_mtu: PacketAttMtu {
                head: PacketL2capHead {
                    dma_len: 0x09,
                    _type: 2,
                    rf_len: 0x07,
                    l2cap_len: 0x03,
                    chan_id: 4,
                },
                opcode: 0x03,
                mtu: [0x17, 0x00],
            },
        }),
        AttResponse::FindInfo {
            payload,
            payload_len,
        } => encode_discovery_response(0x05, &payload[..payload_len as usize]),
        AttResponse::FindByTypeValue {
            payload,
            payload_len,
        } => encode_discovery_response(0x07, &payload[..payload_len as usize]),
        AttResponse::ReadByType {
            payload,
            payload_len,
        } => encode_discovery_response(0x09, &payload[..payload_len as usize]),
        AttResponse::Read {
            payload,
            payload_len,
        } => encode_discovery_response(0x0b, &payload[..payload_len as usize]),
        AttResponse::ReadByGroupType {
            payload,
            payload_len,
        } => encode_discovery_response(0x11, &payload[..payload_len as usize]),
        AttResponse::Write => Some(Packet {
            att_write_rsp: PacketAttWriteRsp {
                head: PacketL2capHead {
                    dma_len: 0x07,
                    _type: 2,
                    rf_len: 0x05,
                    l2cap_len: 0x01,
                    chan_id: 4,
                },
                opcode: 0x13,
            },
        }),
        AttResponse::Error {
            request_opcode,
            handle,
            error,
        } => {
            let mut packet = PKT_ERR_RSP;
            packet.att_err_rsp_mut().err_opcode = request_opcode;
            packet.att_err_rsp_mut().err_handle = handle;
            packet.att_err_rsp_mut().err_reason = error as u8;
            Some(packet)
        }
    }
}

pub struct GattServer<B: GattBackend> {
    profile: ProfileView<B::Binding>,
    _backend: PhantomData<B>,
}

impl<B: GattBackend> GattServer<B> {
    pub const fn new(profile: ProfileView<B::Binding>) -> Self {
        Self {
            profile,
            _backend: PhantomData,
        }
    }

    #[inline(never)]
    pub fn handle_packet(&self, packet: &Packet) -> Option<Packet> {
        let request = AttRequest::parse(packet)?;
        let response = match request {
            AttRequest::ExchangeMtu => self.handle_exchange_mtu(),
            AttRequest::FindInfo { start, end } => self.handle_find_info(start, end),
            AttRequest::FindByTypeValue {
                start,
                end,
                uuid,
                target_value,
            } => self.handle_find_by_type_value(start, end, uuid, target_value),
            AttRequest::ReadByType {
                start,
                end,
                uuid,
                uuid128,
            } => self.handle_read_by_type(start, end, uuid, uuid128),
            AttRequest::Read { handle } => self.handle_read(handle),
            AttRequest::ReadByGroupType { start, end, uuid } => {
                self.handle_read_by_group_type(start, end, uuid)
            }
            AttRequest::Write {
                handle,
                kind,
                l2cap_len,
                payload,
            } => self.handle_write(handle, kind, l2cap_len, payload),
        };
        encode_att_response(response)
    }

    #[inline(never)]
    fn handle_exchange_mtu(&self) -> AttResponse {
        AttResponse::ExchangeMtu
    }

    #[inline(never)]
    fn handle_find_info(&self, start: u16, end: u16) -> AttResponse {
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);
        if start == 0 || start > end {
            return AttResponse::Error {
                request_opcode: 0x04,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        let mut payload = [0u8; 22];
        let mut format = 0u8;
        let mut payload_len = 1usize;
        let clamped_end = end.min(self.profile.total_handles() as u16);

        if start > clamped_end {
            return AttResponse::Error {
                request_opcode: 0x04,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        for handle in start..=clamped_end {
            let Some(attr) = self.profile.get(handle) else {
                continue;
            };
            let (uuid_format, uuid_len) = match attr.uuid {
                AttrUuid::Uuid16(_) => (1u8, 2usize),
                AttrUuid::Uuid128(_) => (2u8, 16usize),
            };

            if format == 0 {
                format = uuid_format;
                payload[0] = uuid_format;
            } else if format != uuid_format {
                break;
            }

            let entry_len = 2 + uuid_len;
            if payload_len + entry_len > payload.len() {
                break;
            }
            payload[payload_len] = handle as u8;
            payload[payload_len + 1] = (handle >> 8) as u8;
            match attr.uuid {
                AttrUuid::Uuid16(uuid) => {
                    payload[payload_len + 2..payload_len + 4].copy_from_slice(&uuid.to_le_bytes());
                }
                AttrUuid::Uuid128(uuid) => {
                    payload[payload_len + 2..payload_len + 18].copy_from_slice(uuid);
                }
            }
            payload_len += entry_len;
        }

        if payload_len == 1 {
            AttResponse::Error {
                request_opcode: 0x04,
                handle: start,
                error: AttError::AttributeNotFound,
            }
        } else {
            AttResponse::FindInfo {
                payload,
                payload_len: payload_len as u8,
            }
        }
    }

    #[inline(never)]
    fn handle_find_by_type_value(
        &self,
        start: u16,
        end: u16,
        uuid: [u8; 2],
        target_value: u16,
    ) -> AttResponse {
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);
        if start == 0 || start > end {
            return AttResponse::Error {
                request_opcode: 0x06,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        let mut payload = [0u8; 22];
        let mut payload_len = 0usize;
        let clamped_end = end.min(self.profile.total_handles() as u16);
        for handle in start..=clamped_end {
            let Some(attr) = self.profile.get(handle) else {
                continue;
            };
            let matches_uuid = match attr.uuid {
                AttrUuid::Uuid16(value) => value.to_le_bytes() == uuid,
                AttrUuid::Uuid128(_) => false,
            };
            if !matches_uuid {
                continue;
            }

            let mut value = [0u8; 22];
            let outcome = B::read(
                attr.binding,
                GattReadRequest {
                    handle,
                    kind: GattReadKind::ReadByType,
                    out: &mut value,
                },
            );
            let GattReadOutcome::Value(value_len) = outcome else {
                continue;
            };
            if value_len < 2 || u16::from_le_bytes([value[0], value[1]]) != target_value {
                continue;
            }

            if payload_len + 4 > payload.len() {
                break;
            }
            payload[payload_len] = handle as u8;
            payload[payload_len + 1] = (handle >> 8) as u8;
            let end_handle = match attr.role {
                AttrRole::PrimaryService { end_handle } => end_handle,
                _ => handle,
            };
            payload[payload_len + 2] = end_handle as u8;
            payload[payload_len + 3] = (end_handle >> 8) as u8;
            payload_len += 4;
        }

        if payload_len == 0 {
            AttResponse::Error {
                request_opcode: 0x06,
                handle: start,
                error: AttError::AttributeNotFound,
            }
        } else {
            AttResponse::FindByTypeValue {
                payload,
                payload_len: payload_len as u8,
            }
        }
    }

    #[inline(never)]
    fn handle_read_by_type(&self, start: u16, end: u16, uuid: &[u8], uuid128: bool) -> AttResponse {
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);
        if start == 0 || start > end {
            return AttResponse::Error {
                request_opcode: 0x08,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        let mut payload = [0u8; 22];
        let mut payload_len = 0usize;
        let clamped_end = end.min(self.profile.total_handles() as u16);

        if !uuid128 && uuid.len() == 2 {
            let requested = u16::from_le_bytes([uuid[0], uuid[1]]);
            if requested == 0x2803 {
                let mut entry_len = 0usize;
                for handle in start..=clamped_end {
                    let Some(attr) = self.profile.get(handle) else {
                        continue;
                    };
                    let AttrRole::CharacteristicDecl {
                        properties,
                        value_handle,
                    } = attr.role
                    else {
                        continue;
                    };
                    let Some(value_attr) = self.profile.get(value_handle) else {
                        continue;
                    };
                    let uuid_len = match value_attr.uuid {
                        AttrUuid::Uuid16(_) => 2usize,
                        AttrUuid::Uuid128(_) => 16usize,
                    };
                    let this_entry_len = 5 + uuid_len;
                    if entry_len != 0 && entry_len != this_entry_len {
                        break;
                    }
                    let offset = 1 + payload_len;
                    if offset + this_entry_len > payload.len() {
                        break;
                    }
                    payload[offset] = handle as u8;
                    payload[offset + 1] = (handle >> 8) as u8;
                    payload[offset + 2] = properties;
                    payload[offset + 3] = value_handle as u8;
                    payload[offset + 4] = (value_handle >> 8) as u8;
                    match value_attr.uuid {
                        AttrUuid::Uuid16(value) => {
                            payload[offset + 5..offset + 7].copy_from_slice(&value.to_le_bytes());
                        }
                        AttrUuid::Uuid128(value) => {
                            payload[offset + 5..offset + 21].copy_from_slice(value);
                        }
                    }
                    payload_len += this_entry_len;
                    entry_len = this_entry_len;
                }

                if payload_len != 0 {
                    payload[0] = entry_len as u8;
                    let total = 1 + payload_len;
                    return AttResponse::ReadByType {
                        payload,
                        payload_len: total as u8,
                    };
                }
            } else {
                for handle in start..=clamped_end {
                    let Some(attr) = self.profile.get(handle) else {
                        continue;
                    };
                    let AttrUuid::Uuid16(attr_uuid) = attr.uuid else {
                        continue;
                    };
                    if attr_uuid.to_le_bytes() != uuid {
                        continue;
                    }
                    let mut value = [0u8; 19];
                    let outcome = B::read(
                        attr.binding,
                        GattReadRequest {
                            handle,
                            kind: GattReadKind::ReadByType,
                            out: &mut value,
                        },
                    );
                    let GattReadOutcome::Value(value_len) = outcome else {
                        continue;
                    };
                    let value_len = value_len as usize;
                    if value_len > value.len() || 3 + value_len > payload.len() {
                        continue;
                    }
                    payload[0] = (2 + value_len) as u8;
                    payload[1] = handle as u8;
                    payload[2] = (handle >> 8) as u8;
                    payload[3..3 + value_len].copy_from_slice(&value[..value_len]);
                    return AttResponse::ReadByType {
                        payload,
                        payload_len: (3 + value_len) as u8,
                    };
                }
            }
        } else if uuid128 && uuid.len() == 16 {
            for handle in start..=clamped_end {
                let Some(attr) = self.profile.get(handle) else {
                    continue;
                };
                let AttrUuid::Uuid128(attr_uuid) = attr.uuid else {
                    continue;
                };
                if attr_uuid != uuid {
                    continue;
                }
                let mut value = [0u8; 19];
                let outcome = B::read(
                    attr.binding,
                    GattReadRequest {
                        handle,
                        kind: GattReadKind::ReadByType,
                        out: &mut value,
                    },
                );
                let GattReadOutcome::Value(value_len) = outcome else {
                    continue;
                };
                let value_len = value_len as usize;
                if value_len > value.len() || 3 + value_len > payload.len() {
                    continue;
                }
                payload[0] = (2 + value_len) as u8;
                payload[1] = handle as u8;
                payload[2] = (handle >> 8) as u8;
                payload[3..3 + value_len].copy_from_slice(&value[..value_len]);
                return AttResponse::ReadByType {
                    payload,
                    payload_len: (3 + value_len) as u8,
                };
            }
        }

        AttResponse::Error {
            request_opcode: 0x08,
            handle: start,
            error: AttError::AttributeNotFound,
        }
    }

    #[inline(never)]
    fn handle_read(&self, handle: u16) -> AttResponse {
        if handle == 0 || handle >> 8 != 0 {
            return AttResponse::Silent;
        }
        let Some(attr) = self.profile.get(handle) else {
            return AttResponse::Silent;
        };

        let permissions = match attr.role {
            AttrRole::PrimaryService { .. } => None,
            AttrRole::CharacteristicDecl { .. } => None,
            AttrRole::CharacteristicValue { permissions } => Some(permissions),
            AttrRole::Descriptor { permissions } => Some(permissions),
        };
        if let Some(permissions) = permissions {
            if !permissions.read {
                return AttResponse::Error {
                    request_opcode: 0x0a,
                    handle,
                    error: AttError::ReadNotPermitted,
                };
            }
        }

        let mut payload = [0u8; 22];
        let payload_len = match attr.role {
            AttrRole::CharacteristicDecl { properties, .. } => {
                payload[0] = properties;
                1
            }
            _ => {
                let outcome = B::read(
                    attr.binding,
                    GattReadRequest {
                        handle,
                        kind: GattReadKind::Read,
                        out: &mut payload,
                    },
                );
                match outcome {
                    GattReadOutcome::Value(value_len) => {
                        let value_len = value_len as usize;
                        if value_len > payload.len() {
                            return AttResponse::Error {
                                request_opcode: 0x0a,
                                handle,
                                error: AttError::InvalidAttributeValueLength,
                            };
                        }
                        value_len
                    }
                    GattReadOutcome::Deferred => return AttResponse::Deferred,
                    GattReadOutcome::Error(error) => {
                        return AttResponse::Error {
                            request_opcode: 0x0a,
                            handle,
                            error,
                        };
                    }
                }
            }
        };

        AttResponse::Read {
            payload,
            payload_len: payload_len as u8,
        }
    }

    #[inline(never)]
    fn handle_read_by_group_type(&self, start: u16, end: u16, uuid: [u8; 2]) -> AttResponse {
        GATT_SERVICE_DISCOVERY_TIMEOUT_TIMESTAMP.set(read_reg_system_tick() | 1);
        if start == 0 || start > end {
            return AttResponse::Error {
                request_opcode: 0x10,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        let mut payload = [0u8; 22];
        let mut written = 0usize;
        let mut format_len = 0usize;
        let clamped_end = end.min(self.profile.total_handles() as u16);

        for handle in start..=clamped_end {
            let Some(attr) = self.profile.get(handle) else {
                continue;
            };
            let AttrRole::PrimaryService { end_handle } = attr.role else {
                continue;
            };
            let AttrUuid::Uuid16(attr_uuid) = attr.uuid else {
                continue;
            };
            if attr_uuid.to_le_bytes() != uuid {
                continue;
            }

            let mut value = [0u8; 22];
            let outcome = B::read(
                attr.binding,
                GattReadRequest {
                    handle,
                    kind: GattReadKind::ReadByType,
                    out: &mut value,
                },
            );
            let GattReadOutcome::Value(value_len) = outcome else {
                continue;
            };
            let value_len = value_len as usize;
            if format_len != 0 && format_len != value_len {
                break;
            }
            let entry_start = 1 + written;
            if entry_start + 4 + value_len > payload.len() {
                break;
            }
            if format_len == 0 {
                format_len = value_len;
            }
            payload[entry_start] = handle as u8;
            payload[entry_start + 1] = (handle >> 8) as u8;
            payload[entry_start + 2] = end_handle as u8;
            payload[entry_start + 3] = (end_handle >> 8) as u8;
            payload[entry_start + 4..entry_start + 4 + value_len]
                .copy_from_slice(&value[..value_len]);
            written += 4 + value_len;
        }

        if written == 0 {
            return AttResponse::Error {
                request_opcode: 0x10,
                handle: start,
                error: AttError::AttributeNotFound,
            };
        }

        payload[0] = (format_len as u8).saturating_add(4);
        AttResponse::ReadByGroupType {
            payload,
            payload_len: (1 + written) as u8,
        }
    }

    #[inline(never)]
    fn handle_write(
        &self,
        handle: u16,
        kind: GattWriteKind,
        l2cap_len: u16,
        payload: &[u8],
    ) -> AttResponse {
        if handle == 0 || handle == 1 || handle >> 8 != 0 {
            return AttResponse::Silent;
        }
        let Some(attr) = self.profile.get(handle) else {
            return AttResponse::Silent;
        };

        let permissions = match attr.role {
            AttrRole::CharacteristicValue { permissions } => Some(permissions),
            AttrRole::Descriptor { permissions } => Some(permissions),
            AttrRole::PrimaryService { .. } | AttrRole::CharacteristicDecl { .. } => None,
        };
        let Some(permissions) = permissions else {
            return if kind == GattWriteKind::Request {
                AttResponse::Write
            } else {
                AttResponse::Silent
            };
        };
        if !permissions.allows(kind) {
            return if kind == GattWriteKind::Request {
                AttResponse::Write
            } else {
                AttResponse::Silent
            };
        }

        let outcome = B::write(
            attr.binding,
            GattWriteRequest {
                handle,
                kind,
                l2cap_len,
                payload,
            },
        );
        match outcome {
            GattWriteOutcome::Accepted => {
                if kind == GattWriteKind::Request {
                    AttResponse::Write
                } else {
                    AttResponse::Silent
                }
            }
            GattWriteOutcome::NoWriteResponse => AttResponse::Silent,
            GattWriteOutcome::Error(error) => {
                if kind == GattWriteKind::Request {
                    AttResponse::Error {
                        request_opcode: 0x12,
                        handle,
                        error,
                    }
                } else {
                    AttResponse::Silent
                }
            }
        }
    }
}

/// Exported firmware entry point. It dispatches only through the legacy profile.
#[export_name = "tlsr8266_l2cap_att_handler"]
#[inline(never)]
pub fn l2cap_att_handler(packet: &Packet) -> Option<Packet> {
    GattServer::<LegacyGattBackend>::new(LEGACY_PROFILE).handle_packet(packet)
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::gatt::service_table::{AttrPermissions, CompiledAttr};
    use mesh_protocol::packet_types::PacketAttRawValue;

    fn att_packet(opcode: u8, l2cap_len: u16, handle: u16, value: &[u8]) -> Packet {
        let mut raw = [0u8; 30];
        let copy_len = value.len().min(raw.len());
        raw[..copy_len].copy_from_slice(&value[..copy_len]);
        Packet {
            att_val: PacketAttRawValue {
                head: PacketL2capHead {
                    dma_len: 0,
                    _type: 2,
                    rf_len: 0,
                    l2cap_len,
                    chan_id: 4,
                },
                opcode,
                handle: handle as u8,
                handle1: (handle >> 8) as u8,
                value: raw,
            },
        }
    }

    #[derive(Clone, Copy)]
    enum TestBinding {
        Bytes(&'static [u8]),
    }

    struct TestBackend;

    impl GattBackend for TestBackend {
        type Binding = TestBinding;

        fn read(binding: Self::Binding, request: GattReadRequest<'_>) -> GattReadOutcome {
            match binding {
                TestBinding::Bytes(value) => {
                    let len = value.len().min(request.out.len()) as u8;
                    request.out[..len as usize].copy_from_slice(&value[..len as usize]);
                    GattReadOutcome::Value(len)
                }
            }
        }

        fn write(_binding: Self::Binding, _request: GattWriteRequest<'_>) -> GattWriteOutcome {
            GattWriteOutcome::Accepted
        }
    }

    static PROFILE_A_ATTRS: [CompiledAttr<TestBinding>; 2] = [
        CompiledAttr {
            role: AttrRole::PrimaryService { end_handle: 2 },
            uuid: AttrUuid::Uuid16(0x2800),
            len: 2,
            max_len: 2,
            binding: TestBinding::Bytes(&[0x00, 0x18]),
        },
        CompiledAttr {
            role: AttrRole::CharacteristicValue {
                permissions: AttrPermissions {
                    read: true,
                    write: true,
                    write_without_rsp: true,
                },
            },
            uuid: AttrUuid::Uuid16(0x2a00),
            len: 2,
            max_len: 2,
            binding: TestBinding::Bytes(&[0xaa, 0xbb]),
        },
    ];

    static PROFILE_B_ATTRS: [CompiledAttr<TestBinding>; 2] = [
        CompiledAttr {
            role: AttrRole::PrimaryService { end_handle: 2 },
            uuid: AttrUuid::Uuid16(0x2800),
            len: 2,
            max_len: 2,
            binding: TestBinding::Bytes(&[0x0a, 0x18]),
        },
        CompiledAttr {
            role: AttrRole::CharacteristicValue {
                permissions: AttrPermissions {
                    read: true,
                    write: true,
                    write_without_rsp: false,
                },
            },
            uuid: AttrUuid::Uuid16(0x2a01),
            len: 2,
            max_len: 2,
            binding: TestBinding::Bytes(&[0xcc, 0xdd]),
        },
    ];

    #[test]
    fn table_driven_typed_parser_supported_operations() {
        let packets = [
            att_packet(0x02, 3, 0, &[]),
            att_packet(0x04, 5, 1, &[29, 0]),
            att_packet(0x06, 9, 1, &[29, 0, 0, 0x28, 0, 0x18]),
            att_packet(0x08, 7, 1, &[28, 0, 0x03, 0x28]),
            att_packet(0x0a, 3, 3, &[]),
            att_packet(0x10, 7, 1, &[28, 0, 0, 0x28]),
            att_packet(0x12, 5, 19, &[0x01, 0x00]),
            att_packet(0x52, 5, 19, &[0x01, 0x00]),
        ];
        for packet in &packets {
            assert!(AttRequest::parse(packet).is_some());
        }
    }

    #[test]
    fn invalid_channel_and_malformed_requests_are_silent() {
        let mut packet = att_packet(0x02, 3, 0, &[]);
        packet.head_mut().chan_id = 5;
        assert!(AttRequest::parse(&packet).is_none());

        assert!(AttRequest::parse(&att_packet(0x04, 1, 1, &[])).is_none());
        assert!(AttRequest::parse(&att_packet(0xff, 3, 0, &[])).is_none());
    }

    #[test]
    fn independent_test_backends_do_not_share_profile_state() {
        let profile_a = ProfileView {
            platform: &PROFILE_A_ATTRS,
            product: &[],
        };
        let profile_b = ProfileView {
            platform: &PROFILE_B_ATTRS,
            product: &[],
        };
        let server_a = GattServer::<TestBackend>::new(profile_a);
        let server_b = GattServer::<TestBackend>::new(profile_b);

        let read_a = att_packet(0x0a, 3, 2, &[]);
        let read_b = att_packet(0x0a, 3, 2, &[]);
        let response_a = server_a.handle_packet(&read_a).unwrap();
        let response_b = server_b.handle_packet(&read_b).unwrap();
        assert_eq!(&response_a.att_read_rsp().value[..2], &[0xaa, 0xbb]);
        assert_eq!(&response_b.att_read_rsp().value[..2], &[0xcc, 0xdd]);

        let find_a = att_packet(0x06, 9, 1, &[2, 0, 0, 0x28, 0, 0x18]);
        let find_b = att_packet(0x06, 9, 1, &[2, 0, 0, 0x28, 0x0a, 0x18]);
        assert!(server_a.handle_packet(&find_a).is_some());
        assert!(server_b.handle_packet(&find_b).is_some());
    }

    #[test]
    fn legacy_profile_is_28_handles_and_mtu_is_23() {
        assert_eq!(LEGACY_PROFILE.total_handles(), 28);
        assert!(LEGACY_PROFILE.get(1).is_some());
        assert!(LEGACY_PROFILE.get(28).is_some());
        assert!(LEGACY_PROFILE.get(29).is_none());

        let mtu = att_packet(0x02, 3, 0, &[]);
        let response = l2cap_att_handler(&mtu).unwrap();
        assert_eq!(response.att_mtu().mtu, [0x17, 0x00]);
    }

    #[test]
    fn provisioning_address_command_reaches_typed_mesh_rx_packet() {
        crate::facade::reset_radio_state_for_test();
        crate::queues::reset_for_test();
        crate::ble::pair::PAIR_LOGIN_OK.set(true);

        let payload = [
            0x01, 0x02, 0x03, // header id
            0xaa, 0xbb, // MIC placeholder
            0x00, 0x00, // direct-to-device destination
            0xe0, // set mesh address command
            0x11, 0x02, // Telink vendor ID
            0x1f, 0x00, // requested mesh address 31
            0, 0, 0, 0, 0, 0, 0, 0,
        ];
        let packet = att_packet(0x52, 23, 21, &payload);

        assert!(l2cap_att_handler(&packet).is_none());
        let queued = crate::queues::MESH_RX_CHANNEL
            .try_receive()
            .expect("typed handle-21 provisioning write must enter mesh RX");

        let rf_len = queued.head().rf_len;
        let l2cap_len = queued.head().l2cap_len;
        let handle = queued.att_write().handle;
        let dst = queued.att_write().value.dst;
        let val0 = queued.att_write().value.val[0];
        let val3 = queued.att_write().value.val[3];
        let val4 = queued.att_write().value.val[4];
        assert_eq!(rf_len, 27, "adapter must provide RF packet length");
        assert_eq!(l2cap_len, 23, "adapter must preserve ATT L2CAP length");
        assert_eq!(handle, 21);
        assert_eq!(dst, [0, 0]);
        assert_eq!(val0 & 0x3f, 0x20);
        assert_eq!(val3, 31);
        assert_eq!(val4, 0);
    }

    #[test]
    fn legacy_read_write_and_invalid_cases() {
        let device_name = att_packet(0x0a, 3, 3, &[]);
        let response = l2cap_att_handler(&device_name).unwrap();
        assert_eq!(response.att_read_rsp().opcode, 0x0b);

        assert!(l2cap_att_handler(&att_packet(0x0a, 3, 0, &[])).is_none());
        assert!(l2cap_att_handler(&att_packet(0x0a, 3, 29, &[])).is_none());
        assert!(l2cap_att_handler(&att_packet(0x12, 3, 1, &[])).is_none());

        let cccd = att_packet(0x12, 5, 19, &[0x01, 0x00]);
        let response = l2cap_att_handler(&cccd).unwrap();
        assert_eq!(response.att_write_rsp().opcode, 0x13);

        let command = att_packet(0x52, 5, 21, &[0; 2]);
        assert!(l2cap_att_handler(&command).is_none());
    }

    /// Every ATT opcode the server accepts must reach its handler and answer
    /// with the deployed response opcode.
    ///
    /// The discovery encoders in this file were previously reached by no test
    /// at all — the ATT core sat at 42% line coverage while well-exercised
    /// modules were above 97% (epic #111 audit).
    #[test]
    fn att_opcode_matrix_reaches_every_dispatch_arm() {
        // MTU exchange -> Exchange MTU Response.
        let mtu = l2cap_att_handler(&att_packet(0x02, 3, 0, &[])).expect("mtu response");
        assert_eq!(mtu.att_read_rsp().opcode, 0x03);

        // Read -> Read Response.
        let read = l2cap_att_handler(&att_packet(0x0a, 3, 3, &[])).expect("read response");
        assert_eq!(read.att_read_rsp().opcode, 0x0b);

        // Read By Type -> Read By Type Response. The start handle rides in the
        // ATT header; the value carries end handle + UUID.
        let mut by_type = [0u8; 4];
        by_type[0..2].copy_from_slice(&28u16.to_le_bytes());
        by_type[2..4].copy_from_slice(&0x2a00u16.to_le_bytes());
        let found = l2cap_att_handler(&att_packet(0x08, 7, 1, &by_type)).expect("read by type");
        assert_eq!(found.att_read_rsp().opcode, 0x09);

        // Find Information -> Find Information Response.
        let mut info = [0u8; 2];
        info[0..2].copy_from_slice(&28u16.to_le_bytes());
        let entries = l2cap_att_handler(&att_packet(0x04, 5, 1, &info)).expect("find info");
        assert_eq!(entries.att_read_rsp().opcode, 0x05);

        // Read By Group Type -> Read By Group Type Response.
        let mut group = [0u8; 4];
        group[0..2].copy_from_slice(&28u16.to_le_bytes());
        group[2..4].copy_from_slice(&0x2800u16.to_le_bytes());
        let services = l2cap_att_handler(&att_packet(0x10, 7, 1, &group)).expect("read by group");
        assert_eq!(services.att_read_rsp().opcode, 0x11);

        // Write Request to a writable descriptor -> Write Response.
        let acked =
            l2cap_att_handler(&att_packet(0x12, 5, 19, &[0x01, 0x00])).expect("write response");
        assert_eq!(acked.att_write_rsp().opcode, 0x13);

        // Write Command -> never a response.
        assert!(l2cap_att_handler(&att_packet(0x52, 5, 19, &[0x01, 0x00])).is_none());

        // An opcode the server does not implement is silent, not a panic.
        assert!(l2cap_att_handler(&att_packet(0x1e, 3, 0, &[])).is_none());

        // A truncated Write Request carries no payload to parse. The deployed
        // firmware still answered 0x13; today it is silent. Pinned so the
        // difference is visible rather than implicit — #120 owns the full
        // per-operation response matrix.
        assert!(l2cap_att_handler(&att_packet(0x12, 2, 19, &[])).is_none());
    }

    /// The Read By Group Type encoder must lay out real service rows, not just
    /// report a length. Mutating the field extraction used to change no test.
    #[test]
    fn read_by_group_type_encodes_service_rows() {
        let mut req = [0u8; 4];
        req[0..2].copy_from_slice(&0xffffu16.to_le_bytes());
        req[2..4].copy_from_slice(&0x2800u16.to_le_bytes());

        let response = l2cap_att_handler(&att_packet(0x10, 7, 1, &req)).expect("group response");
        assert_eq!(response.att_read_rsp().opcode, 0x11);
        let value = &response.att_read_rsp().value;

        // Entry length: start(2) + end(2) + uuid16(2).
        assert_eq!(value[0], 6, "entry length byte must describe the entry");

        // First entry: GAP service covers handles 1..=6 with uuid 0x1800.
        assert_eq!(u16::from_le_bytes([value[1], value[2]]), 1);
        assert_eq!(u16::from_le_bytes([value[3], value[4]]), 6);
        assert_eq!(u16::from_le_bytes([value[5], value[6]]), 0x1800);

        // Second entry: Device Information covers 7..=9 with uuid 0x180a.
        assert_eq!(u16::from_le_bytes([value[7], value[8]]), 7);
        assert_eq!(u16::from_le_bytes([value[9], value[10]]), 9);
        assert_eq!(u16::from_le_bytes([value[11], value[12]]), 0x180a);
    }

    /// The Find Information encoder must emit handle/uuid pairs.
    #[test]
    fn find_information_encodes_handle_uuid_pairs() {
        let mut req = [0u8; 2];
        req[0..2].copy_from_slice(&2u16.to_le_bytes());

        let response =
            l2cap_att_handler(&att_packet(0x04, 5, 1, &req)).expect("find info response");
        assert_eq!(response.att_read_rsp().opcode, 0x05);
        let value = &response.att_read_rsp().value;

        // Format 0x01 selects 16-bit UUIDs; entries are bare handle/uuid pairs
        // with no per-entry length byte.
        assert_eq!(value[0], 1, "format byte selects 16-bit UUIDs");
        assert_eq!(u16::from_le_bytes([value[1], value[2]]), 1);
        assert_eq!(u16::from_le_bytes([value[3], value[4]]), 0x2800);
        // Two handles were requested, so a second pair must follow.
        assert_eq!(u16::from_le_bytes([value[5], value[6]]), 2);
    }

    #[test]
    fn gatt_compat_fixture_profile_parity() {
        let source = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/fixtures/gatt_compat.txt"
        ));
        let mut row_count = 0;
        for line in source.lines() {
            if line.starts_with("gatt.row.") && line.contains(".uuid_len =") {
                row_count += 1;
            }
        }
        assert_eq!(row_count, 29);
        assert_eq!(LEGACY_PROFILE.platform.len(), 28);
        assert!(source.contains("gatt.att.mtu = 23"));
        assert!(source.contains("gatt.att.write_rsp_opcode = 13"));
    }
}
