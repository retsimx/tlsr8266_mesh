use core::cmp::min;
use core::ptr::{addr_of, addr_of_mut, slice_from_raw_parts_mut};
use core::slice;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::{app, BIT, uprintln};
use crate::embassy::time_driver::clock_time64;
use crate::main_light::{rf_link_data_callback, rf_link_response_callback};
use crate::mesh::{MESH_NODE_ST_VAL_LEN, mesh_node_st_val_t};
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_1US, clock_time, clock_time_exceed};
use crate::sdk::mcu::register::{read_reg_system_tick};
use crate::sdk::packet_types::{*};
use crate::sdk::light::{*};
use crate::state::{*};

/// Updates the mesh node status from received packet data
pub fn mesh_node_update_status(pkt: &[mesh_node_st_val_t]) -> u32
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    let mut src_index = 0;
    let mut result = 0xfffffffe;
    let tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
    while src_index < pkt.len() && pkt[src_index].dev_adr != 0 {
        // todo: This needs to be removed when we figure out why dev address 1 is being introduced
        // todo: incorrectly
        if pkt[src_index].dev_adr == 1 {
            src_index += 1;
            continue;
        }

        if DEVICE_ADDRESS.get() as u8 != pkt[src_index].dev_adr {
            let mesh_node_max = MESH_NODE_MAX.get();
            let mut current_index = 1;
            let mut mesh_node = &mut mesh_node_st[current_index];
            if mesh_node_max >= 2 {
                if mesh_node.val.dev_adr != pkt[src_index].dev_adr {
                    for tidx in 1..MESH_NODE_MAX_NUM {
                        current_index = tidx;
                        mesh_node = &mut mesh_node_st[current_index];

                        if mesh_node_max <= tidx as u8 || pkt[src_index].dev_adr == mesh_node.val.dev_adr {
                            break;
                        }
                    }
                }
            }

            if MESH_NODE_MAX_NUM == current_index {
                return 1;
            }

            if mesh_node_max as usize == current_index {
                MESH_NODE_MAX.inc();

                mesh_node.val = pkt[src_index];
                mesh_node.tick = tick;

                MESH_NODE_MASK.lock()[mesh_node_max as usize >> 5] |= 1 << (mesh_node_max & 0x1f);

                result = mesh_node_max as u32;
            } else if current_index < mesh_node_max as usize {
                let sn_difference = pkt[src_index].sn - mesh_node.val.sn;
                let par_match = pkt[src_index].par == mesh_node.val.par;

                // todo: Why the divided by two here?
                let timeout = (ONLINE_STATUS_TIMEOUT * 1000) / 2;

                result = current_index as u32;
                if sn_difference - 2 < 0x3f || (sn_difference != 0 && (mesh_node.tick == 0 || (((timeout * CLOCK_SYS_CLOCK_1US) >> 0x10) as u16) < tick - mesh_node.tick)) {
                    mesh_node.val = pkt[src_index];

                    if !par_match || mesh_node.tick == 0 {
                        MESH_NODE_MASK.lock()[current_index >> 5] |= 1 << (current_index & 0x1f);
                    }

                    mesh_node.tick = tick;
                }
            }
        }

        src_index += 1;
    }
    return 1;
}

/// Flushes mesh node status by checking for timed out nodes
#[cfg_attr(test, mry::mry)]
pub fn mesh_node_flush_status()
{
    static TICK_NODE_REPORT: AtomicU32 = AtomicU32::new(0);

    // Only report status every 500ms
    if !clock_time_exceed(TICK_NODE_REPORT.load(Ordering::Relaxed), 500000) {
        return;
    }

    let tick = read_reg_system_tick();
    TICK_NODE_REPORT.store(tick, Ordering::Relaxed);

    let mut mesh_node_st = MESH_NODE_ST.lock();

    // Iterate over each mesh node and check if it's timed out
    for count in 1..MESH_NODE_MAX.get() as usize {
        if mesh_node_st[count].tick != 0 && (CLOCK_SYS_CLOCK_1US * ONLINE_STATUS_TIMEOUT * 1000) >> 0x10 < (tick >> 0x10 | 1) - mesh_node_st[count].tick as u32 {
            mesh_node_st[count].tick = 0;

            // Set the bit in the mask so that the status is reported (Since the device has changed to offline now)
            MESH_NODE_MASK.lock()[count >> 5] |= 1 << (count & 0x1f);
        }
    }
}

/// Keeps the current device alive in the mesh network
fn mesh_node_keep_alive()
{
    DEVICE_NODE_SN.inc();
    if DEVICE_NODE_SN.get() == 0 {
        DEVICE_NODE_SN.set(1);
    }

    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st[0].val.sn = DEVICE_NODE_SN.get();
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;
}

/// Advertises the status of mesh nodes
fn mesh_node_adv_status(p_data: &mut [u8]) -> u32
{
    static MESH_NODE_CUR: AtomicUsize = AtomicUsize::new(1);

    // Clear the result data
    p_data.fill(0);

    // Get the max number of elements the result data can hold
    let mut elems = p_data.len() / MESH_NODE_ST_VAL_LEN;
    if (MESH_NODE_MAX.get() as usize) < p_data.len() / MESH_NODE_ST_VAL_LEN {
        elems = MESH_NODE_MAX.get() as usize;
    }

    {
        let mut mesh_node_st = MESH_NODE_ST.lock();

        // Copy our status in to the result data first
        p_data[0..MESH_NODE_ST_VAL_LEN].copy_from_slice(
            unsafe {
                slice::from_raw_parts(
                    addr_of!(mesh_node_st[0].val) as *const u8,
                    MESH_NODE_ST_VAL_LEN,
                )
            }
        );
    }

    // Update our own record to keep our status record in sync
    mesh_node_keep_alive();

    let mut mesh_node_st = MESH_NODE_ST.lock();

    let max_node = MESH_NODE_MAX.get() as usize;
    let mut count = 1;

    let mut out_index = count;
    while out_index < elems && count < max_node {
        let mnc = MESH_NODE_CUR.load(Ordering::Relaxed);
        if mnc < max_node && mesh_node_st[mnc].tick != 0 {
            let ptr = MESH_NODE_ST_VAL_LEN * out_index;
            out_index = out_index + 1;
            p_data[ptr..ptr + MESH_NODE_ST_VAL_LEN].copy_from_slice(
                unsafe {
                    slice::from_raw_parts(
                        addr_of!(mesh_node_st[mnc].val) as *const u8,
                        MESH_NODE_ST_VAL_LEN,
                    )
                }
            );
        }

        MESH_NODE_CUR.store(mnc + 1, Ordering::Relaxed);

        if max_node <= MESH_NODE_CUR.load(Ordering::Relaxed) {
            MESH_NODE_CUR.store(1, Ordering::Relaxed);
        }

        count += 1;
    }

    return out_index as u32;
}

/// Sends online status message to mesh network
pub fn mesh_send_online_status()
{
    static ADV_ST_SN: AtomicU32 = AtomicU32::new(0);
    static LAST_STATUS_TIME: AtomicU32 = AtomicU32::new(0);

    if !clock_time_exceed(LAST_STATUS_TIME.get(), 1000 * SEND_MESH_STATUS_INTERVAL_MS) {
        return;
    }

    LAST_STATUS_TIME.set(clock_time());

    let mut pkt_light_adv_status = Packet {
        att_write: PacketAttWrite {
            head: PacketL2capHead {
                dma_len: 0x27,
                _type: 2,
                rf_len: 0x25,
                l2cap_len: 0x21,
                chan_id: 0xffff,
            },
            opcode: 0,
            handle: 0,
            handle1: 0,
            value: PacketAttValue::default(),
        }
    };

    let pktdata = unsafe { &mut *slice_from_raw_parts_mut(addr_of!(pkt_light_adv_status.att_write().value) as *mut u8, core::mem::size_of::<PacketAttValue>()) };

    mesh_node_flush_status();
    mesh_node_adv_status(&mut pktdata[..24]);

    ADV_ST_SN.store(ADV_ST_SN.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    unsafe {
        let val = ADV_ST_SN.load(Ordering::Relaxed);
        slice::from_raw_parts_mut(addr_of_mut!(pkt_light_adv_status.att_write_mut().opcode), 3).copy_from_slice(slice::from_raw_parts(addr_of!(val) as *const u8, 3))
    }

    pktdata[24..28].fill(0xa5);

    // todo: Perhaps this send count should be greater than 0 to improve reliability
    app().mesh_manager.add_send_mesh_msg(&pkt_light_adv_status, 0, 0);
}

/// Constructs a mesh packet with the given parameters
pub fn mesh_construct_packet(sno: u32, dst: u16, cmd_op_para: &[u8], retransmit_count: u8, send_ack: bool) -> Packet
{
    assert!(cmd_op_para.len() > 2);
    assert!(cmd_op_para.len() <= 13);

    let device_address = DEVICE_ADDRESS.get();

    let mut pkt = MeshPkt {
        head: PacketL2capHead {
            dma_len: 0x27,
            _type: 2,
            rf_len: 0x25,
            l2cap_len: 0x21,
            chan_id: 0xff03,
        },
        src_tx: device_address,
        handle1: 0,
        sno: [0; 3],
        src_adr: device_address,
        dst_adr: dst,
        op: 0,
        vendor_id: 0,
        par: [0; 10],
        internal_par1: [0; 5],
        ttl: 0,
        internal_par2: [0; 4],
        no_use: [0; 4],
    };

    // Set send number
    pkt.sno[0] = sno as u8;
    pkt.sno[1] = (sno >> 8) as u8;
    pkt.sno[2] = (sno >> 16) as u8;

    // todo: replace this unsafe
    // Set opcode
    // pkt.op = cmd_op_para[0];

    unsafe {
        slice::from_raw_parts_mut(addr_of_mut!(pkt.op), cmd_op_para.len()).copy_from_slice(cmd_op_para)
    }

    pkt.internal_par1[INTERNAL_PAR_RETRANSMIT_COUNT] = retransmit_count;
    pkt.internal_par1[INTERNAL_PAR_SEND_ACK] = if send_ack { 1 } else { 0 };

    Packet { mesh: pkt }
}

/// Enables or disables mesh status reporting
#[cfg_attr(test, mry::mry)]
pub fn mesh_report_status_enable(enable: bool)
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    if enable {
        if MESH_NODE_MAX.get() >> 5 != 0 {
            mesh_node_mask.iter_mut().for_each(|v| { *v = 0xfffffffe });
        }

        if MESH_NODE_MAX.get() & 0x1f != 0 {
            mesh_node_mask[MESH_NODE_MAX.get() as usize >> 5] = (1 << (MESH_NODE_MAX.get() & 0x1f)) - 1;
        }
    }

    MESH_NODE_REPORT_ENABLE.set(enable);
}

/// Enables mesh status reporting for specific device addresses
pub fn mesh_report_status_enable_mask(data: &[u8])
{
    let mut mesh_node_mask = MESH_NODE_MASK.lock();
    let mut mesh_node_st = MESH_NODE_ST.lock();

    MESH_NODE_REPORT_ENABLE.set(data[0] != 0);
    if MESH_NODE_REPORT_ENABLE.get() && data.len() > 1 {
        for index in 1..data.len() {
            if MESH_NODE_MAX.get() != 0 {
                mesh_node_st.iter_mut().enumerate().for_each(|(i, v)| {
                    if data[index] == v.val.dev_adr {
                        mesh_node_mask[i >> 5] |= 1 << (i & 0x1f);
                    }
                });
            }
        }
    }
}

/// Checks if a packet matches group or device addresses
#[cfg_attr(test, mry::mry)]
pub fn rf_link_match_group_mac(pkt: &Packet) -> (bool, bool)
{
    let mut group_match = false;
    let mut device_match = false;

    if pkt.ll_app().value.dst & !DEVICE_ADDR_MASK_DEFAULT != 0 {
        for addr in *GROUP_ADDRESS.lock() {
            if addr == pkt.ll_app().value.dst {
                group_match = true;
                break;
            }
        }
        if pkt.ll_app().value.dst == 0xffff {
            group_match = true;
        }
    } else {
        let addr = pkt.ll_app().value.dst & DEVICE_ADDR_MASK_DEFAULT;
        if addr == 0 || addr == DEVICE_ADDRESS.get() {
            device_match = true;
        }
    }

    (group_match, device_match)
}

/// Updates device status with the given parameters
pub fn ll_device_status_update(val_par: &[u8])
{
    let mut mesh_node_st = MESH_NODE_ST.lock();

    mesh_node_st[0].val.par.copy_from_slice(val_par);
    mesh_node_st[0].tick = ((read_reg_system_tick() >> 0x10) | 1) as u16;

    MESH_NODE_MASK.lock()[0] |= 1;
}
