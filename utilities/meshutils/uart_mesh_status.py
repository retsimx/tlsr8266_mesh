#!/usr/bin/env python3
"""
UART Mesh Node Status Monitor for TLSR8266 Mesh Light System

Sends periodic ENABLE_UART commands over UART to enable mesh node status
reporting on the firmware.  The firmware immediately replies with a full
initial dump of all known nodes, then sends incremental NodeStatus packets
whenever a node goes online/offline or its on/off state changes.

Packet format (NODE_STATUS = 0x07):
  byte[0]  counter
  byte[1]  0x00 (not ACK)
  byte[2]  NODE_STATUS (0x07)
  byte[3+] entries, 3 bytes each: [node_id, online, on_off]
             online: 1 = reachable, 0 = timed out / gone offline
             on_off: 1 = light on,  0 = light off
  byte[42-43]  CRC16

Up to 13 entries fit per packet ((44-3)/3 = 13).

Usage:
  python uart_mesh_status.py [--port /dev/ttyUSB0] [--baudrate 115200]

Examples:
  python uart_mesh_status.py
  python uart_mesh_status.py --port /dev/ttyUSB1
"""

import argparse
import datetime
import signal
import sys
import threading
from time import sleep

import serial

from uart_constants import (
    ACK, ENABLE_UART, LIGHT_STATUS, MESH_MESSAGE, NODE_STATUS, PACKET_LENGTH,
    PANIC_MESSAGE, PRINT_MESSAGE, UART_BAUDRATE,
)
from uart_protocol import (
    build_ack, build_packet, check_crc, decode_mesh_message,
    decode_node_status, frame_payload, timestamp as ts,
)

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

connection   = None
running      = True
print_buffer = ""   # reassembly buffer for chunked PRINT_MESSAGE packets

REQUEST_INTERVAL_S = 1  # how often to send ENABLE_UART keep-alive


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def send_ack(counter: int):
    connection.write(build_ack(counter))
    connection.flush()


# ---------------------------------------------------------------------------
# Packet parsing
# ---------------------------------------------------------------------------

def print_node_status(payload: bytes):
    """Parse and display NODE_STATUS (0x07) entries."""
    for node_id, online, on_off in decode_node_status(payload):
        now         = ts()
        online_str  = "ONLINE " if online else "OFFLINE"
        on_str      = "ON " if on_off else "OFF"
        print(f"[{now}] node 0x{node_id:02X} ({node_id:3d})  {online_str}  {on_str}")


def process_packet(pkt: bytes):
    global print_buffer

    cmd_code = pkt[2]
    payload  = frame_payload(pkt)
    now      = ts()

    if cmd_code == NODE_STATUS:
        print_node_status(payload)

    elif cmd_code == LIGHT_STATUS:
        # Old-format LightStatus (0x03) — firmware no longer sends these for status,
        # but handle gracefully just in case.
        trimmed = payload.rstrip(b"\x00")
        if trimmed:
            print(f"[{now}] LIGHT_STATUS (old fmt): {trimmed.hex()}")

    elif cmd_code == PRINT_MESSAGE:
        chunk = payload.rstrip(b"\x00").decode("ascii", errors="replace")
        print_buffer += chunk
        while "\n" in print_buffer:
            line, print_buffer = print_buffer.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                print(f"[{now}] DEBUG: {line}")
        if len(print_buffer) > 200:
            print(f"[{now}] DEBUG: {print_buffer.strip()}")
            print_buffer = ""

    elif cmd_code == PANIC_MESSAGE:
        msg = payload.rstrip(b"\x00").decode("ascii", errors="replace").strip()
        if msg:
            print(f"[{now}] PANIC: {msg}")

    elif cmd_code == MESH_MESSAGE:
        msg = decode_mesh_message(payload)
        if msg is None:
            print(f"[{now}] MESH:   (short) {payload[:10].hex()}")
        else:
            print(
                f"[{now}] MESH:   src=0x{msg.src:04X} dst=0x{msg.dst:04X} "
                f"op=0x{msg.op:02X} vendor=0x{msg.vendor:04X} "
                f"params={msg.params.hex()}"
            )

    else:
        trimmed = payload.rstrip(b"\x00")
        if trimmed:
            print(f"[{now}] OTHER:  cmd=0x{cmd_code:02X} payload={trimmed.hex()}")


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------

def receiver_thread():
    global running, print_buffer

    read_buf   = bytearray()
    crc_errors = 0
    last_rx    = 0.0

    print(f"[{ts()}] Receiver started — waiting for packets …")

    while running:
        try:
            chunk = connection.read(PACKET_LENGTH - len(read_buf))
            if chunk:
                read_buf.extend(chunk)
                last_rx = datetime.datetime.now().timestamp()

            if len(read_buf) < PACKET_LENGTH:
                continue

            pkt      = bytes(read_buf[:PACKET_LENGTH])
            read_buf = read_buf[PACKET_LENGTH:]

            check = check_crc(pkt)
            if not check.ok:
                crc_errors += 1
                print(f"[{ts()}] CRC ERROR #{crc_errors}: "
                      f"expected 0x{check.expected:04X} got 0x{check.actual:04X}")
                continue

            if pkt[1] == ACK:
                continue

            send_ack(pkt[0])
            process_packet(pkt)

        except serial.SerialException as exc:
            print(f"[{ts()}] Serial error: {exc}")
            running = False
            break
        except Exception as exc:
            print(f"[{ts()}] Unexpected error: {exc}")

        now = datetime.datetime.now().timestamp()
        if print_buffer.strip() and last_rx and now - last_rx > 2.0:
            print(f"[{ts()}] DEBUG: {print_buffer.strip()}")
            print_buffer = ""


def request_thread():
    """Periodically send ENABLE_UART to keep status reporting active across reboots."""
    enable_pkt = build_packet(ENABLE_UART)
    first = True
    while running:
        try:
            connection.write(enable_pkt)
            connection.flush()
            if first:
                print(f"[{ts()}] Sent ENABLE_UART — firmware will send an initial node "
                      "dump then stream changes as they happen")
                first = False
        except Exception as exc:
            print(f"[{ts()}] Failed to send request: {exc}")

        for _ in range(REQUEST_INTERVAL_S * 10):
            if not running:
                break
            sleep(0.1)


def signal_handler(signum, frame):
    global running
    print(f"\n[{ts()}] Shutting down …")
    running = False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global connection, running

    parser = argparse.ArgumentParser(
        description="TLSR8266 Mesh Node Status Monitor via UART",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port",     default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baudrate", default=UART_BAUDRATE, type=int, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    print(f"[{ts()}] Connecting to {args.port} @ {args.baudrate} baud …")
    try:
        connection = serial.Serial(args.port, args.baudrate, timeout=1)
    except Exception as exc:
        print(f"Error: {exc}")
        return 1

    print(f"[{ts()}] Connected.")
    print()
    print("  What to expect:")
    print("  • NODE_STATUS packets arrive as 3-byte entries: [node_id, online, on_off]")
    print("  • Firmware sends a full initial dump on the first ENABLE_UART command,")
    print("    then incremental updates whenever a node goes online/offline or toggles.")
    print("  • ONLINE/OFFLINE reflects whether the node is still broadcasting")
    print("    (timed out after ~4 s of silence).")
    print("  • ON/OFF reflects par[0] — the last known light state for that node.")
    print()

    rx_thread  = threading.Thread(target=receiver_thread, daemon=True)
    req_thread = threading.Thread(target=request_thread,  daemon=True)
    rx_thread.start()
    req_thread.start()

    try:
        while running:
            sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        connection.close()
        print(f"[{ts()}] Done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
