import argparse
import datetime
import math
import struct
import sys
import threading
from time import sleep

import serial

# CMD
# 0 = counter
# 1 = not used
# 2 = cmd
# 3 .. 42 = data
# 42 .. 43 = crc16 of 0 .. 42


# ACK
# 0 = counter
# 1 = ACK cmd = 0xff
# 2 .. 42 empty
# 42 .. 43 crc16 of 0 .. 42

ENABLE_UART = 0x01
LIGHT_CTRL = 0x02
LIGHT_STATUS = 0x03
MESH_MESSAGE = 0x04
PANIC_MESSAGE = 0x05
PRINT_MESSAGE = 0x06
ACK = 0xff

# 0.125 / 0.15
ACK_TIMEOUT = 0.2
counter = 0
connection = None


def reset_connection():
    global connection
    connection = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)


events = {}


def crc16(packet):
    length = len(packet) - 2
    poly = [0, 0xa001]
    crc = 0xffff

    for j in range(0, length):
        ds = packet[j]

        for i in range(8):
            crc = (crc >> 1) ^ poly[(crc ^ ds) & 1] & 0xffff
            ds >>= 1

    return crc


def send_cmd(cmd, data):
    assert len(data) <= 15
    global counter
    while True:
        packet = [0]*44

        packet[0] = counter & 0xff

        packet[2] = cmd

        packet[3:3+len(data)] = data[:]
        packet[18] = 0
        packet[19] = 0

        crc = crc16(packet)
        packet[42] = crc & 0xff
        packet[43] = (crc >> 8) & 0xff

        connection.write(packet)
        connection.flush()

        idx = counter & 0xff
        counter += 1
        events[idx] = threading.Event()
        if events[idx].wait(ACK_TIMEOUT):
            del events[idx]
            break


last_packet = None
last_packet_wait = threading.Event()


def get_last_packet():
    return last_packet


def clear_last_packet():
    global last_packet
    last_packet = None


def recv_thread():
    read_data = bytearray()
    while True:
        while len(read_data) < 44:
            read_data.extend(connection.read())

        packet = read_data[:44]
        read_data = read_data[44:]

        # Check if this is a panic message and deal with it separately
        if packet[2] == PRINT_MESSAGE:
            msg = packet[3:]
            print(msg[:msg.index(0) if 0 in msg else len(msg)].decode('ascii', errors='ignore'), end="")

        # Check if this is a panic message and deal with it separately
        if packet[2] == PANIC_MESSAGE:
            packet = packet[3:]
            print(packet[:packet.index(0) if 0 in packet else len(packet)].decode('ascii', errors='ignore'), end="")
            continue

        crc = crc16(packet)
        if crc & 0xff != packet[42] or (crc >> 8) & 0xff != packet[43]:
            # print("CRC error: ", packet)
            reset_connection()
            continue

        if packet[1] == ACK:
            if packet[0] in events:
                events[packet[0]].set()
                continue
            else:
                continue

        # Ack the packet
        ack_pkt = [0] * 44

        ack_pkt[0] = packet[0]
        ack_pkt[1] = 0xff

        crc = crc16(ack_pkt)
        ack_pkt[42] = crc & 0xff
        ack_pkt[43] = (crc >> 8) & 0xff

        connection.write(ack_pkt)
        connection.flush()

        global last_packet
        last_packet = packet
        last_packet_wait.set()


class PacketParser:
    def __init__(self, data):
        self.progress = 0
        self.index = -1
        self.data = data

        if len(self.data) % 8 == 0:
            self.total = int(len(self.data) / 8)
        else:
            self.total = int(math.floor(len(self.data) / 8 + 1))

    def has_next_packet(self):
        return self.total > 0 and (self.index + 1) < self.total

    def get_next_packet_index(self):
        return self.index + 1

    def get_next_packet(self):
        index = self.get_next_packet_index()
        packet = self.get_packet(index)
        self.index = index
        return packet

    def get_packet(self, index):
        length = len(self.data)
        size = 8

        if length > size:
            if index + 1 == self.total:
                packet_size = length - index * size
            else:
                packet_size = size
        else:
            packet_size = length

        packet_size += 2

        packet = [0]*10
        for i in range(2, len(packet)-2):
            packet[i] = 0xff

        packet[2:2+packet_size-2] = self.data[index*size:index*size+packet_size-2]

        self.fill_index(packet, index)

        packet = bytearray(packet)
        # print(f"ota packet ----> index: {index} total: {self.total} crc: {crc} content: {binascii.hexlify(packet, ':')}")

        return packet

    def fill_index(self, packet, index):
        packet[0] = index & 0xff
        packet[1] = (index >> 8) & 0xff

    def crc16(self, packet):
        length = len(packet) - 2
        poly = [0, 0xa001]
        crc = 0xffff

        for j in range(0, length):
            ds = packet[j]

            for i in range(8):
                crc = (crc >> 1) ^ poly[(crc ^ ds) & 1] & 0xffff
                ds >>= 1

        return crc


def get_response(message, local_id, response_id, resp_val=None, max_retry=None):
    data = [local_id & 0xff, (local_id >> 8) & 0xff] + message
    send_cmd(LIGHT_CTRL, data)

    while not get_last_packet() or get_last_packet()[6] != local_id or get_last_packet()[10] & 0x3f != response_id or \
        (resp_val is not None and get_last_packet()[11:11+len(resp_val)] != resp_val):

        last_packet_wait.clear()
        if not last_packet_wait.wait(0.5):
            if max_retry is not None:
                if max_retry == 0:
                    return None

                max_retry -= 1

            send_cmd(LIGHT_CTRL, data)

    result = get_last_packet()
    clear_last_packet()
    return result


def flash_node(local_id, fw_file, force):
    print(f"Writing FW to node {local_id}")

    # Check that we should bother updating the firmware
    with open(fw_file, 'rb') as f:
        parser = PacketParser(f.read())

    # Send OTA start (Returns the current firmware version on the device)
    dev_ver = get_response([0x24, 0], local_id, 0x25, max_retry=10)

    # Light can't be reached
    if dev_ver is None:
        print("Device didn't respond.")
        return

    dev_ver = struct.unpack("I", dev_ver[13:13 + 4])[0]
    fw_ver = struct.unpack("I", parser.data[2:6])[0]

    dev_16mhz = (dev_ver >> 31) == 1
    dev_ver = dev_ver & 0x7fffffff

    fw_16mhz = (fw_ver >> 31) == 1
    fw_ver = fw_ver & 0x7fffffff

    print(f"Device version: {dev_ver}, new version: {fw_ver}, 16MHz device: {dev_16mhz}")

    if dev_16mhz != fw_16mhz:
        print(f"Attempt to flash incompatible crystal version of firmware. Bailing out. Device is 16MHz? "
              f"{dev_16mhz}, FW is 16MHz? {fw_16mhz}", file=sys.stderr)

    if dev_ver >= fw_ver:
        print("Device firmware is newer or the same as the provided firmware file. Nothing to do.")
        if not force:
            print("Bailing out.")
            return

    # Flash the firmware
    now = datetime.datetime.now()
    completion = int(parser.index / parser.total * 100.)
    while parser.has_next_packet():
        new_completion = int(parser.index / parser.total * 100.)
        if new_completion != completion:
            completion = new_completion
            print(f"{completion}% {datetime.datetime.now() - now}")

        packet = list(parser.get_next_packet())
        response = get_response([0x26, 0, 0] + packet, local_id, 0x27, bytearray(packet[0:2]), max_retry=100)
        # Light has vanished?
        if response is None:
            print("Device has disappeared")
            break

        assert bytearray(packet[0:2]) == bytearray(response[11:13])
        sleep(0.05)

    # Send the final packet and finish the OTA update
    packet = list(parser.get_next_packet())
    data = [local_id & 0xff, (local_id >> 8) & 0xff, 0x28, 0, 0] + packet

    for _ in range(10):
        send_cmd(LIGHT_CTRL, data)

    print("Firmware written")


def main():
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("--fw_file", help="Firmware file to flash", required=True, type=str)
    args_parser.add_argument("--mesh_address", help="Mesh address of the node to update (number between 1 and 63)", required=True, type=int)
    args_parser.add_argument("--force", required=False, default=False, action=argparse.BooleanOptionalAction)
    args = args_parser.parse_args()

    # Open the serial connection
    reset_connection()

    # Start the receiver thread
    threading.Thread(target=recv_thread, daemon=True).start()

    # Ask the MCU to start reporting
    send_cmd(ENABLE_UART, [])

    print("UART initialised")

    flash_node(args.mesh_address, args.fw_file, args.force)


if __name__ == "__main__":
    main()