import asyncio
import json
import math
import os
import random
import struct
import sys

import requests
from Cryptodome.Cipher import AES
from Cryptodome.Random import get_random_bytes
from bleak import BleakClient
import argparse


mesh_name = "Filled later"
mesh_pass = "Filled later"
shared_key = get_random_bytes(8)

pair_characteristic_uuid = "00010203-0405-0607-0809-0a0b0c0d1914"
notify_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1911'
command_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1912'
ota_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1913'
firmware_revision_uuid = '00002a26-0000-1000-8000-00805f9b34fb'


def encode_mesh_credentials():
    global mesh_name, mesh_pass
    if len(mesh_name) > 16:
        mesh_name = mesh_name[:16]

    mesh_name = mesh_name.encode('utf-8')
    while len(mesh_name) < 16:
        mesh_name += b'\0'

    if len(mesh_pass) > 16:
        mesh_pass = mesh_pass[:16]

    mesh_pass = mesh_pass.encode('utf-8')
    while len(mesh_pass) < 16:
        mesh_pass += b'\0'

    plaintext = b''

    for i in range(16):
        plaintext += (mesh_name[i] ^ mesh_pass[i]).to_bytes(length=1, byteorder='little')

    return plaintext


def encrypt_data(enc_key, data):
    enc_key = bytearray(enc_key)
    enc_key.reverse()

    data = bytearray(data)
    data.reverse()

    cipher = AES.new(bytes(enc_key), AES.MODE_ECB)
    result = bytearray(cipher.encrypt(bytes(data)))

    return result


def reverse_section(array, begin, end):
    while begin < end:
        tmp = array[end]
        array[end] = array[begin]
        array[begin] = tmp

        begin += 1
        end -= 1


def process_session_key(data, plaintext_mesh_credentials):
    # Parse the login result
    sk = data[1:1 + 16]
    rands = data[1:1 + 8]

    key = rands[:]
    while len(key) < 16:
        key += b'\0'

    encrypted = encrypt_data(key, plaintext_mesh_credentials)
    result = bytearray(b'\0' * 16)
    result[:len(rands)] = rands[:]
    result[8:8 + 8] = encrypted[8:8 + 8]

    reverse_section(result, 8, 15)

    if result != sk:
        raise Exception("session_key invalid")

    key = bytearray(shared_key[:])
    key[8:] = rands[:]

    session_key = bytearray(encrypt_data(plaintext_mesh_credentials, key))
    reverse_section(session_key, 0, len(session_key) - 1)

    return session_key


class UniqueHeaderGenerator:
    def __init__(self):
        self.counter = 0x7fffffff

    def generate_next(self):
        max_num = 0xffffff
        if self.counter > max_num:
            self.counter = round(random.random() * (max_num - 0x80FFFF)) + 1

        self.counter += 1
        return self.counter


unique_header_id_generator = UniqueHeaderGenerator()


class Command:
    def __init__(self, data, characteristic_uuid, device):
        self.data = data
        self.characteristic_uuid = characteristic_uuid
        self.device = device

    async def write(self):
        await self.device.write_gatt_char(self.characteristic_uuid, self.data)

    async def read(self):
        return await self.device.read_gatt_char(self.characteristic_uuid),


class CommandAction:
    def __init__(self, base):
        self.mac_address = base.mac_address
        self.session_key = base.session_key
        self.callback = base.callback
        self.no_response = base.no_response
        self.mesh_address = base.mesh_address
        self.opcode = base.opcode
        self.params = base.params
        self.vendor_id = base.vendor_id

        self.header_id = unique_header_id_generator.generate_next()

    async def encode_and_send(self, con):
        await self.send_command(self.encode_command(), con)

    def encode_command(self):
        command = bytearray(b'\0'*20)
        offset = 0

        # Write the header id
        command[offset] = self.header_id & 255
        offset += 1
        command[offset] = self.header_id >> 8 & 255
        offset += 1
        command[offset] = self.header_id >> 16 & 255
        offset += 1
        offset += 1
        offset += 1
        command[offset] = self.mesh_address & 255
        offset += 1
        command[offset] = self.mesh_address >> 8 & 255
        offset += 1
        print(self.opcode)
        command[offset] = (self.opcode & 0xff).to_bytes(length=4, byteorder='little', signed=True)[0] | 192
        offset += 1
        command[offset] = self.vendor_id >> 8 & 255
        offset += 1
        command[offset] = self.vendor_id & 255
        offset += 1
        if self.params:
            command[offset:offset+len(self.params)] = self.params[:]

        return command

    def generate_nonce(self):
        mac_bytes = bytearray(self.mac_address)[:]
        ivs = bytearray(b'\0'*8)
        ivs[:len(mac_bytes)] = mac_bytes[:]
        ivs[4] = 1
        ivs[5] = self.header_id & 255
        ivs[6] = (self.header_id >> 8) & 255
        ivs[7] = (self.header_id >> 16) & 255

        return ivs

    async def send_command(self, command, device):
        nonce = self.generate_nonce()
        data = self.generate_data(self.session_key, nonce, command)

        command = Command(
            data,
            command_characteristic_uuid,
            device
        )

        await command.write()

    def encrypt_data_chunk(self, session_key, r):
        encrypted = encrypt_data(session_key, r)
        p1 = 0
        p2 = len(encrypted)
        r = bytearray(b'\0' * len(encrypted))
        while True:
            p2 -= 1
            if p2 < 0:
                break

            r[p2] = encrypted[p1]
            p1 += 1

        return r

    def generate_data(self, session_key, nonce, command):
        r = bytearray(b'\0'*16)
        r[:8] = nonce[:]
        r[8] = 15

        r = self.encrypt_data_chunk(session_key, r)

        for i in range(15):
            r[i & 15] ^= command[i+5]

            if (i & 15) == 15 or i == 14:
                r = self.encrypt_data_chunk(session_key, r)

        for i in range(2):
            command[i+3] = r[i]

        r = bytearray(b'\0' * 16)
        r[1:8+1] = nonce[:]

        e = bytearray(b'\0'*16)
        for i in range(15):
            if (i & 15) == 0:
                e = self.encrypt_data_chunk(session_key, r)
                r[0] += 1

            command[i+5] ^= e[i & 15]

        return command


class BaseCommandAction:
    def __init__(self,
                 mac_address=None,
                 session_key=None,
                 no_response=False,
                 opcode=0,
                 mesh_address=0,
                 params=None,
                 vendor_id=0,
                 callback=None
    ):
        self.mac_address = mac_address
        self.session_key = session_key
        self.no_response = no_response
        self.opcode = opcode
        self.mesh_address = mesh_address
        self.params = params
        self.vendor_id = vendor_id
        self.callback = callback

    def build_command_action(self):
        return CommandAction(self)


class PacketParser:
    def __init__(self, data):
        self.progress = 0
        self.index = -1
        self.data = data

        if len(self.data) % 16 == 0:
            self.total = int(len(self.data) / 16)
        else:
            self.total = int(math.floor(len(self.data) / 16 + 1))

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
        size = 16

        if length > size:
            if index + 1 == self.total:
                packet_size = length - index * size
            else:
                packet_size = size
        else:
            packet_size = length

        packet_size += 4

        packet = [0]*20
        for i in range(2, len(packet)-2):
            packet[i] = 0xff

        packet[2:2+packet_size-4] = self.data[index*size:index*size+packet_size-4]

        self.fill_index(packet, index)
        crc = self.crc16(packet)
        self.fill_crc(packet, crc)

        packet = bytearray(packet)
        # print(f"ota packet ----> index: {index} total: {self.total} crc: {crc} content: {binascii.hexlify(packet, ':')}")

        return packet

    def fill_index(self, packet, index):
        packet[0] = index & 0xff
        packet[1] = (index >> 8) & 0xff

    def fill_crc(self, packet, crc):
        offset = len(packet) - 2
        packet[offset] = crc & 0xff
        packet[offset+1] = (crc >> 8) & 0xff

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


async def main():
    args_parser = argparse.ArgumentParser()
    args_parser.add_argument("--fw_file", help="Firmware file to flash", required=True, type=str)
    args_parser.add_argument("--mac", help="MAC address of device to flash", required=True, type=str)
    args_parser.add_argument('--mesh_name', help="The name of the mesh. eg `mymesh`. (Must be 16 bytes or fewer)", required=True, type=str)
    args_parser.add_argument('--mesh_password', help="The password of the mesh. eg `mymeshpwd` (Must be 16 bytes or fewer)", required=True, type=str)
    args_parser.add_argument("--force", required=False, default=False, action=argparse.BooleanOptionalAction)
    args = args_parser.parse_args()

    print(f"Trying to flash {args.fw_file} to device {args.mac}")

    print("Trying to connect to device...")
    async with BleakClient(args.mac) as client:
        # Check that we should bother updating the firmware
        with open(args.fw_file, 'rb') as f:
            parser = PacketParser(f.read())

        print("Reading firmware version from device...")
        dev_ver = await client.read_gatt_char(firmware_revision_uuid)

        dev_ver = struct.unpack("I", dev_ver[:4])[0]
        fw_ver = struct.unpack("I", parser.data[2:6])[0]

        dev_16mhz = (dev_ver >> 31) == 1
        dev_ver = dev_ver & 0x7fffffff

        fw_16mhz = (fw_ver >> 31) == 1
        fw_ver = fw_ver & 0x7fffffff

        print(f"Device version: {dev_ver}, new version: {fw_ver}")

        if dev_16mhz != fw_16mhz:
            print(f"Attempt to flash incompatible crystal version of firmware. Bailing out. Device is 16MHz? "
                  f"{dev_16mhz}, FW is 16MHz? {fw_16mhz}", file=sys.stderr)
            if not args.force:
                exit(1)

        if not args.force:
            if dev_ver >= fw_ver:
                print("Device firmware is newer or the same as the provided firmware file. Nothing to do.")
                exit(2)
        else:
            print("Forcing flash as requested")

        # Login to the device
        print("Trying login...")

        global mesh_name, mesh_pass
        mesh_name = args.mesh_name
        mesh_pass = args.mesh_password

        plaintext_mesh_credentials = encode_mesh_credentials()

        enc_key = bytearray(shared_key)[:] + b'\0' * 8
        encrypted = encrypt_data(enc_key, plaintext_mesh_credentials)

        command_data = bytearray(b'\0' * 17)
        command_data[0] = 12    # gatt_opcode.BLE_GATT_OP_PAIR_ENC_REQ
        command_data[1:1 + len(shared_key)] = shared_key[:]
        command_data[9:9 + 8] = encrypted[8:]

        reverse_section(command_data, 9, 16)

        await client.write_gatt_char(pair_characteristic_uuid, command_data)
        process_session_key(
            await client.read_gatt_char(pair_characteristic_uuid),
            plaintext_mesh_credentials
        )

        print("Login complete, writing device firmware...")

        await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())

        completion = int(parser.index / parser.total * 100.)
        while parser.has_next_packet():
            new_completion = int(parser.index / parser.total * 100.)
            if new_completion != completion:
                completion = new_completion
                print(f"{completion}%")
                await client.read_gatt_char(ota_characteristic_uuid)

            await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())

        # Write the final packet
        await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())
        await client.read_gatt_char(ota_characteristic_uuid)

        print("Firmware written.")

try:
    asyncio.run(main())
except OSError as e:
    pass
