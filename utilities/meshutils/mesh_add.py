import argparse
import asyncio
import binascii
import os
import random
from asyncio import sleep

from Cryptodome.Cipher import AES
from Cryptodome.Random import get_random_bytes
from bleak import BleakClient, BleakScanner

mesh_name = "out_of_mesh"
mesh_pass = "123"
shared_key = get_random_bytes(8)

pair_characteristic_uuid = "00010203-0405-0607-0809-0a0b0c0d1914"
notify_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1911'
command_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1912'


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


async def main():
    args_parser = argparse.ArgumentParser(description='TLSR')
    args_parser.add_argument('--mesh_address', help="The address of the new node in the mesh. Should be a unique (in the mesh) value between 1 and 63", required=True, type=int)
    args_parser.add_argument('--mesh_name', help="The name of the mesh. eg `mymesh`. (Must be 16 bytes or fewer)", required=True, type=str)
    args_parser.add_argument('--mesh_password', help="The password of the mesh. eg `mymeshpwd` (Must be 16 bytes or fewer)", required=True, type=str)

    args = args_parser.parse_args()

    print("Cleaning up existing connected devices...")
    print(os.system("/bin/bash ./scripts/device_cleanup.sh"))

    # Find the device
    device = await BleakScanner.find_device_by_name(name=mesh_name, timeout=5)

    if not device:
        print("Unable to find any devices")
        return

    print("Found device, connecting...")
    async with BleakClient(device.address) as client:
        # Login to the device
        print("Trying login...")

        plaintext_mesh_credentials = encode_mesh_credentials()

        enc_key = bytearray(shared_key)[:] + b'\0' * 8
        encrypted = encrypt_data(enc_key, plaintext_mesh_credentials)

        command_data = bytearray(b'\0' * 17)
        command_data[0] = 12    # gatt_opcode.BLE_GATT_OP_PAIR_ENC_REQ
        command_data[1:1 + len(shared_key)] = shared_key[:]
        command_data[9:9 + 8] = encrypted[8:]

        reverse_section(command_data, 9, 16)

        await client.write_gatt_char(pair_characteristic_uuid, command_data)
        session_key = process_session_key(
            await client.read_gatt_char(pair_characteristic_uuid),
            plaintext_mesh_credentials
        )

        print("Login complete, writing mesh details...")

        mesh_address = args.mesh_address
        print("New mesh address:", mesh_address)

        mac_bytes = bytearray(binascii.unhexlify(device.address.replace(':', '')))
        mac_bytes.reverse()

        print("Commencing pairing...")
        action = BaseCommandAction(
            mac_address=mac_bytes,
            opcode=0xe0,     # gatt_opcode.BLE_GATT_OP_CTRL_E0,
            params=[mesh_address & 0xff, (mesh_address >> 8) & 0xff],
            mesh_address=0,
            session_key=session_key,
            vendor_id=int("1102", 16),
            no_response=True
        ).build_command_action()

        await action.encode_and_send(client)

        await sleep(4)

        name = args.mesh_name.encode()
        name = bytearray(name) + bytearray([0] * (16 - len(name)))
        name = encrypt_data(session_key, name)
        name.reverse()

        pwd = args.mesh_password.encode()
        pwd = bytearray(pwd) + bytearray([0] * (16 - len(pwd)))
        pwd = encrypt_data(session_key, pwd)
        pwd.reverse()

        ltk = encrypt_data(session_key,
                           [0x8b, 0x34, 0x4e, 0x8e, 0x5e, 0x0c, 0x1f, 0xf6, 0x20, 0x36, 0x3e, 0xfd, 0x6e, 0x97, 0x47,
                            0xce])
        ltk.reverse()

        name = bytearray([4]) + name
        pwd = bytearray([5]) + pwd
        ltk = bytearray([6]) + ltk + bytearray([0x01])

        name_command = Command(name, pair_characteristic_uuid, client)
        pwd_command = Command(pwd, pair_characteristic_uuid, client)
        ltk_command = Command(ltk, pair_characteristic_uuid, client)

        await name_command.write()
        await sleep(0.2)
        await pwd_command.write()
        await sleep(0.2)
        await ltk_command.write()
        await sleep(0.2)

        result = await client.read_gatt_char(pair_characteristic_uuid)

        if result[0] != 0x7:
            print("Light could not be added to mesh")
            return

        print(f"Light with MAC ({device.address}) added to mesh ok")

        def on_message(*args, **kwargs):
            print(args, kwargs)

        await client.write_gatt_char(notify_characteristic_uuid, [1])
        await client.start_notify(notify_characteristic_uuid, on_message)

        await asyncio.sleep(120)


asyncio.run(main())
