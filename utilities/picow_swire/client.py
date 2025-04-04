import argparse
import struct
import time

import serial

serial_connection = None


def send(data):
    serial_connection.write(b"\x05A\x01")
    if serial_connection.read(2) != b"R\x01":
        raise Exception("Couldn't enter raw paste REPL, perhaps reset the rpi")

    window_size_increment = struct.unpack("H", serial_connection.read(2))[0]

    while len(data):
        segment = data[0:max(window_size_increment, len(data))]
        data = data[len(segment):]
        serial_connection.write(segment.encode())

        if len(data):
            time.sleep(0.1)
            if serial_connection.read(1) != b"\x01":
                raise Exception("Couldn't transmit entire data? This is unusual")

        else:
            serial_connection.write(b"\x04")

    while serial_connection.read(1) != b"\x04":
        pass

    result = b''
    while True:
        c = serial_connection.read(1)
        if c == b'\x04':
            break

        result += c

    exc = b''
    while True:
        c = serial_connection.read(1)
        if c == b'\x04':
            break

        exc += c

    if serial_connection.read(1) != b">":
        raise Exception("Didn't exit raw paste repl correctly. Don't know why.")

    return result, exc


def write_addr(addr, data):
    result, exc = send(f"print(repr(write_addr({addr}, {repr(list(data))})))")
    assert not len(exc)


def read_addr(addr, length):
    result, exc = send(f"print(repr(read_addr({addr}, {length})))")
    assert not len(exc)

    result = list(eval(result.decode().strip()))
    return result


def read_flash(addr, length):
    result, exc = send(f"print(repr(read_flash({addr}, {length})))")
    assert not len(exc)

    result = list(eval(result.decode().strip()))
    return result


def write_flash(addr, data):
    result, exc = send(f"print(repr(write_flash({addr}, {repr(list(data))})))")
    assert not len(exc)


RAM_SIZE = 0x010000
FLASH_SIZE = 0x7d000


def hexdump(bytearr):
    return ' '.join(f'{b:02x}' for b in bytearr)


def make_read_request(addr, n_bytes):
    result = read_addr(addr, n_bytes)
    if len(result) != n_bytes:
        raise Exception("Invalid read")

    return result


def make_write_request(addr, data):
    write_addr(addr, data)


def get_soc_id():
    res = make_read_request(0x007e, 2)
    return res[1] << 8 | res[0]


def send_flash_write_enable():
    # CNS low.
    make_write_request(0x0d, [0x00])
    # Write enable.
    make_write_request(0x0c, [0x06])
    # CNS high.
    make_write_request(0x0d, [0x01])


def send_flash_erase():
    # CNS low.
    make_write_request(0x0d, [0x00])
    # Write enable.
    make_write_request(0x0c, [0x60])
    # CNS high.
    make_write_request(0x0d, [0x01])


def send_flash_get_status():
    # CNS low.
    make_write_request(0x0d, [0x00])
    # Get flash status command.
    make_write_request(0x0c, [0x05])
    # Start SPI.
    make_write_request(0x0c, [0xff])
    # Read the status byte.
    res = make_read_request(0x0c, 1)
    # CNS high.
    make_write_request(0x0d, [0x01])
    return res


def send_cpu_stop():
    return make_write_request(0x0602, [0x05])


def send_csn_high():
    return make_write_request(0x000d, [0x01])


def send_csn_low():
    return make_write_request(0x000d, [0x00])


def dump_ram():
    contents = []
    CHUNK_SIZE = 32
    for addr in range(0x00, RAM_SIZE, CHUNK_SIZE):
        # Report progress.
        if addr & 0xff == 0:
            print(f'0x{addr:04x} {100 * addr / RAM_SIZE:05.2f}%')
        contents.extend(
            make_read_request(addr, CHUNK_SIZE))
    return contents


def write_to_file(filename, contents):
    print(f"Writing {len(contents)} bytes to {filename}")
    with open(filename, 'wb') as f:
        f.write(bytes(contents))


def dump_flash_main(args):
    print(f'Dumping flash to {args.filename}...')
    contents = []
    CHUNK_SIZE = 512
    for addr in range(0x00, FLASH_SIZE, CHUNK_SIZE):
        # Report progress.
        if addr & 0xff == 0:
            print(f'0x{addr:06x} {100 * addr / FLASH_SIZE:05.2f}%')
        # Retry the same address in case something goes wrong.
        while True:
            try:
                r1 = read_flash(addr, CHUNK_SIZE)
                res = read_flash(addr, CHUNK_SIZE)
                if r1 != res:
                    raise

                if args.debug:
                    print(f'Read: {hexdump(res)}')
                contents.extend(res)
                write_to_file(args.filename, contents)
                break
            except Exception as e:
                print(f"Retrying 0x{addr:08x}... {e}")

    write_to_file(args.filename, contents)


def erase_flash_main(args):
    print(f'Erasing flash...')
    send_flash_write_enable()
    send_flash_erase()
    while True:
        res = send_flash_get_status()
        print(f'Flash status: {hexdump(res)}')
        if res[0] == 0:
            break
        time.sleep(1)

    # CNS high.
    # make_write_request(0x0d, [0x01]))


def write_flash_main(args):
    print(f'Erasing flash...')
    send_flash_write_enable()
    send_flash_erase()
    while True:
        res = send_flash_get_status()
        print(f'Flash status: {hexdump(res)}')
        if res[0] == 0:
            break
        time.sleep(1)

    print(f'Writing flash from {args.filename}...')

    CHUNK_SIZE = 1024*4
    with open(args.filename, 'rb') as f:
        contents = f.read()
        size = len(contents)
        for addr in range(0x00, size, CHUNK_SIZE):
            # Report progress.
            if addr & 0xff == 0:
                print(f'0x{addr:04x} {100 * addr / size:05.2f}%')
            data = contents[addr:min(addr + CHUNK_SIZE, size)]
            if args.debug:
                print(f'writing: {hexdump(data)}')

            write_flash(addr, list(data))

    while True:
        res = send_flash_get_status()
        print(f'Flash status: {hexdump(res)}')
        if res[0] == 0:
            break
        time.sleep(1)


def dump_ram_main(args):
    print(f'Dumping ram to {args.filename}...')
    write_to_file(args.filename, dump_ram())


def get_soc_id_main(args):
    print(f'SOC ID: 0x{get_soc_id():04x}')


def cpu_run_main(args):
    # Tell CPU to run.
    make_write_request(0x0602, [0x88])


def cpu_reset_main(args):
    send("reset_mcu(True)")


def flashing_reset_main(args):
    send("reset_mcu(False)")


def write_remote(device):
    global serial_connection
    serial_connection = serial.Serial(device, 114200, timeout=1)

    serial_connection.read_all()
    serial_connection.write(b"\x01")
    time.sleep(0.1)
    serial_connection.read_all()

    with open('remote.py', 'r') as f:
        print(send(f.read()))


def main():
    args_parser = argparse.ArgumentParser(description='TLSR')
    args_parser.add_argument(
        '--debug', action="store_true", help="Enabled debugging information.")
    args_parser.add_argument(
        '--device', help="Path to Raspberry Pi Pico Serial Device", default="/dev/ttyACM0")
    args_parser.add_argument(
        '--xtal-speed', help="Remote device external crystal speed in MHz", default="12")
    subparsers = args_parser.add_subparsers(dest="cmd", required=True)

    dump_flash_parser = subparsers.add_parser('dump_flash')
    dump_flash_parser.set_defaults(func=dump_flash_main)
    dump_flash_parser.add_argument('filename', type=str, help="Dump chip firmware.")

    dump_ram_parser = subparsers.add_parser('dump_ram', help="Dump chip ram.")
    dump_ram_parser.set_defaults(func=dump_ram_main)
    dump_ram_parser.add_argument('filename', type=str)

    get_soc_id_parser = subparsers.add_parser('get_soc_id', help="Get the SoC chip ID.")
    get_soc_id_parser.set_defaults(func=get_soc_id_main)

    write_flash_parser = subparsers.add_parser('write_flash', help="Write firmware to chip.")
    write_flash_parser.set_defaults(func=write_flash_main)
    write_flash_parser.add_argument('filename', type=str)

    erase_flash_parser = subparsers.add_parser('erase_flash', help="Erase chip flash.")
    erase_flash_parser.set_defaults(func=erase_flash_main)

    cpu_run_parser = subparsers.add_parser('cpu_run', help="Resumes the CPU if it's in a paused state.")
    cpu_run_parser.set_defaults(func=cpu_run_main)

    cpu_reset_parser = subparsers.add_parser('cpu_reset', help="Resets the entire CPU via the reset pin.")
    cpu_reset_parser.set_defaults(func=cpu_reset_main)

    flashing_reset_parser = subparsers.add_parser('flashing_reset', help="Resets the entire CPU via the reset pin, renegotiates the SWire interface, then pauses the CPU ready for flashing.")
    flashing_reset_parser.set_defaults(func=flashing_reset_main)

    args = args_parser.parse_args()

    write_remote(args.device)

    args.func(args)


if __name__ == "__main__":
    main()
