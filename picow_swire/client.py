import socket
import struct
import argparse
import time

HOST = ""  # The server's hostname or IP address
PORT = 8000  # The port used by the server


def write_addr(addr, data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = struct.pack('B', 0) + struct.pack('H', addr) + bytearray(data)
        s.sendall(data)
        assert s.recv(4096)[0] == 1


def read_addr(addr, len):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = struct.pack('B', 1) + struct.pack('H', addr) + struct.pack('H', len)
        s.sendall(data)
        result = s.recv(4096)
        return result


def read_flash(addr, len):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = struct.pack('B', 2) + struct.pack('I', addr) + struct.pack('H', len)
        s.sendall(data)
        result = s.recv(4096)
        return result


def write_flash(addr, data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = struct.pack('B', 3) + struct.pack('I', addr) + bytearray(data)
        s.sendall(data)
        assert s.recv(4096)[0] == 1


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

    CHUNK_SIZE = 256
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

            while True:
                write_flash(addr, list(data))
                res = read_flash(addr, len(data))
                if data == res:
                    break

                print(f"Error writing to 0x{addr:04x}, trying again...")

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
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        data = struct.pack('B', 4)
        s.sendall(data)


def main():
    args_parser = argparse.ArgumentParser(description='TLSR')
    args_parser.add_argument(
        '--debug', action="store_true", help="Enabled debugging information.")
    subparsers = args_parser.add_subparsers(dest="cmd", required=True)

    dump_flash_parser = subparsers.add_parser('dump_flash')
    dump_flash_parser.set_defaults(func=dump_flash_main)
    dump_flash_parser.add_argument('filename', type=str)

    dump_ram_parser = subparsers.add_parser('dump_ram')
    dump_ram_parser.set_defaults(func=dump_ram_main)
    dump_ram_parser.add_argument('filename', type=str)

    get_soc_id_parser = subparsers.add_parser('get_soc_id')
    get_soc_id_parser.set_defaults(func=get_soc_id_main)

    write_flash_parser = subparsers.add_parser('write_flash')
    write_flash_parser.set_defaults(func=write_flash_main)
    write_flash_parser.add_argument('filename', type=str)

    erase_flash_parser = subparsers.add_parser('erase_flash')
    erase_flash_parser.set_defaults(func=erase_flash_main)

    cpu_run_parser = subparsers.add_parser('cpu_run')
    cpu_run_parser.set_defaults(func=cpu_run_main)

    cpu_reset_parser = subparsers.add_parser('cpu_reset')
    cpu_reset_parser.set_defaults(func=cpu_reset_main)

    args = args_parser.parse_args()

    args.func(args)


if __name__ == "__main__":
    main()
