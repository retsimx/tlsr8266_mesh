import struct

import gc

import binascii
import machine
import network
import socket
import time
from machine import Pin
from rp2 import PIO, StateMachine, asm_pio
import rp2
import micropython

WIFI_SSID = ""
WIFI_PASS = ""

rp2.country('AU')
rp2.PIO(0).remove_program()
rp2.PIO(1).remove_program()

# UART_BAUD = 3187850       # This was the value when the chip was programmed with the original firmware
UART_BAUD = 2838900         # This is the value when the chip is erased
TX_RATE = 2500000

reset_pin = Pin(16, Pin.OUT)
reset_pin.high()

input_pin = Pin(15, Pin.IN)

@asm_pio(
    autopush=True,
    push_thresh=8,
    in_shiftdir=rp2.PIO.SHIFT_LEFT
)
def uart_rx_mini():
    # fmt: off
    # Wait for low to be sent from tx
    wait(0, pin, 0)
    # Wait for start bit
    wait(1, pin, 0)
    wait(0, pin, 0) [30]
    # Sample data
    set(x, 7)
    label("nextbit")
    in_(pins, 1)    [31]
    nop()           [30]
    jmp(x_dec, "nextbit")
    # fmt: on


@asm_pio(
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    set_init=PIO.OUT_HIGH,
)
def uart_tx_mini():
    # fmt: off
    pull(block)
    # Skip the first 22 bits
    out(x, 22)
    # Read the "Short low level" command
    out(x, 1)
    jmp(not_x, "finish")

    # Not a low level command, write the bits out
    set(y, 8)
    label("nextbit")
    # Read the next bit in to x
    out(x, 1)
    # If x is zero, jump to the zero label
    jmp(not_x, "zero")
    # Otherwise handle outputting the 1
    set(pins, 0) [19]
    set(pins, 1) [1]
    # If there is more data to send, do that
    jmp(y_dec, "nextbit")
    jmp("finish")

    # Handle sending low bit
    label("zero")
    set(pins, 0) [4]
    set(pins, 1) [16]
    # If there is more data to send, do that
    jmp(y_dec, "nextbit")

    label("finish")
    # One time unit of low level to signal the end of the message
    set(pins, 0) [4]
    set(pins, 1)
    # fmt: on

# Set up the state machine we're going to use to receive the characters.
sm = StateMachine(
    1,
    uart_rx_mini,
    freq=UART_BAUD,
    in_base=input_pin  # For WAIT, IN
)

tx_sm = StateMachine(
    0,
    uart_tx_mini,
    freq=TX_RATE,
    set_base=Pin(17)
)
tx_sm.active(1)


@micropython.native
def send_byte(cmd, byte):
    # Make sure this command isn't a "low level" command
    byte |= 1 << 9

    if cmd:
        byte |= 1 << 8

    tx_sm.put(byte)

    while tx_sm.tx_fifo():
        pass


@micropython.native
def write_addr(addr, data):
    send_byte(1, 0x5a)
    send_byte(0, (addr >> 8) & 0xff)
    send_byte(0, addr & 0xff)
    send_byte(0, 0)

    for i in range(len(data)):
        send_byte(0, data[i])

    send_byte(1, 0xff)


@micropython.native
def read_addr(addr, len):
    result = bytearray(len)

    send_byte(1, 0x5a)
    send_byte(0, (addr >> 8) & 0xff)
    send_byte(0, addr & 0xff)
    send_byte(0, 0x80)

    for idx in range(len):
        time.sleep_us(70)

        # After sending the RW_ID byte, the master sends one unit of low level.
        sm.restart()
        sm.active(1)

        # Wait for the sm to become active
        while not sm.active():
            pass

        # Clear any data in the fifo
        while sm.rx_fifo():
            sm.get()

        # Trigger a low level
        tx_sm.put(0)

        now = time.ticks_ms()
        while not sm.rx_fifo():
            if time.ticks_ms() - now > 100:
                send_byte(1, 0xff)
                raise Exception("Timeout waiting for data")

        value = sm.get() & 0xff
        result[idx] = ~value & 0xff

        sm.active(0)

    send_byte(1, 0xff)

    return result


def reset_mcu(reset_only=False):
    # Halt the CPU
    while True:
        reset_pin.low()
        time.sleep_ms(100)
        reset_pin.high()

        if reset_only:
            return

        try:
            while read_addr(0x00b2, 1)[0] != 127:
                write_addr(0x00b2, [127])
                write_addr(0x0602, [0x05])

            break
        except:
            pass


print("About to reset MCU")

# reset_pin.low()
# time.sleep_ms(100)
# reset_pin.high()
#
# write_addr(0x0602, [0x05])
# write_addr(0x00b2, [127])
#
# while True:
#     errors = 0
#     for i in range(100):
#         value = read_addr(0x007e, 2)
#         if value[0] != 0x25 or value[1] != 0x53:
#             errors += 1
#
#     print(binascii.hexlify(value), UART_BAUD, errors)
#
#     if not errors:
#         break
#
#     sm = StateMachine(
#         1,
#         uart_rx_mini,
#         freq=UART_BAUD,
#         in_base=input_pin  # For WAIT, IN
#     )
#
#     UART_BAUD += 100
#
# start_baud = UART_BAUD
# print("Found start baud", start_baud)
#
# while True:
#     errors = 0
#     for i in range(100):
#         value = read_addr(0x007e, 2)
#         if value[0] != 0x25 or value[1] != 0x53:
#             errors += 1
#
#     print(binascii.hexlify(read_addr(0x007e, 2)), UART_BAUD, errors)
#
#     if errors:
#         break
#
#     sm = StateMachine(
#         1,
#         uart_rx_mini,
#         freq=UART_BAUD,
#         in_base=input_pin  # For WAIT, IN
#     )
#
#     UART_BAUD += 100
#
# print("Found")
# print("Start baud", start_baud)
# print("End baud", UART_BAUD)
# print("Recommended baud", (start_baud+UART_BAUD)/2)
#
# while True:
#     pass

reset_mcu()

print("MCU Configured")

# Connect to WLAN
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# Set power mode to get WiFi power-saving off (if needed)
wlan.config(pm=0xa11140)

wlan.connect(WIFI_SSID, WIFI_PASS)

while not wlan.isconnected():
    print('Waiting for wifi connection...')
    time.sleep(1)

ip = wlan.ifconfig()[0]
print(f'WiFi connected with IP: {ip}')

print("Ready for instructions...")


@micropython.native
def read_flash(addr, chunk_size):
    contents = []
    # send_flash_write_enable()
    # CNS low.
    write_addr(0x0d, [0x00])
    # Read command.
    write_addr(0x0c, [0x03])
    write_addr(0x0c, [(addr >> 16) & 0xff])
    write_addr(0x0c, [(addr >> 8) & 0xff])
    write_addr(0x0c, [addr & 0xff])

    # FIFO mode.
    write_addr(0xb3, [0x80])

    for i in range(chunk_size):
        write_addr(0x0c, [0xff])
        res = read_addr(0x0c, 1)
        assert len(res) == 1
        contents.extend(res)

    # RAM mode.
    write_addr(0xb3, [0x00])

    # CNS high.
    write_addr(0x0d, [0x01])
    return contents


def send_flash_write_enable():
    # CNS low.
    write_addr(0x0d, [0x00])
    # Write enable.
    write_addr(0x0c, [0x06])
    # CNS high.
    write_addr(0x0d, [0x01])


def write_flash(addr, data):
    send_flash_write_enable()

    # CNS low.
    write_addr(0x0d, [0x00])

    # Write command.
    write_addr(0x0c, [0x02])
    write_addr(0x0c, [(addr >> 16) & 0xff])
    write_addr(0x0c, [(addr >> 8) & 0xff])
    write_addr(0x0c, [addr & 0xff])

    # FIFO mode.
    write_addr(0xb3, [0x80])

    # Write data
    # CPU stop?
    write_addr(0x0c, data)

    # # RAM mode.
    write_addr(0xb3, [0x00])

    # CNS high.
    write_addr(0x0d, [0x01])


s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 8000))
s.listen(5)
while True:
    conn, addr = s.accept()

    data = conn.recv(4096)

    if data[0] == 1:
        raddr = struct.unpack('H', data[1:3])[0]
        data_len = struct.unpack('H', data[3:5])[0]
        try:
            result = read_addr(raddr, data_len)
            conn.send(result)
        except:
            pass
    elif data[0] == 0:
        waddr = struct.unpack('H', data[1:3])[0]
        data = data[3:]
        write_addr(waddr, data)
        conn.send(bytearray([1]))
    elif data[0] == 2:
        raddr = struct.unpack('I', data[1:5])[0]
        chunk_size = struct.unpack('H', data[5:7])[0]
        while True:
            try:
                result = read_flash(raddr, chunk_size)
                conn.send(bytearray(result))
                break
            except:
                print("Read error reading flash from", raddr)
                reset_mcu()
    elif data[0] == 3:
        waddr = struct.unpack('I', data[1:5])[0]
        data = data[5:]
        while True:
            try:
                write_flash(waddr, data)
                conn.send(bytearray([1]))
                break
            except:
                print("Write error writing flash to", waddr)
                reset_mcu()
    elif data[0] == 4:
        reset_mcu(True)
    elif data[0] == 5:
        reset_mcu(False)
    else:
        print("Unknown command")

    conn.close()
    gc.collect()
