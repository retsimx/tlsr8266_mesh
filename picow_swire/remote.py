import binascii
import time

import micropython
import rp2
import utime
from machine import Pin, freq
from rp2 import PIO, StateMachine, asm_pio

# Make the pi go a bit quickr
freq(270000000)

# TX rates
# 122051000 : TYBT1
# 130289000 : E104-BT05-TB

SWS_MULTIPLIER = 3
RX_RATE = 122051000     # The receive speed (Change this based on TYBT1 or E104-BT05 values above)
TX_RATE = 50000000      # The transmission speed (This value is ok for both boards I'm working with)

rp2.PIO(0).remove_program()
rp2.PIO(1).remove_program()

reset_pin = Pin(16, Pin.OUT)
reset_pin.high()

input_pin = Pin(15, Pin.IN)

@asm_pio(
    autopush=True,
    push_thresh=8,
    in_shiftdir=rp2.PIO.SHIFT_LEFT
)
def rx_pio():
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
def tx_pio():
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
rx_sm = StateMachine(
    1,
    rx_pio,
    freq=RX_RATE,
    in_base=input_pin  # For WAIT, IN
)

tx_sm = StateMachine(
    0,
    tx_pio,
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
        # After sending the RW_ID byte, the master sends one unit of low level.
        rx_sm.restart()
        rx_sm.active(1)

        # Clear any data in the fifo
        while rx_sm.rx_fifo():
            rx_sm.get()

        # Trigger a low level
        tx_sm.put(0)

        now = time.ticks_ms()
        while not rx_sm.rx_fifo():
            if time.ticks_ms() - now > 100:
                send_byte(1, 0xff)
                raise Exception("Timeout waiting for data")

        value = rx_sm.get() & 0xff
        result[idx] = ~value & 0xff

        rx_sm.active(0)

    send_byte(1, 0xff)

    return result


def reset_mcu(reset_only=False):
    # Halt the CPU
    while True:
        if reset_only:
            reset_pin.low()
            time.sleep_ms(1)
            reset_pin.high()
            return

        reset_pin.low()
        time.sleep_ms(1)
        reset_pin.high()
        time.sleep_ms(1)

        write_addr(0x00b2, [SWS_MULTIPLIER])
        write_addr(0x0602, [0x05])

        try:
            while read_addr(0x00b2, 1)[0] != SWS_MULTIPLIER:
                reset_pin.low()
                time.sleep_ms(1)
                reset_pin.high()
                time.sleep_ms(1)

                write_addr(0x00b2, [SWS_MULTIPLIER])
                write_addr(0x0602, [0x05])

            break
        except:
            reset_pin.low()
            time.sleep_ms(1)
            reset_pin.high()
            time.sleep_ms(1)

            write_addr(0x00b2, [SWS_MULTIPLIER])
            write_addr(0x0602, [0x05])


print("About to reset MCU")

# reset_pin.low()
# time.sleep_ms(1)
# reset_pin.high()
# time.sleep_ms(1)
#
# write_addr(0x0602, [0x05])
# write_addr(0x00b2, [SWS_MULTIPLIER])
#
# while True:
#     write_addr(0x0602, [0x05])
#     write_addr(0x00b2, [SWS_MULTIPLIER])
#
#     errors = 0
#     for i in range(100):
#         value = b'\0'
#         try:
#             value = read_addr(0x007e, 2)
#             if value[0] != 0x25 or value[1] != 0x53:
#                 errors += 1
#         except:
#             errors += 1
#
#     print(binascii.hexlify(value), RX_RATE, errors)
#
#     if not errors:
#         break
#
#     rx_sm = StateMachine(
#         1,
#         rx_pio,
#         freq=RX_RATE,
#         in_base=input_pin  # For WAIT, IN
#     )
#
#     RX_RATE += 1000
#
# start_baud = RX_RATE
# print("Found start baud", start_baud)
#
# while True:
#     errors = 0
#     for i in range(100):
#         value = b'\0'
#         try:
#             value = read_addr(0x007e, 2)
#             if value[0] != 0x25 or value[1] != 0x53:
#                 errors += 1
#         except:
#             errors += 1
#
#     print(binascii.hexlify(value), RX_RATE, errors)
#
#     if errors:
#         break
#
#     rx_sm = StateMachine(
#         1,
#         rx_pio,
#         freq=RX_RATE,
#         in_base=input_pin  # For WAIT, IN
#     )
#
#     RX_RATE += 10000
#
# print("Found")
# print("Start baud", start_baud)
# print("End baud", RX_RATE)
# print("Recommended baud", int((start_baud+RX_RATE)/2))

# while True:
#     time.sleep(10)

reset_mcu()

print("MCU Configured, ready for instructions...")


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


@micropython.native
def send_flash_write_enable():
    # CNS low.
    write_addr(0x0d, [0x00])
    # Write enable.
    write_addr(0x0c, [0x06])
    # CNS high.
    write_addr(0x0d, [0x01])


@micropython.native
def write_flash_sector(addr, data):
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


@micropython.native
def write_flash(inaddr, indata):
    chunk_size = 256
    size = len(indata) + inaddr
    for addr in range(inaddr, size, chunk_size):
        data = list(indata[addr - inaddr:min(addr + chunk_size - inaddr, size - inaddr)])

        for count in range(5):
            write_flash_sector(addr, data)
            # Give the flash a moment to write the data and finish before we read
            utime.sleep_us(300)
            res = read_flash(addr, len(data))
            if data == res:
                break

            print(f"Error writing to 0x{addr:04x}, trying again...")

            if count == 4:
                raise Exception("Couldn't write sector")
