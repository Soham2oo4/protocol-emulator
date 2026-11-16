import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

# Control pins on uio_in
GO_WRITE = 2
SET_REG = 3
GO_READ = 4


def read_int(signal):
    """Read a signal as an int, treating unknown (X/Z) values as 0."""
    try:
        return int(signal.value)
    except ValueError:
        return 0


class I2CBus:
    """Simulates the two I2C wires with pull-up resistors, plus a fake
    EEPROM-style I2C target with 256 bytes of memory.

    Target behaviour (like a real 24Cxx EEPROM):
      write: START, addr+W, reg, data..., STOP  -> stores data at reg, reg+1, ...
      read:  START, addr+W, reg, START, addr+R, data..., NACK, STOP
    """

    def __init__(self, dut, target_addr=0x50, stretch_cycles=0):
        self.dut = dut
        self.target_addr = target_addr
        self.stretch_cycles = stretch_cycles

        self.ctrl = 0                    # control pin bits driven onto uio_in[2..4]
        self.mem = [0] * 256
        self.ptr = 0

        # What the target is doing to the wires
        self.t_sda_low = False
        self.t_scl_low = False
        self.stretch_left = 0

        # Target state machine
        self.state = "idle"
        self.shift = 0
        self.nbits = 0
        self.first_byte = True
        self.ptr_set = False
        self.read_mode = False
        self.after_ack = "recv"
        self.tx_byte = 0
        self.tx_idx = 0
        self.ctrl_acked = False
        self.prev_scl = 1
        self.prev_sda = 1

        # Observations checked by tests
        self.starts = 0
        self.stops = 0
        self.ctrl_acks = []              # ACK (True) / NACK (False) from controller after each byte we sent

    async def run(self):
        while True:
            await FallingEdge(self.dut.clk)

            if self.stretch_left > 0:
                self.stretch_left -= 1
                self.t_scl_low = self.stretch_left > 0

            oe = read_int(self.dut.uio_oe)
            scl = 0 if (oe & 1) or self.t_scl_low else 1
            sda = 0 if (oe & 2) or self.t_sda_low else 1
            self.dut.uio_in.value = scl | (sda << 1) | (self.ctrl & ~0b11)

            self._target(scl, sda)
            self.prev_scl, self.prev_sda = scl, sda

    def _put_bit(self):
        bit = (self.tx_byte >> self.tx_idx) & 1
        self.t_sda_low = bit == 0

    def _target(self, scl, sda):
        rising = self.prev_scl == 0 and scl == 1
        falling = self.prev_scl == 1 and scl == 0

        # START / repeated START / STOP: SDA changes while SCL stays high
        if scl == 1 and self.prev_scl == 1:
            if self.prev_sda == 1 and sda == 0:
                self.starts += 1
                self.state = "recv"
                self.shift = 0
                self.nbits = 0
                self.first_byte = True
                self.ptr_set = False
                self.t_sda_low = False
                return
            if self.prev_sda == 0 and sda == 1:
                self.stops += 1
                self.state = "idle"
                self.t_sda_low = False
                return

        if self.state == "recv":
            if rising:
                self.shift = ((self.shift << 1) | sda) & 0xFF
                self.nbits += 1
            elif falling and self.nbits == 8:
                byte = self.shift
                if self.first_byte:
                    ack = (byte >> 1) == self.target_addr
                    self.read_mode = bool(byte & 1)
                    self.first_byte = False
                    self.after_ack = "send" if self.read_mode else "recv"
                else:
                    ack = True
                    self.after_ack = "recv"
                    if not self.ptr_set:
                        self.ptr = byte
                        self.ptr_set = True
                    else:
                        self.mem[self.ptr] = byte
                        self.ptr = (self.ptr + 1) % 256
                if ack:
                    self.t_sda_low = True
                    self.state = "ack"
                    if self.stretch_cycles:
                        self.t_scl_low = True
                        self.stretch_left = self.stretch_cycles
                else:
                    self.state = "idle"

        elif self.state == "ack":
            if falling:                      # our ACK bit is over
                if self.after_ack == "send":
                    self.tx_byte = self.mem[self.ptr]
                    self.tx_idx = 7
                    self._put_bit()          # first data bit, MSB first
                    self.state = "send"
                else:
                    self.t_sda_low = False
                    self.state = "recv"
                    self.shift = 0
                    self.nbits = 0

        elif self.state == "send":
            if falling:                      # a data bit is over
                self.tx_idx -= 1
                if self.tx_idx < 0:
                    self.t_sda_low = False   # release SDA for the controller's ACK/NACK
                    self.state = "mack"
                else:
                    self._put_bit()

        elif self.state == "mack":
            if rising:
                self.ctrl_acked = sda == 0
                self.ctrl_acks.append(self.ctrl_acked)
                self.ptr = (self.ptr + 1) % 256
            elif falling:
                if self.ctrl_acked:          # controller wants another byte
                    self.tx_byte = self.mem[self.ptr]
                    self.tx_idx = 7
                    self._put_bit()
                    self.state = "send"
                else:
                    self.state = "idle"      # NACK: wait for STOP


async def setup(dut, **bus_kwargs):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    bus = I2CBus(dut, **bus_kwargs)

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0b011
    dut.rst_n.value = 0
    cocotb.start_soon(bus.run())

    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    return bus


async def pulse(dut, bus, pin):
    bus.ctrl |= 1 << pin
    await ClockCycles(dut.clk, 10)
    bus.ctrl &= ~(1 << pin)
    await ClockCycles(dut.clk, 10)


async def wait_done(dut, timeout=200_000):
    """Wait until done is set and busy is clear. Returns the nack bit."""
    for _ in range(timeout):
        status = read_int(dut.uio_out)
        busy, nack, done = (status >> 5) & 1, (status >> 6) & 1, (status >> 7) & 1
        if done and not busy:
            return nack
        await ClockCycles(dut.clk, 1)
    assert False, "transaction timed out"


async def i2c_write(dut, bus, reg, data):
    dut.ui_in.value = reg
    await pulse(dut, bus, SET_REG)
    dut.ui_in.value = data
    await pulse(dut, bus, GO_WRITE)
    return await wait_done(dut)


async def i2c_read(dut, bus, reg):
    dut.ui_in.value = reg
    await pulse(dut, bus, SET_REG)
    await pulse(dut, bus, GO_READ)
    nack = await wait_done(dut)
    return nack, read_int(dut.uo_out)


@cocotb.test()
async def test_write_then_read(dut):
    bus = await setup(dut)

    assert await i2c_write(dut, bus, 0x10, 0x5A) == 0
    assert bus.mem[0x10] == 0x5A, "EEPROM should hold the written byte"

    nack, data = await i2c_read(dut, bus, 0x10)
    assert nack == 0
    assert data == 0x5A, f"read back 0x{data:02X}"

    assert bus.ctrl_acks == [False], "controller must NACK the last byte of a read"
    assert bus.starts == 3, "write has 1 START, read has START + repeated START"
    assert bus.stops == 2


@cocotb.test()
async def test_read_preloaded_memory(dut):
    bus = await setup(dut)
    rng = random.Random(42)
    bus.mem = [rng.randrange(256) for _ in range(256)]

    for reg in [0x00, 0x01, 0x7F, 0x80, 0xFF]:
        nack, data = await i2c_read(dut, bus, reg)
        assert nack == 0
        assert data == bus.mem[reg], f"reg 0x{reg:02X}: got 0x{data:02X}, expected 0x{bus.mem[reg]:02X}"


@cocotb.test()
async def test_wrong_address_gets_nack(dut):
    bus = await setup(dut, target_addr=0x51)

    nack, _ = await i2c_read(dut, bus, 0x00)
    assert nack == 1, "nobody at 0x50, read should NACK"
    assert bus.stops == 1, "controller must still release the bus with STOP"

    assert await i2c_write(dut, bus, 0x00, 0xAA) == 1
    assert bus.mem[0x00] == 0, "nothing should be written"


@cocotb.test()
async def test_clock_stretching(dut):
    bus = await setup(dut, stretch_cycles=2000)

    assert await i2c_write(dut, bus, 0x20, 0xC3) == 0
    nack, data = await i2c_read(dut, bus, 0x20)
    assert nack == 0 and data == 0xC3


@cocotb.test()
async def test_random_write_readback(dut):
    bus = await setup(dut)
    rng = random.Random(1234)

    for _ in range(10):
        reg, val = rng.randrange(256), rng.randrange(256)
        assert await i2c_write(dut, bus, reg, val) == 0
        nack, data = await i2c_read(dut, bus, reg)
        assert nack == 0 and data == val, f"reg 0x{reg:02X}: wrote 0x{val:02X}, read 0x{data:02X}"