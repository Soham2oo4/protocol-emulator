import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge


def read_int(signal):
    """Read a signal as an int, treating unknown (X/Z) values as 0."""
    try:
        return int(signal.value)
    except ValueError:
        return 0


class I2CBus:
    """Simulates the two I2C wires with pull-up resistors, plus a fake
    I2C target device that listens for its address and ACKs bytes."""

    def __init__(self, dut, target_addr=0x50, stretch_cycles=0):
        self.dut = dut
        self.target_addr = target_addr
        self.stretch_cycles = stretch_cycles

        self.go = 0                 # drives uio_in[2]

        # What the target is doing to the wires
        self.t_sda_low = False
        self.t_scl_low = False
        self.stretch_left = 0

        # Target state machine
        self.state = "idle"
        self.shift = 0
        self.nbits = 0
        self.first_byte = True
        self.prev_scl = 1
        self.prev_sda = 1

        # What the target observed (checked by the tests)
        self.addr_seen = []
        self.received = []
        self.starts = 0
        self.stops = 0

    async def run(self):
        while True:
            await FallingEdge(self.dut.clk)

            if self.stretch_left > 0:
                self.stretch_left -= 1
                self.t_scl_low = self.stretch_left > 0

            # Wired-AND: a line is low if anyone pulls it low, else the pull-up wins
            oe = read_int(self.dut.uio_oe)
            scl = 0 if (oe & 1) or self.t_scl_low else 1
            sda = 0 if (oe & 2) or self.t_sda_low else 1
            self.dut.uio_in.value = scl | (sda << 1) | (self.go << 2)

            self._target(scl, sda)
            self.prev_scl, self.prev_sda = scl, sda

    def _target(self, scl, sda):
        rising = self.prev_scl == 0 and scl == 1
        falling = self.prev_scl == 1 and scl == 0

        # START / STOP: SDA changes while SCL stays high
        if scl == 1 and self.prev_scl == 1:
            if self.prev_sda == 1 and sda == 0:
                self.starts += 1
                self.state = "recv"
                self.shift = 0
                self.nbits = 0
                self.first_byte = True
                self.t_sda_low = False
                return
            if self.prev_sda == 0 and sda == 1:
                self.stops += 1
                self.state = "idle"
                self.t_sda_low = False
                return

        if self.state == "recv":
            if rising:                         # sample data on SCL rising edge
                self.shift = (self.shift << 1) | sda
                self.nbits += 1
            elif falling and self.nbits == 8:  # byte complete: decide ACK
                if self.first_byte:
                    self.addr_seen.append(self.shift)
                    ack = (self.shift >> 1) == self.target_addr and (self.shift & 1) == 0
                    self.first_byte = False
                else:
                    self.received.append(self.shift)
                    ack = True
                if ack:
                    self.t_sda_low = True
                    self.state = "ack"
                    if self.stretch_cycles:
                        self.t_scl_low = True
                        self.stretch_left = self.stretch_cycles
                else:
                    self.state = "idle"        # NACK: just leave SDA released

        elif self.state == "ack":
            if falling:                        # ACK bit finished: release SDA
                self.t_sda_low = False
                self.state = "recv"
                self.shift = 0
                self.nbits = 0


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


async def write_byte(dut, bus, byte, timeout=100_000):
    """Trigger one I2C write and wait until 'done' is set and 'busy' is clear.
    Returns the nack bit."""
    dut.ui_in.value = byte
    bus.go = 1
    await ClockCycles(dut.clk, 10)
    bus.go = 0

    for _ in range(timeout):
        status = read_int(dut.uo_out)
        busy, nack, done = status & 1, (status >> 1) & 1, (status >> 2) & 1
        if done and not busy:
            return nack
        await ClockCycles(dut.clk, 1)
    assert False, "transaction timed out"


@cocotb.test()
async def test_write_one_byte(dut):
    bus = await setup(dut)
    nack = await write_byte(dut, bus, 0xA5)

    assert nack == 0, "target should ACK"
    assert bus.addr_seen == [0xA0], f"address byte was {bus.addr_seen}"
    assert bus.received == [0xA5], f"target received {bus.received}"
    assert bus.starts == 1 and bus.stops == 1


@cocotb.test()
async def test_wrong_address_gets_nack(dut):
    bus = await setup(dut, target_addr=0x51)   # nobody at 0x50
    nack = await write_byte(dut, bus, 0x3C)

    assert nack == 1, "controller should see NACK"
    assert bus.received == [], "no data should be sent after a NACK"
    assert bus.stops == 1, "controller should still send STOP"


@cocotb.test()
async def test_clock_stretching(dut):
    bus = await setup(dut, stretch_cycles=2000)  # target holds SCL low after each ACK
    nack = await write_byte(dut, bus, 0x5A)

    assert nack == 0
    assert bus.received == [0x5A]


@cocotb.test()
async def test_random_bytes(dut):
    bus = await setup(dut)
    rng = random.Random(1234)
    sent = [rng.randrange(256) for _ in range(20)]

    for b in sent:
        assert await write_byte(dut, bus, b) == 0

    assert bus.received == sent
    assert bus.starts == 20 and bus.stops == 20
