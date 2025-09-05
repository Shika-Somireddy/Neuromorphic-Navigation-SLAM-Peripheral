# SPDX-FileCopyrightText: © 2025 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from tqv import TinyQV

# When submitting your design, change this to the peripheral number
# in peripherals.v.  e.g. if your design is i_user_peri05, set this to 5.
# The peripheral number is not used by the test harness.
PERIPHERAL_NUM = 19

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start SLAM Peripheral Test")

    # Set the clock period to 100 ns (10 MHz)
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Interact with your design's registers through this TinyQV class.
    # This will allow the same test to be run when your design is integrated
    # with TinyQV - the implementation of this class will be replaces with a
    # different version that uses Risc-V instructions instead of the SPI test
    # harness interface to read and write the registers.
    tqv = TinyQV(dut, PERIPHERAL_NUM)

    #reset
    await tqv.reset()

    dut._log.info("Testing SLAM movement logic")

    # Enable updates (write control_flags[0] = 1 at address 4)
    await tqv.write_word_reg(4, 0x1)

    # Move east by 5 (direction=0, step=5)
    await tqv.write_word_reg(0, 0x0005)
    await ClockCycles(dut.clk, 2)

    # Read position (address 8) -> X=5, Y=0
    pos = await tqv.read_word_reg(8)
    x = pos & 0xFFFF
    y = (pos >> 16) & 0xFFFF
    assert x == 5 and y == 0, f"Expected X=5, Y=0, got X={x}, Y={y}"

    # Move north by 3 (direction=1, step=3)
    await tqv.write_word_reg(0, (1 << 16) | 0x0003)
    await ClockCycles(dut.clk, 2)

    pos = await tqv.read_word_reg(8)
    x = pos & 0xFFFF
    y = (pos >> 16) & 0xFFFF
    assert x == 5 and y == 3, f"Expected X=5, Y=3, got X={x}, Y={y}"

    # Check uo_out reflects lower 4 bits of X and Y
    assert dut.uo_out.value == ((y & 0xF) << 4 | (x & 0xF))

    # Move east until X > 1000 to trigger interrupt
    await tqv.write_word_reg(0, 0x0400)  # step=1024, dir=0
    await ClockCycles(dut.clk, 2)

    assert await tqv.is_interrupt_asserted(), "Interrupt not triggered when X>1000"

    # Clear interrupt by writing 1 to address 16
    await tqv.write_word_reg(16, 0x1)
    await ClockCycles(dut.clk, 1)
    assert not await tqv.is_interrupt_asserted(), "Interrupt not cleared"

    dut._log.info("SLAM peripheral test completed successfully")
