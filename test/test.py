# SPDX-FileCopyrightText: © 2025 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from tqv import TinyQV

PERIPHERAL_NUM = 13

@cocotb.test()
async def test_neuro_nav_slam(dut):
    dut._log.info("Starting neuromorphic SLAM test")

    # 10 MHz clock
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    tqv = TinyQV(dut, PERIPHERAL_NUM)

    # Reset
    await tqv.reset()

    # Write spike packet: direction = east (2'b00), weight = 0x0020 ===
    spike = 0x00000020  # bits [17:16] = 00 (east), bits [15:0] = 0x0020
    await tqv.write_word_reg(0, spike)

    # Enable movement
    await tqv.write_word_reg(4, 0x00000001)

    # Wait for update
    await ClockCycles(dut.clk, 3)

    # Read position output
    pos = await tqv.read_word_reg(8)
    assert pos == 0x00000020, f"Expected pos_x=0x20, got {hex(pos)}"

    # Check output mapping (lower 4 bits of pos_y and pos_x)
    assert dut.uo_out.value == 0x20, f"Expected uo_out=0x20, got {hex(dut.uo_out.value)}"

    # Move north
    spike_north = (1 << 16) | 0x0030  # direction = 2'b01, weight = 0x0030
    await tqv.write_word_reg(0, spike_north)
    await ClockCycles(dut.clk, 3)

    pos = await tqv.read_word_reg(8)
    assert pos == 0x00300020, f"Expected pos_y=0x30, pos_x=0x20, got {hex(pos)}"

    # Trigger interrupt by exceeding threshold 
    spike_big = (0 << 16) | 0xFFFF  # move east by 0xFFFF
    await tqv.write_word_reg(0, spike_big)
    await ClockCycles(dut.clk, 3)

    assert await tqv.is_interrupt_asserted(), "Interrupt should be asserted"

    # Clear interrupt 
    await tqv.write_byte_reg(0x10, 0x01)
    await ClockCycles(dut.clk, 2)
    assert not await tqv.is_interrupt_asserted(), "Interrupt should be cleared"

    dut._log.info("Neuromorphic SLAM test completed successfully")