# SPDX-FileCopyrightText: © 2025 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from tqv import TinyQV

PERIPHERAL_NUM = 13

async def dump_state(dut, tqv, label=""):
    """Helper to log internal register + outputs."""
    raw0 = await tqv.read_word_reg(0)   # sensor_input
    pos  = await tqv.read_word_reg(8)   # pos_x/pos_y packed
    x = pos & 0xFFFF
    y = (pos >> 16) & 0xFFFF
    dut._log.info(f"[{label}] sensor_input=0x{raw0:08X}, pos_x={x} (0x{x:04X}), pos_y={y} (0x{y:04X}), uo_out=0x{int(dut.uo_out.value):02X}")
    return raw0, x, y


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start SLAM Peripheral Test (extended debug)")

    # Clock
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    tqv = TinyQV(dut, PERIPHERAL_NUM)

    # Reset
    await tqv.reset()
    dut._log.info("Reset done")

    # Enable updates
    await tqv.write_word_reg(4, 0x1)
    dut._log.info("Enabled control_flags[0] = 1 at addr 4")

    # === Test 1: Move east by 5 ===
    await tqv.write_word_reg(0, 0x0005)   # dir=0, step=5
    await ClockCycles(dut.clk, 4)
    raw0, x, y = await dump_state(dut, tqv, "Move East by 5")
    assert x == 5 and y == 0, f"Expected X=5, Y=0, got X={x}, Y={y} (raw0=0x{raw0:08X})"

    # === Test 2: Move north by 3 ===
    await tqv.write_word_reg(0, (1 << 16) | 0x0003)   # dir=1, step=3
    await ClockCycles(dut.clk, 4)
    raw0, x, y = await dump_state(dut, tqv, "Move North by 3")
    assert x == 5 and y == 3, f"Expected X=5, Y=3, got X={x}, Y={y} (raw0=0x{raw0:08X})"

    # === Test 3: Check uo_out lower bits ===
    expect_uo = ((y & 0xF) << 4) | (x & 0xF)
    got_uo = int(dut.uo_out.value)
    dut._log.info(f"[uo_out check] expect=0x{expect_uo:02X}, got=0x{got_uo:02X}")
    assert got_uo == expect_uo

    # === Test 4: Trigger interrupt with big X step ===
    await tqv.write_word_reg(0, 0x0400)   # step=1024, dir=0
    await ClockCycles(dut.clk, 4)
    raw0, x, y = await dump_state(dut, tqv, "Trigger interrupt (X>1000)")
    assert await tqv.is_interrupt_asserted(), "Interrupt not triggered when X>1000"

    # === Test 5: Clear interrupt ===
    await tqv.write_word_reg(16, 0x1)
    await ClockCycles(dut.clk, 2)
    dut._log.info("[Interrupt clear] user_interrupt=%d" % int(dut.user_interrupt.value))
    assert not await tqv.is_interrupt_asserted(), "Interrupt not cleared"

    dut._log.info("SLAM peripheral test completed successfully")
