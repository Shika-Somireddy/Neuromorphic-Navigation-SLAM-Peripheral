/* Copyright (c) 2025 Rishika Somireddy
* SPDX-License-Identifier: Apache-2.0 
*/

`default_nettype none

module tqvp_neuro_nav_SLAM (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [7:0]  ui_in,
    output logic [7:0]  uo_out,
    input  logic [5:0]  address,
    input  logic [31:0] data_in,
    input  logic [1:0]  data_write_n, // Byte strobes for 32-bit data (active low)
    input  logic         data_read_n,  // Single read strobe (active low)
    output logic [31:0] data_out,
    output logic        data_ready,
    output logic        user_interrupt
);

    // === Core Registers ===
    logic [31:0] spike_packet;      // {dir[1:0], weight[15:0]}
    logic [31:0] control_flags;     // enable/reset/etc.
    logic [15:0] pos_x, pos_y;      // current position
    logic [31:0] result_output;     // {Y, X}
    logic         slam_interrupt;    // Interrupt status register

    // --- Read/Write Strobe Generation ---
    logic write_strobe;
    logic read_strobe;

    assign write_strobe = !(|data_write_n); // Assert when all byte strobes are low
    assign read_strobe  = !data_read_n;     // Assert when read strobe is low

    // === Write Logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_packet  <= 32'h0;
            control_flags <= 32'h0;
        end else if (write_strobe) begin
            case (address)
                6'h0:  spike_packet  <= data_in;
                6'h4:  control_flags <= data_in;
                6'h8:  // pos_x/y registers are read-only
                6'hC: begin
                    // Write-to-clear interrupt
                    if (data_in[0]) 
                        slam_interrupt <= 1'b0;
                end
            endcase
        end
    end

    // === Position Update ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_x <= 16'h0;
            pos_y <= 16'h0;
        end else if (control_flags[0]) begin
            case (spike_packet[17:16]) // direction
                2'd0: pos_x <= pos_x + spike_packet[15:0]; // east
                2'd1: pos_y <= pos_y + spike_packet[15:0]; // north
                2'd2: pos_x <= pos_x - spike_packet[15:0]; // west
                2'd3: pos_y <= pos_y - spike_packet[15:0]; // south
            endcase
        end
    end

    assign result_output = {pos_y, pos_x};

    // === Readback Logic ===
    always_comb begin
        if (read_strobe) begin
            case (address)
                6'h0: data_out = spike_packet;
                6'h4: data_out = control_flags;
                6'h8: data_out = result_output;
                6'hC: data_out = {31'h0, slam_interrupt};
                default: data_out = 32'h0;
            endcase
        end else begin
            data_out = 32'hz; // High impedance during no read
        end
    end

    // === Data Ready Handshake ===
    assign data_ready = read_strobe;

    // === Output Mapping ===
    assign uo_out = {pos_y[3:0], pos_x[3:0]};

    // === Interrupt Logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slam_interrupt <= 1'b0;
        end else if ((pos_x > 16'd1000) || (pos_y > 16'd1000)) begin
            slam_interrupt <= 1'b1;
        end
    end

    assign user_interrupt = slam_interrupt;

    // === Prevent Unused Signal Warnings ===
    // verilator lint_off UNUSED
    wire _unused_ui_in;
    assign _unused_ui_in = ui_in[0];
    // verilator lint_on UNUSED

endmodule