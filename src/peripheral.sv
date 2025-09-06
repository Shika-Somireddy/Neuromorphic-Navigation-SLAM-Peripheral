/* Copyright (c) 2025 Rishika Somireddy
* SPDX-License-Identifier: Apache-2.0 
*/

`default_nettype none

module tqvp_neuro_nav_SLAM (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [7:0]  ui_in,
    output logic [7:0]  uo_out,
    input  logic [5:0]  address,
    input  logic [31:0] data_in,
    input  logic [1:0]  data_write_n,
    input  logic [1:0]  data_read_n,
    output logic [31:0] data_out,
    output logic        data_ready,
    output logic        user_interrupt
);

    // === Neuromorphic Register File ===
    logic [31:0] spike_packet;       // Encoded spike: {timestamp, neuron_id}
    logic [31:0] control_flags;      // enable/reset/etc.
    logic [15:0] neuron_pos_x [0:15]; // 16 place cells for X
    logic [15:0] neuron_pos_y [0:15]; // 16 place cells for Y
    logic [31:0] result_output;      // Encoded position from active neurons

    // === Spike Decoder ===
    logic [3:0] active_neuron_id;
    logic [15:0] spike_weight;

    always_comb begin
        active_neuron_id = spike_packet[19:16]; // neuron ID
        spike_weight     = spike_packet[15:0];  // movement magnitude
    end

    // === Write Logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_packet   <= 32'h0;
            control_flags  <= 32'h0;
        end else begin
            unique case (address)
                6'h0: if (data_write_n != 2'b11) spike_packet  <= data_in;
                6'h4: if (data_write_n != 2'b11) control_flags <= data_in;
                default: ;
            endcase
        end
    end

    // === Neuromorphic SLAM Core ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i++) begin
                neuron_pos_x[i] <= 16'h0;
                neuron_pos_y[i] <= 16'h0;
            end
            result_output <= 32'h0;
        end else if (control_flags[0]) begin
            // Hebbian-like update: reinforce active neuron
            neuron_pos_x[active_neuron_id] <= neuron_pos_x[active_neuron_id] + spike_weight;
            neuron_pos_y[active_neuron_id] <= neuron_pos_y[active_neuron_id] + spike_weight;

            // Aggregate position from active neurons (simplified)
            result_output <= {neuron_pos_y[active_neuron_id], neuron_pos_x[active_neuron_id]};
        end
    end

    // === Readback ===
    always_comb begin
        unique case (address)
            6'h0: data_out = spike_packet;
            6'h4: data_out = control_flags;
            6'h8: data_out = result_output;
            default: data_out = 32'h0;
        endcase
    end

    assign data_ready = 1'b1;

    // === Output Mapping ===
    assign uo_out = {result_output[3:0], result_output[19:16]}; // simplified spike trace

    // === Interrupt Logic ===
    logic slam_interrupt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slam_interrupt <= 1'b0;
        end else if (result_output[31:16] > 16'd1000 || result_output[15:0] > 16'd1000) begin
            slam_interrupt <= 1'b1;
        end else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0]) begin
            slam_interrupt <= 1'b0;
        end
    end

    assign user_interrupt = slam_interrupt;

    // === Prevent Unused Signal Warnings ===
    logic _unused;
    assign _unused = &{ui_in, data_read_n, 1'b0};

endmodule