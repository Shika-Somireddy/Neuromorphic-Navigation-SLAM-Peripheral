/*
 * Copyright (c) 2025 Rishika Somireddy
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tqvp_neuro_nav_SLAM (
    input  logic        clk,          
    input  logic        rst_n,        

    input  logic [7:0]  ui_in,        // now used for spike inputs (lower 4 bits)
    output logic [7:0]  uo_out,       

    input  logic [5:0]  address,      
    input  logic [31:0] data_in,      

    input  logic [1:0]  data_write_n, 
    input  logic [1:0]  data_read_n,  
    
    output logic [31:0] data_out,     
    output logic        data_ready,
    output logic        user_interrupt
);

    // === Register File ===
    logic [31:0] sensor_input;    // legacy step + direction (kept for compatibility)
    logic [31:0] control_flags;   // enable/reset/etc.
    logic [15:0] pos_x;           // X coordinate
    logic [15:0] pos_y;           // Y coordinate
    logic [31:0] result_output;   // {Y, X}

    // === Neuromorphic additions ===
    // 8-bit integer synaptic weights for each direction
    logic [7:0] w_east, w_north, w_west, w_south;

    // simple decay/homeostasis counter (wraps) - when wraps, decrement weights by 1 (if > 1)
    logic [7:0] decay_counter;

    // previous ui_in for rising-edge detection
    logic [7:0] prev_ui_in;

    // === Write logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_input  <= 32'h0;
            control_flags <= 32'h0;
            // initialize weights small (1)
            w_east  <= 8'd1;
            w_north <= 8'd1;
            w_west  <= 8'd1;
            w_south <= 8'd1;
            decay_counter <= 8'h0;
            prev_ui_in <= 8'h0;
        end else begin
            unique case (address)
                6'h0: if (data_write_n != 2'b11) sensor_input  <= data_in;
                6'h4: if (data_write_n != 2'b11) control_flags <= data_in;
                // weights writable at 0x14 - 0x17
                6'h14: if (data_write_n != 2'b11) w_east  <= data_in[7:0];
                6'h15: if (data_write_n != 2'b11) w_north <= data_in[7:0];
                6'h16: if (data_write_n != 2'b11) w_west  <= data_in[7:0];
                6'h17: if (data_write_n != 2'b11) w_south <= data_in[7:0];
                default: ; // no write
            endcase
            // sample previous ui_in for edge detection
            prev_ui_in <= ui_in;
        end
    end

    // === Core SLAM-like Logic (legacy + spiking mode) ===
    // control_flags bits:
    // [0] updates enable
    // [1] learning enable
    // [2] mode (0=spiking, 1=legacy sensor_input)
    // [15:8] decay period (byte)
    logic [3:0] spike_rising; // detected rising edges for 4 directions

    always_comb begin
        // detect rising edges on ui_in[3:0]
        spike_rising = (ui_in[3:0] & ~prev_ui_in[3:0]);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_x <= 16'h0;
            pos_y <= 16'h0;
            result_output <= 32'h0;
            decay_counter <= 8'h0;
            // weights already initialized in write block on reset
        end else begin
            // mode select: legacy sensor_input mode if control_flags[2]==1
            if (control_flags[0]) begin // updates enabled
                if (control_flags[2]) begin
                    // legacy behavior: use sensor_input field (preserve compatibility)
                    unique case (sensor_input[17:16])  // direction
                        2'd0: pos_x <= pos_x + sensor_input[15:0]; // east
                        2'd1: pos_y <= pos_y + sensor_input[15:0]; // north
                        2'd2: pos_x <= pos_x - sensor_input[15:0]; // west
                        2'd3: pos_y <= pos_y - sensor_input[15:0]; // south
                        default: ; // no move
                    endcase
                end else begin
                    // spiking mode: use ui_in[3:0] rising edges, add/subtract weights
                    // east
                    if (spike_rising[0]) pos_x <= pos_x + {8'h0, w_east};
                    // north
                    if (spike_rising[1]) pos_y <= pos_y + {8'h0, w_north};
                    // west
                    if (spike_rising[2]) pos_x <= pos_x - {8'h0, w_west};
                    // south
                    if (spike_rising[3]) pos_y <= pos_y - {8'h0, w_south};

                    // learning: if enabled, increment corresponding weight on spike
                    if (control_flags[1]) begin
                        if (spike_rising[0] && (w_east != 8'hFF))  w_east  <= w_east  + 8'd1;
                        if (spike_rising[1] && (w_north != 8'hFF)) w_north <= w_north + 8'd1;
                        if (spike_rising[2] && (w_west != 8'hFF))  w_west  <= w_west  + 8'd1;
                        if (spike_rising[3] && (w_south != 8'hFF)) w_south <= w_south + 8'd1;
                    end
                end
            end
            // update result output
            result_output <= {pos_y, pos_x};

            // decay/homeostasis logic: use control_flags[15:8] as period (0 means disabled)
            if (control_flags[15:8] != 8'h0) begin
                decay_counter <= decay_counter + 8'd1;
                if (decay_counter == control_flags[15:8]) begin
                    // decrement weights by 1 but keep at minimum 1
                    if (w_east > 8'd1)  w_east  <= w_east  - 8'd1;
                    if (w_north > 8'd1) w_north <= w_north - 8'd1;
                    if (w_west > 8'd1)  w_west  <= w_west  - 8'd1;
                    if (w_south > 8'd1) w_south <= w_south - 8'd1;
                    decay_counter <= 8'h0;
                end
            end
        end
    end

    // === Readback ===
    always_comb begin
        unique case (address)
            6'h0:  data_out = sensor_input;
            6'h4:  data_out = control_flags;
            6'h8:  data_out = {pos_y, pos_x};
            6'hC:  data_out = result_output;
            // 0x10 is reserved for interrupt clear (writes only) - read returns interrupt state
            6'h10: data_out = {31'h0, slam_interrupt};
            // weights
            6'h14: data_out = {24'h0, w_east};
            6'h15: data_out = {24'h0, w_north};
            6'h16: data_out = {24'h0, w_west};
            6'h17: data_out = {24'h0, w_south};
            default: data_out = 32'h0;
        endcase
    end

    assign data_ready = 1'b1;

    // === Output Mapping ===
    // Split lower 4 bits of X and Y
    assign uo_out = {pos_y[3:0], pos_x[3:0]};

    // === Interrupt Logic ===
    logic slam_interrupt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slam_interrupt <= 1'b0;
        end else if ((pos_x > 16'd1000) || (pos_y > 16'd1000)) begin
            slam_interrupt <= 1'b1; // fire when position exceeds threshold
        end else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0]) begin
            slam_interrupt <= 1'b0; // clear interrupt (preserve original behavior)
        end
    end

    assign user_interrupt = slam_interrupt;

    // === Prevent Unused Signal Warnings ===
    logic _unused;
    assign _unused = &{data_read_n, 1'b0};

endmodule
