/*
 * Copyright (c) 2025 Rishika Somireddy
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tqvp_neuro_nav_SLAM (
    input         clk,          
    input         rst_n,        

    input  [7:0]  ui_in,        
    output [7:0]  uo_out,       

    input  [5:0]  address,      
    input  [31:0] data_in,      

    input  [1:0]  data_write_n, 
    input  [1:0]  data_read_n,  
    
    output [31:0] data_out,     
    output        data_ready,
    output        user_interrupt
);

    // === Registers ===
    reg [31:0] sensor_input;    
    reg [31:0] control_flags;   
    reg [15:0] pos_x;           
    reg [15:0] pos_y;           
    reg [31:0] result_output;   

    // Neuromorphic / learning registers
    reg [7:0] w_east, w_north, w_west, w_south;
    reg [7:0] decay_counter;
    reg [7:0] prev_ui_in;
    reg [3:0] spike_rising;

    // === Write logic ===
    always @(posedge clk) begin
        if (!rst_n) begin
            sensor_input  <= 32'h0;
            control_flags <= 32'h0;
            w_east  <= 8'd1;
            w_north <= 8'd1;
            w_west  <= 8'd1;
            w_south <= 8'd1;
            decay_counter <= 8'h0;
            prev_ui_in <= 8'h0;
        end else begin
            if (address == 6'h0 && data_write_n != 2'b11)
                sensor_input <= data_in;
            if (address == 6'h4 && data_write_n != 2'b11)
                control_flags <= data_in;

            prev_ui_in <= ui_in;
        end
    end

    // === Spiking detection ===
    always @* begin
        spike_rising = ui_in[3:0] & ~prev_ui_in[3:0];
    end

    // === Core SLAM logic ===
    always @(posedge clk) begin
        if (!rst_n) begin
            pos_x <= 16'h0;
            pos_y <= 16'h0;
            result_output <= 32'h0;
            decay_counter <= 8'h0;
        end else begin
            if (control_flags[0]) begin
                if (control_flags[2]) begin
                    case (sensor_input[17:16])
                        2'd0: pos_x <= pos_x + sensor_input[15:0];
                        2'd1: pos_y <= pos_y + sensor_input[15:0];
                        2'd2: pos_x <= pos_x - sensor_input[15:0];
                        2'd3: pos_y <= pos_y - sensor_input[15:0];
                        default: ;
                    endcase
                end else begin
                    if (spike_rising[0]) pos_x <= pos_x + {8'h0, w_east};
                    if (spike_rising[1]) pos_y <= pos_y + {8'h0, w_north};
                    if (spike_rising[2]) pos_x <= pos_x - {8'h0, w_west};
                    if (spike_rising[3]) pos_y <= pos_y - {8'h0, w_south};

                    if (control_flags[1]) begin
                        if (spike_rising[0] && w_east != 8'hFF)  w_east  <= w_east  + 1;
                        if (spike_rising[1] && w_north != 8'hFF) w_north <= w_north + 1;
                        if (spike_rising[2] && w_west != 8'hFF)  w_west  <= w_west  + 1;
                        if (spike_rising[3] && w_south != 8'hFF) w_south <= w_south + 1;
                    end
                end
            end

            // decay
            if (control_flags[15:8] != 8'h0) begin
                decay_counter <= decay_counter + 1;
                if (decay_counter == control_flags[15:8]) begin
                    if (w_east  > 1) w_east  <= w_east  - 1;
                    if (w_north > 1) w_north <= w_north - 1;
                    if (w_west  > 1) w_west  <= w_west  - 1;
                    if (w_south > 1) w_south <= w_south - 1;
                    decay_counter <= 0;
                end
            end

            result_output <= {pos_y, pos_x};
        end
    end

    // === Readback ===
    assign data_out = (address == 6'h0) ? sensor_input :
                      (address == 6'h4) ? control_flags :
                      (address == 6'h8) ? {pos_y, pos_x} :
                      (address == 6'hC) ? result_output :
                      (address == 6'h14) ? {24'h0, w_east} :
                      (address == 6'h15) ? {24'h0, w_north} :
                      (address == 6'h16) ? {24'h0, w_west} :
                      (address == 6'h17) ? {24'h0, w_south} :
                      32'h0;

    assign data_ready = 1'b1;

    // === Output mapping ===
    assign uo_out = {pos_y[3:0], pos_x[3:0]};

    // === Interrupt logic ===
    reg slam_interrupt;
    always @(posedge clk) begin
        if (!rst_n) begin
            slam_interrupt <= 0;
        end else if ((pos_x > 16'd1000) || (pos_y > 16'd1000)) begin
            slam_interrupt <= 1;
        end else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0]) begin
            slam_interrupt <= 0;
        end
    end

    assign user_interrupt = slam_interrupt;

    // === Prevent unused warnings ===
    wire _unused = &{data_read_n, 1'b0};

endmodule
