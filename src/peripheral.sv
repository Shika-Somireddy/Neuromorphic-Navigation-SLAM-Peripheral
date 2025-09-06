/*
 * Copyright (c) 2025 Rishika Somireddy
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

    // === Registers ===
    logic [31:0] sensor_input;    
    logic [31:0] control_flags;   
    logic [15:0] pos_x;           
    logic [15:0] pos_y;           
    logic [31:0] result_output;   
    logic [7:0]  prev_ui_in;
    logic [3:0]  spike_rising;

    // === Write logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_input  <= 32'h0;
            control_flags <= 32'h0;
            pos_x <= 16'h0;
            pos_y <= 16'h0;
            result_output <= 32'h0;
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
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            spike_rising <= 4'h0;
        else
            spike_rising <= ui_in[3:0] & ~prev_ui_in[3:0];
    end

    // === Core odometry logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_x <= 16'h0;
            pos_y <= 16'h0;
            result_output <= 32'h0;
        end else if (control_flags[0]) begin
            // Sensor-based movement
            case (sensor_input[17:16])
                2'd0: pos_x <= pos_x + sensor_input[15:0];
                2'd1: pos_y <= pos_y + sensor_input[15:0];
                2'd2: pos_x <= pos_x - sensor_input[15:0];
                2'd3: pos_y <= pos_y - sensor_input[15:0];
                default: ;
            endcase

            // Spiking odometry
            if (spike_rising[0]) pos_x <= pos_x + 1;
            if (spike_rising[1]) pos_y <= pos_y + 1;
            if (spike_rising[2]) pos_x <= pos_x - 1;
            if (spike_rising[3]) pos_y <= pos_y - 1;

            result_output <= {pos_y, pos_x};
        end
    end

    // === Readback ===
    always_comb begin
        case (address)
            6'h0: data_out = sensor_input;
            6'h4: data_out = control_flags;
            6'h8: data_out = {pos_y, pos_x};
            6'hC: data_out = result_output;
            default: data_out = 32'h0;
        endcase
    end

    assign data_ready = 1'b1;

    // === Output mapping ===
    assign uo_out = {pos_y[3:0], pos_x[3:0]};

    // === Interrupt logic ===
    logic slam_interrupt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            slam_interrupt <= 1'b0;
        else if ((pos_x > 16'd1000) || (pos_y > 16'd1000))
            slam_interrupt <= 1'b1;
        else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0])
            slam_interrupt <= 1'b0;
    end
    assign user_interrupt = slam_interrupt;

    // === Prevent unused warnings ===
    logic _unused;
    assign _unused = &{data_read_n, 1'b0};

endmodule