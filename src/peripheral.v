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

    // === Register File ===
    logic [31:0] sensor_input;    // step + direction
    logic [31:0] control_flags;   // enable/reset/etc.
    logic [15:0] pos_x;           // X coordinate
    logic [15:0] pos_y;           // Y coordinate
    logic [31:0] result_output;   // {Y, X}

    // === Write logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_input  <= 32'h0;
            control_flags <= 32'h0;
        end else begin
            unique case (address)
                6'h0: if (data_write_n != 2'b11) sensor_input  <= data_in;
                6'h4: if (data_write_n != 2'b11) control_flags <= data_in;
                default: ; // no write
            endcase
        end
    end

    // === Core SLAM-like Logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_x <= 16'h0;
            pos_y <= 16'h0;
            result_output <= 32'h0;
        end else begin
            if (control_flags[0]) begin // enable updates
                unique case (sensor_input[17:16])  // direction
                    2'd0: pos_x <= pos_x + sensor_input[15:0]; // east
                    2'd1: pos_y <= pos_y + sensor_input[15:0]; // north
                    2'd2: pos_x <= pos_x - sensor_input[15:0]; // west
                    2'd3: pos_y <= pos_y - sensor_input[15:0]; // south
                    default: ; // no move
                endcase
            end
            result_output <= {pos_y, pos_x};
        end
    end

    // === Readback ===
    always_comb begin
        unique case (address)
            6'h0: data_out = sensor_input;
            6'h4: data_out = control_flags;
            6'h8: data_out = {pos_y, pos_x};
            6'hC: data_out = result_output;
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
            slam_interrupt <= 1'b0; // clear interrupt
        end
    end

    assign user_interrupt = slam_interrupt;

    // === Prevent Unused Signal Warnings ===
    logic _unused;
    assign _unused = &{ui_in, data_read_n, 1'b0};

endmodule