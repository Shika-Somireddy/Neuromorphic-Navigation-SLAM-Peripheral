`default_nettype none

module tqvp_neuro_nav_SLAM (
    input  wire        clk,          
    input  wire        rst_n,        

    input  wire [7:0]  ui_in,        
    output wire [7:0]  uo_out,       

    input  wire [5:0]  address,      
    input  wire [31:0] data_in,      

    input  wire [1:0]  data_write_n, 
    input  wire [1:0]  data_read_n,  
    
    output reg  [31:0] data_out,     
    output wire        data_ready,
    output wire        user_interrupt
);

    // === Registers ===
    reg [31:0] sensor_input;    
    reg [31:0] control_flags;   
    reg [15:0] pos_x;           
    reg [15:0] pos_y;           
    reg [31:0] result_output;   
    reg [7:0]  prev_ui_in;
    reg [3:0]  spike_rising;

    // Spike flags (single-bit)
    reg spike_x_pos, spike_y_pos, spike_x_neg, spike_y_neg;

    localparam [15:0] THRESHOLD = 16'd10;

    // === Write logic ===
    always @(posedge clk or negedge rst_n) begin
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

    // === Spiking detection (async-reset safe) ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_rising <= 4'h0;
            spike_x_pos <= 1'b0;
            spike_y_pos <= 1'b0;
            spike_x_neg <= 1'b0;
            spike_y_neg <= 1'b0;
        end else begin
            spike_rising <= ui_in[3:0] & ~prev_ui_in[3:0];

            spike_x_pos <= spike_rising[0];
            spike_y_pos <= spike_rising[1];
            spike_x_neg <= spike_rising[2];
            spike_y_neg <= spike_rising[3];
        end
    end

    // === Core odometry logic (single driver per signal) ===
    always @(posedge clk or negedge rst_n) begin
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
                default: begin
                    pos_x <= pos_x;
                    pos_y <= pos_y;
                end
            endcase

            // Spike-based updates
            if (spike_x_pos && pos_x < 16'hFFFF)
                pos_x <= pos_x + 1;
            if (spike_y_pos && pos_y < 16'hFFFF)
                pos_y <= pos_y + 1;
            if (spike_x_neg && pos_x > 0)
                pos_x <= pos_x - 1;
            if (spike_y_neg && pos_y > 0)
                pos_y <= pos_y - 1;

            // Thresholded output
            result_output[15:0]  <= (pos_x >= THRESHOLD) ? 16'h1 : 16'h0;
            result_output[31:16] <= (pos_y >= THRESHOLD) ? 16'h1 : 16'h0;
        end else begin
            // Maintain values if control flag not set
            pos_x <= pos_x;
            pos_y <= pos_y;
            result_output <= result_output;
        end
    end

    // === Readback logic ===
    always @* begin
        case (address)
            6'h0: data_out = sensor_input;
            6'h4: data_out = control_flags;
            6'h8: data_out = {pos_y, pos_x};
            6'hC: data_out = result_output;
            default: data_out = 32'h0;
        endcase
    end

    assign data_ready = 1'b1;
    assign uo_out = {pos_y[3:0], pos_x[3:0]};

    // === Interrupt logic ===
    reg slam_interrupt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            slam_interrupt <= 1'b0;
        else if ((pos_x > 16'd1000) || (pos_y > 16'd1000))
            slam_interrupt <= 1'b1;
        else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0])
            slam_interrupt <= 1'b0;
    end
    assign user_interrupt = slam_interrupt;

    // === Prevent unused warnings ===
    wire _unused;
    assign _unused = &{data_read_n, 1'b0};

endmodule