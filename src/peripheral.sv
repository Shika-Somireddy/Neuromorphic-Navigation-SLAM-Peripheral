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
    logic [15:0] pos_x_membrane;           
    logic [15:0] pos_y_membrane;           
    logic [31:0] result_output;   
    logic [7:0]  prev_ui_in;
    logic [3:0]  spike_rising;

    // Refractory counters
    logic [3:0] refractory_x, refractory_y;

    // Spike thresholds
    localparam [15:0] THRESHOLD = 16'd10;

    // === Write logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sensor_input  <= 32'h0;
            control_flags <= 32'h0;
            pos_x_membrane <= 16'h0;
            pos_y_membrane <= 16'h0;
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

    // === Neuromorphic odometry logic ===
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_x_membrane <= 16'h0;
            pos_y_membrane <= 16'h0;
            result_output <= 32'h0;
            refractory_x <= 4'h0;
            refractory_y <= 4'h0;
        end else if (control_flags[0]) begin
            // Leaky integration
            pos_x_membrane <= pos_x_membrane - (pos_x_membrane >> 4) + (sensor_input[15:0]);
            pos_y_membrane <= pos_y_membrane - (pos_y_membrane >> 4) + (sensor_input[31:16]);

            // Spiking odometry with refractory
            if (spike_rising[0] && !refractory_x[0]) begin
                pos_x_membrane <= pos_x_membrane + 1;
                refractory_x[0] <= 4'hF; // reset refractory
            end
            if (spike_rising[1] && !refractory_y[0]) begin
                pos_y_membrane <= pos_y_membrane + 1;
                refractory_y[0] <= 4'hF;
            end
            if (spike_rising[2] && !refractory_x[1]) begin
                pos_x_membrane <= pos_x_membrane - 1;
                refractory_x[1] <= 4'hF;
            end
            if (spike_rising[3] && !refractory_y[1]) begin
                pos_y_membrane <= pos_y_membrane - 1;
                refractory_y[1] <= 4'hF;
            end

            // decrement refractory counters
            refractory_x <= refractory_x - 1;
            refractory_y <= refractory_y - 1;

            // Thresholding to generate output spikes
            result_output[15:0]  <= (pos_x_membrane >= THRESHOLD) ? 16'h1 : 16'h0;
            result_output[31:16] <= (pos_y_membrane >= THRESHOLD) ? 16'h1 : 16'h0;
        end
    end

    // === Readback ===
    always_comb begin
        case (address)
            6'h0: data_out = sensor_input;
            6'h4: data_out = control_flags;
            6'h8: data_out = {pos_y_membrane, pos_x_membrane};
            6'hC: data_out = result_output;
            default: data_out = 32'h0;
        endcase
    end

    assign data_ready = 1'b1;
    assign uo_out = {pos_y_membrane[3:0], pos_x_membrane[3:0]};

    // === Interrupt logic ===
    logic slam_interrupt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            slam_interrupt <= 1'b0;
        else if ((pos_x_membrane > 16'd1000) || (pos_y_membrane > 16'd1000))
            slam_interrupt <= 1'b1;
        else if (address == 6'h10 && data_write_n != 2'b11 && data_in[0])
            slam_interrupt <= 1'b0;
    end
    assign user_interrupt = slam_interrupt;

    // === Prevent unused warnings ===
    logic _unused;
    assign _unused = &{data_read_n, 1'b0};

endmodule