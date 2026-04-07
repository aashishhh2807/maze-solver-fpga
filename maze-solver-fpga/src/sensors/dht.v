/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         dht.v
# File Description: DHT11 sensor interface module that implements the 
#                   single-wire communication protocol to read 40-bit 
#                   temperature and humidity data, extract integral and 
#                   decimal values, and generate a data_valid flag.
# Global variables: None
*/

module dht (
    input  wire        clk_50M,
    input  wire        reset,          // Active LOW
    inout  wire        sensor,

    output reg  [7:0]  T_integral,
    output reg  [7:0]  T_decimal,
    output reg  [7:0]  RH_integral,
    output reg  [7:0]  RH_decimal,
    output reg  [7:0]  Checksum,
    output reg         data_valid
);

    // ================= TIMING =================
    localparam [31:0] ONE_SEC    = 32'd49_999_999;
    localparam [31:0] START_LOW  = 32'd899_999;
    localparam [31:0] START_WAIT = 32'd1_999;
    localparam [31:0] BIT_ONE_TH = 32'd2_500;
    
    // ================= STATES =================
    localparam IDLE        = 4'd0,
               WAIT_1S     = 4'd1,
               START_LOW_S = 4'd2,
               START_WAIT_S= 4'd3,
               RESP_LOW    = 4'd4,
               RESP_HIGH   = 4'd5,
               BIT_LOW_S   = 4'd6,
               BIT_HIGH_S  = 4'd7,
               DONE        = 4'd8;

    reg [3:0]  state;
    reg [31:0] count;
    reg [5:0]  bit_count;
    reg [39:0] shift_reg;

    reg sensor_out;
    reg sensor_oe;
    reg sensor_in_d;
        
    assign sensor = sensor_oe ? sensor_out : 1'bz;
    wire sensor_in = sensor;

    initial begin
        state       = IDLE;
        count       = 32'd0;
        bit_count   = 6'd0;
        shift_reg   = 40'd0;
        sensor_out  = 1'b1;
        sensor_oe   = 1'b0;
        T_integral  = 8'd0;
        T_decimal   = 8'd0;
        RH_integral = 8'd0;
        RH_decimal  = 8'd0;
        Checksum    = 8'd0;
        data_valid  = 1'b0;
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state      <= IDLE;
            count        <= 32'd0;
            data_valid <= 1'b0;
        end else begin

            sensor_in_d <= sensor_in;
            case (state)

            IDLE: begin
                count      <= 32'd0;
                shift_reg  <= 40'd0;
                sensor_oe  <= 1'b0;
                sensor_out <= 1'b1;
                state <= WAIT_1S;
            end

            WAIT_1S: begin
                if (count < ONE_SEC)
                    count <= count + 1;
                else begin
                    count      <= 32'd0;
                    sensor_oe  <= 1'b1;
                    sensor_out <= 1'b0;
                    state <= START_LOW_S;
                end
            end

            START_LOW_S: begin
                if (count < START_LOW)begin
                    count      <= count + 1;
                    sensor_oe  <= 1'b1;
                    sensor_out <= 1'b0;
                end
                else begin
                    count      <= 32'd0;
                    sensor_oe  <= 1'b1;
                    sensor_out <= 1'b1;
                    state <= START_WAIT_S;
                end
            end

            START_WAIT_S: begin
                if (count < START_WAIT)begin
                    count <= count + 1;
                    sensor_oe  <= 1'b1;
                    sensor_out <= 1'b1;
                end
                else begin
                    sensor_oe <= 1'b0;
                    if (!sensor_in_d) begin
                        state <= RESP_LOW;
                        count <= 32'b0;
                    end
                end
            end

            RESP_LOW: begin
                sensor_oe <= 1'b0;
                if (sensor_in_d)
                    state <= RESP_HIGH;
            end

            RESP_HIGH: begin
                sensor_oe <= 1'b0;
                if (!sensor_in_d) begin
                    state     <= BIT_LOW_S;
                    count     <= 32'd0;
                    bit_count <= 6'd0;
                    shift_reg <= 40'd0;
                 end
            end

            BIT_LOW_S: begin
                sensor_oe <= 1'b0;
                if (sensor_in_d)begin
                    state <= BIT_HIGH_S;
                end
            end

            BIT_HIGH_S: begin
                sensor_oe <= 1'b0;
                if (sensor_in_d) 
                        count <= count + 1;
                else begin
                    if (count > BIT_ONE_TH)
                        shift_reg <= {shift_reg[38:0], 1'b1};
                    else
                        shift_reg <= {shift_reg[38:0], 1'b0};

                    bit_count <= bit_count + 1;
                    count <= 32'd0;
                    
                    if (bit_count == 6'd39)
                        state <= DONE;
                    else begin
                        state <= BIT_LOW_S;
                    end
                end
            end

            DONE: begin
                sensor_oe <= 0; // output high
                sensor_out <= 1;
                
                RH_integral <= shift_reg[39:32];
                RH_decimal  <= shift_reg[31:24];
                T_integral  <= shift_reg[23:16];
                T_decimal   <= shift_reg[15:8];
                Checksum    <= shift_reg[7:0];
                data_valid  <= 1'b1;

                state <= IDLE;
                end

            default: state <= IDLE;

            endcase
        end
    end

endmodule
