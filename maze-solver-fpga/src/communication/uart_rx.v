/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         uart_rx.v
# File Description: UART receiver module operating at 115200 baud (50 MHz clock)
#                   with EVEN parity checking. It receives serial data from 
#                   Bluetooth, reconstructs 8-bit parallel data, generates a 
#                   receive-done pulse, and flags parity errors.
# Global variables: None
*/

module uart_rx(
    input  wire clk,        // 50 MHz
    input  wire reset_n,
    input  wire rx,         // Bluetooth TX
    output reg  [7:0] rx_data,
    output reg  rx_done,
    output reg  parity_error
);

parameter CLKS_PER_BIT = 434; // 50MHz / 115200

reg [8:0] clk_cnt;
reg [2:0] bit_idx;
reg [7:0] rx_shift;
reg       parity_bit;
reg [2:0] state;

localparam IDLE   = 3'd0;
localparam START  = 3'd1;
localparam DATA   = 3'd2;
localparam PARITY = 3'd3;
localparam STOP   = 3'd4;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state        <= IDLE;
        clk_cnt      <= 0;
        bit_idx      <= 0;
        rx_done      <= 1'b0;
        rx_data      <= 8'd0;
        parity_bit   <= 1'b0;
        parity_error <= 1'b0;
    end else begin
        rx_done <= 1'b0; // default

        case (state)

        // ---------------- IDLE ----------------
        IDLE: begin
            if (rx == 1'b0) begin
                clk_cnt <= CLKS_PER_BIT/2;
                state   <= START;
            end
        end

        // ---------------- START ----------------
        START: begin
            if (clk_cnt == 0) begin 
                clk_cnt <= CLKS_PER_BIT - 1;
                bit_idx <= 0;
                state   <= DATA;
            end else
                clk_cnt <= clk_cnt - 1;
        end

        // ---------------- DATA (8 bits) ----------------
        DATA: begin
            if (clk_cnt == 0) begin
                rx_shift[bit_idx] <= rx;
                clk_cnt <= CLKS_PER_BIT - 1;

                if (bit_idx == 3'd7)
                    state <= PARITY;
                else
                    bit_idx <= bit_idx + 1'b1;
            end else
                clk_cnt <= clk_cnt - 1;
        end

        // ---------------- PARITY ----------------
        PARITY: begin
            if (clk_cnt == 0) begin
                parity_bit <= rx;
                clk_cnt <= CLKS_PER_BIT - 1;

                // EVEN parity check
                if (^rx_shift != rx)
                    parity_error <= 1'b1;
                else
                    parity_error <= 1'b0;

                state <= STOP;
            end else
                clk_cnt <= clk_cnt - 1;
        end

        // ---------------- STOP ----------------
        STOP: begin
            if (clk_cnt == 0) begin
                rx_data <= rx_shift;
                rx_done <= 1'b1;
                state   <= IDLE;
            end else
                clk_cnt <= clk_cnt - 1;
        end

        endcase
    end
end

endmodule