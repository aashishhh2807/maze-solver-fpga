/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         uart_tx.v
# File Description: UART transmitter module operating at 115200 baud 
#                   (50 MHz clock) with EVEN parity generation. It sends 
#                   8-bit parallel data serially with start, parity, and 
#                   stop bits, and generates a tx_done pulse after 
#                   transmission completion.
# Global variables: None
*/

module uart_tx(
    input  wire       clk_50M,
    input  wire       tx_start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        tx_done
);

localparam integer CLKS_PER_BIT = 434;

localparam [3:0] IDLE   = 4'd0,
                 START  = 4'd1,
                 DATA0  = 4'd2,
                 DATA1  = 4'd3,
                 DATA2  = 4'd4,
                 DATA3  = 4'd5,
                 DATA4  = 4'd6,
                 DATA5  = 4'd7,
                 DATA6  = 4'd8,
                 DATA7  = 4'd9,
                 PARITY = 4'd10,
                 STOP   = 4'd11;

reg [8:0] shift_reg;
reg [3:0] state;
reg [8:0] baud_counter;

initial begin
    tx           = 1'b1;
    tx_done      = 1'b0;
    shift_reg    = 9'd0;
    state        = IDLE;
    baud_counter = 9'd0;
end

always @(posedge clk_50M) begin
    tx_done <= 1'b0;

    case (state)

        IDLE: begin
            tx <= 1'b1;
            baud_counter <= 0;
            if (tx_start) begin
                shift_reg <= {^data, data};
                state <= START;
            end
        end

        START: begin
            tx <= 1'b0;
            baud_counter <= baud_counter + 1'b1;
            if (baud_counter >= CLKS_PER_BIT-1) begin
                baud_counter <= 0;
                state <= DATA0;
            end
        end

        DATA0, DATA1, DATA2, DATA3, DATA4, DATA5, DATA6, DATA7: begin
            tx <= shift_reg[0]; 
            baud_counter <= baud_counter + 1'b1;
            if (baud_counter >= CLKS_PER_BIT-1) begin
                baud_counter <= 0;
                shift_reg <= {1'b0, shift_reg[8:1]}; 
                case (state)
                    DATA0: state <= DATA1;
                    DATA1: state <= DATA2;
                    DATA2: state <= DATA3;
                    DATA3: state <= DATA4;
                    DATA4: state <= DATA5;
                    DATA5: state <= DATA6;
                    DATA6: state <= DATA7;
                    DATA7: state <= PARITY;
                endcase
            end
        end

        PARITY: begin
            tx <= shift_reg[0]; // parity bit
            baud_counter <= baud_counter + 1'b1;
            if (baud_counter >= CLKS_PER_BIT-1) begin
                baud_counter <= 0;
                state <= STOP;
            end
        end

        STOP: begin
            tx <= 1'b1;
            baud_counter <= baud_counter + 1'b1;
            if (baud_counter >= CLKS_PER_BIT-1) begin
                baud_counter <= 0;
                tx_done <= 1'b1;
                state <= IDLE;
            end
        end

        default: begin
            tx <= 1'b1;
            state <= IDLE;
        end
    endcase
end

endmodule
