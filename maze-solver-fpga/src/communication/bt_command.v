/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         bt_command.v
# File Description: Bluetooth coordinate transmission module that sends 
#                   updated (x,y) maze position via UART in ASCII format 
#                   whenever the robot enters a new cell.
# Global variables: None
*/

module bt_command(
    input  wire clk_50M,
    input  wire reset_n,

    input  wire [3:0] x,
    input  wire [3:0] y,

    output reg        tx_start,
    output reg [7:0]  tx_data,
    input  wire       tx_done
);

reg [3:0] prev_x;
reg [3:0] prev_y;

reg [2:0] send_state;

localparam IDLE   = 3'd0,
           SEND_X = 3'd1,
           SEND_C = 3'd2,
           SEND_Y = 3'd3,
           SEND_R = 3'd4,
           SEND_N = 3'd5;

always @(posedge clk_50M or negedge reset_n) begin
    if (!reset_n) begin
        send_state <= IDLE;
        tx_start   <= 0;
        prev_x     <= 0;
        prev_y     <= 0;
    end
    else begin
        tx_start <= 0;   // default

        case(send_state)

        // ================= IDLE =================
        IDLE: begin
            if ((x != prev_x) || (y != prev_y)) begin
                send_state <= SEND_X;
            end
        end

        // ================= SEND X =================
        SEND_X: begin
            tx_data  <= x + 8'd48;   // ASCII
            tx_start <= 1;
            send_state <= SEND_C;
        end

        // ================= SEND COMMA =================
        SEND_C: if (tx_done) begin
            tx_data  <= 8'h2C;   // ','
            tx_start <= 1;
            send_state <= SEND_Y;
        end

        // ================= SEND Y =================
        SEND_Y: if (tx_done) begin
            tx_data  <= y + 8'd48;
            tx_start <= 1;
            send_state <= SEND_R;
        end

        // ================= SEND \r =================
        SEND_R: if (tx_done) begin
            tx_data  <= 8'h0D;
            tx_start <= 1;
            send_state <= SEND_N;
        end

        // ================= SEND \n =================
        SEND_N: if (tx_done) begin
            tx_data  <= 8'h0A;
            tx_start <= 1;
            prev_x <= x;
            prev_y <= y;

            send_state <= IDLE;
        end

        endcase
    end
end

endmodule