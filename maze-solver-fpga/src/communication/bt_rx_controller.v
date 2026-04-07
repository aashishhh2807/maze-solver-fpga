/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         bt_rx_controller.v
# File Description: Bluetooth receive command controller that detects 
#                   the "START-n-#" command pattern via UART, extracts 
#                   the numeric value (0–9), and generates a start pulse 
#                   for maze execution.
# Global variables: None
*/
module bt_rx_controller (
    input  wire       clk,
    input  wire       reset_n,     // active LOW
    input  wire [7:0] rx_data,
    input  wire       rx_done,
    input  wire       parity_error,

    output reg        start_bot,
    output reg [3:0]  cmd_num      // <-- captured number (0–9)
);

reg [3:0] char_idx;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        char_idx  <= 4'd0;
        start_bot <= 1'b0;
        cmd_num   <= 4'd0;
    end else begin

        if (rx_done) begin
            case (char_idx)

                4'd0: char_idx <= (rx_data == "S") ? 4'd1 : 4'd0;
                4'd1: char_idx <= (rx_data == "T") ? 4'd2 : 4'd0;
                4'd2: char_idx <= (rx_data == "A") ? 4'd3 : 4'd0;
                4'd3: char_idx <= (rx_data == "R") ? 4'd4 : 4'd0;
                4'd4: char_idx <= (rx_data == "T") ? 4'd5 : 4'd0;
                4'd5: char_idx <= (rx_data == "-") ? 4'd6 : 4'd0;

                // --------- DIGIT STATE ---------
                4'd6: begin
                    if (rx_data >= "0" && rx_data <= "9") begin
                        cmd_num  <= rx_data - "0";  // ASCII → number
                        char_idx <= 4'd7;
                    end else
                        char_idx <= 4'd0;
                end

                4'd7: char_idx <= (rx_data == "-") ? 4'd8 : 4'd0;

                4'd8: begin
                    if (rx_data == "#") begin
                        start_bot <= 1'b1;
                        char_idx  <= 4'd0;
                    end else
                        char_idx <= 4'd0;
                end

                default: char_idx <= 4'd0;
            endcase
        end
    end
end

endmodule
