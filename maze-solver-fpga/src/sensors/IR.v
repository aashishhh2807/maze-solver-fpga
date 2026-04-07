/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         IR.v
# File Description: Infrared sensor interface module that detects dead-end 
#                   condition by reading IR sensor output and generating 
#                   an active-high dead_end signal.
# Global variables: None
*/

module IR (
    input  wire clk_50M,
    input  wire reset,        // Active LOW
    input  wire ir_in,         // IR sensor output
    output reg  dead_end       // 1 = dead end detected
);

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            dead_end <= 1'b0;
        else
            dead_end <= ~ir_in;   // direct mapping
    end

endmodule
