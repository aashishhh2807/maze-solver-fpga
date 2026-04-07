/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         pwm_generator.v
# File Description: 8-bit PWM generator module that produces a pulse-width 
#                   modulated output signal based on the provided duty cycle 
#                   input for motor speed control.
# Global variables: None
*/
module pwm_generator (
    input  wire        clk_50M,
    input  wire        reset,     // Active LOW
    input  wire [7:0]  duty,      // 0–255 duty cycle
    output reg         pwm_out
);

    reg [7:0] counter;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            counter <= 8'd0;
        else
            counter <= counter + 1;
    end

    always @(*) begin
        pwm_out = (counter < duty) ? 1'b1 : 1'b0;
    end

endmodule