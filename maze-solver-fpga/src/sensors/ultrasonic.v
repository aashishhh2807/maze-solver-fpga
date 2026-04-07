/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         ultrasonic.v
# File Description: Ultrasonic sensor interface module that generates a 
#                   periodic trigger pulse, measures echo pulse width using 
#                   a 50 MHz clock, calculates distance in millimeters, and 
#                   provides obstacle detection output based on a threshold.
# Global variables: None
*/

module ultrasonic (
    input  wire        clk_50M,
    input  wire        reset,        // Active LOW
    input  wire        echo_rx,

    output reg         trig,
    output reg         op,
    output reg [15:0]  distance_out
);

    // ----------------------------------
    // Constants
    // ----------------------------------
    localparam TRIG_PULSE  = 32'd500;        // 10 µs
    localparam TRIG_PERIOD = 32'd3_000_000;  // 60 ms

    // ----------------------------------
    // Registers
    // ----------------------------------
    reg [31:0] trig_count;
    reg [31:0] echo_count;
    reg        measuring;

    // ----------------------------------
    // Trigger generator (periodic)
    // ----------------------------------
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            trig       <= 0;
            trig_count <= 0;
        end else begin
            if (trig_count < TRIG_PULSE)
                trig <= 1;
            else
                trig <= 0;

            if (trig_count < TRIG_PERIOD)
                trig_count <= trig_count + 1;
            else
                trig_count <= 0;   // 🔑 RESET counter → retrigger
        end
    end

    // ----------------------------------
    // Echo measurement & distance
    // ----------------------------------
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            echo_count   <= 0;
            measuring    <= 0;
            distance_out <= 0;
            op           <= 0;
        end else begin
            if (echo_rx) begin
                measuring  <= 1;
                echo_count <= echo_count + 1;
            end else if (measuring) begin
                measuring <= 0;

                // Distance in mm (50 MHz)
                distance_out <= echo_count / 16'd145;

                // Object detect (< 300 mm)
                op <= ((echo_count / 16'd145) < 16'd300);

                echo_count <= 0;
            end
        end
    end

endmodule