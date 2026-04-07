/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         servo_pwm.v
# File Description: Servo motor control module that generates 50Hz PWM 
#                   signal for positional control. On enable, the servo 
#                   moves to a target position, waits for 2 seconds, 
#                   returns to initial position, and asserts a done signal.
# Global variables: None
*/

module servo_pwm (
    input  wire clk_50M,
    input  wire reset,        // Active LOW
    input  wire enable,       // start servo sequence
    output reg  servo_out,
    output reg  done           // goes HIGH when finished
);

    // ------------------------------------------------
    // PWM timing (50 MHz)
    // ------------------------------------------------
    localparam PERIOD = 20'd1_000_000; // 20 ms

    // ------------------------------------------------
    // Servo pulse values (OPPOSITE DIRECTION)
    // Swap these to reverse motion
    // ------------------------------------------------
    localparam PULSE_INIT   = 20'd95_000; // start position
    localparam PULSE_TARGET = 20'd42_000;  // move opposite direction

    // ------------------------------------------------
    // 2-second delay counter
    // ------------------------------------------------
    localparam WAIT_2S = 32'd125_000_000; // 2 seconds @ 50 MHz

    // ------------------------------------------------
    // Control registers
    // ------------------------------------------------
    reg [19:0] pulse_reg;
    reg [31:0] wait_cnt;

    // ------------------------------------------------
    // Motion control (ONE-SHOT)
    // ------------------------------------------------
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            pulse_reg <= PULSE_INIT;
            wait_cnt  <= 32'd0;
            done      <= 1'b0;
        end else begin
            // If not enabled → reset internal state
            if (!enable) begin
                pulse_reg <= PULSE_INIT;
                wait_cnt  <= 32'd0;
                done      <= 1'b0;
            end
            // Enabled and not done → run sequence
            else if (!done) begin
                pulse_reg <= PULSE_TARGET;

                if (wait_cnt < WAIT_2S)
                    wait_cnt <= wait_cnt + 1'b1;
                else begin
                    pulse_reg <= PULSE_INIT;
                    done <= 1'b1;
                end
            end
        end
    end


    // ------------------------------------------------
    // PWM generator (continuous, stable)
    // ------------------------------------------------
    reg [19:0] cnt;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            cnt <= 20'd0;
            servo_out <= 1'b0;
        end else begin
            if (cnt >= PERIOD - 1)
                cnt <= 20'd0;
            else
                cnt <= cnt + 1'b1;

            servo_out <= (cnt < pulse_reg);
        end
    end

endmodule
 