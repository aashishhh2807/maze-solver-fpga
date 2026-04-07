/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         tick.v
# File Description: Quadrature encoder decoder module that synchronizes 
#                   encoder inputs, detects valid state transitions, and 
#                   increments or decrements a 16-bit tick counter based 
#                   on wheel rotation direction.
# Global variables: None
*/

module tick (
    input  wire        clk,
    input  wire        reset,   // ACTIVE-LOW reset

    input  wire        encA,
    input  wire        encB,

    output reg [15:0]  count
);

    // -------------------------------------------------
    // Synchronizers 
    // -------------------------------------------------
    reg encA_d1, encA_d2;
    reg encB_d1, encB_d2;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            encA_d1 <= 1'b0;
            encA_d2 <= 1'b0;
            encB_d1 <= 1'b0;
            encB_d2 <= 1'b0;
        end else begin
            encA_d1 <= encA;
            encA_d2 <= encA_d1;
            encB_d1 <= encB;
            encB_d2 <= encB_d1;
        end
    end

    // -------------------------------------------------
    // Quadrature decoder (count only)
    // -------------------------------------------------
    reg [1:0] prev_state;
    reg [1:0] curr_state;

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            prev_state <= 2'b00;
            curr_state <= 2'b00;
            count      <= 16'd0;
        end else begin
            curr_state <= {encA_d2, encB_d2};

            if (curr_state != prev_state) begin
                case ({prev_state, curr_state})

                    // -------- Forward --------
                    4'b0001,
                    4'b0111,
                    4'b1110,
                    4'b1000:
                        count <= count + 1'b1;

                    // -------- Reverse --------
                    4'b0010,
                    4'b0100,
                    4'b1101,
                    4'b1011:
                        count <= count - 1'b1;

                    // -------- Invalid --------
                    default:
                        count <= count; // ignore
                endcase

                prev_state <= curr_state;
            end
        end
    end

endmodule