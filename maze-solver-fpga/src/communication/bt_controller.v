/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         bt_controller.v
# File Description: Bluetooth transmission controller module that formats
#                   and sends maze data (dead-end index, soil status,
#                   temperature, humidity, and END message) via UART
#                   using a controlled FSM-based packet structure.
# Global variables: None
*/

module bt_controller (
    input  wire        clk_50M,
    input  wire        reset,        // Active LOW
    input  wire        enable,
    input  wire        end_bt_enable,

    input  wire [7:0]  payload1,
    input  wire [7:0]  payload2,
    input  wire [7:0]  payload3,
    input  wire [7:0]  payload4,
    input  wire [7:0]  payload5,
    input  wire [7:0]  payload6,

    input  wire        s_done,        // SERVO DONE (PULSE)
    input  wire        tx_busy,       // FROM UART

    output reg  [7:0]  tx_data,
    output reg         tx_start
);

    // =================================================
    // Internal registers
    // =================================================
    reg [25:0] delay_cnt;
    reg        send_en;
    
    reg        sending_end;
    reg        end_sent_local;

    reg [4:0]  char_idx;
    reg        pending_send;
    reg        sending_servo;
    reg        sent_once;
    reg        sent_servo;

    // Servo-done latch (pulse-safe)
    reg servo_done_latched;

    // UART busy edge detect
    reg  tx_busy_d;
    wire tx_done;

    // Enable edge detect
    reg  enable_d;
    reg  enable_l;
    wire enable_rise;


    // =================================================
    // END detection
    // =================================================

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            end_sent_local <= 1'b0;
        else if (end_bt_enable)
            end_sent_local <= 1'b1;
    end


    // =================================================
    // UART DONE detection (busy falling edge)
    // =================================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            tx_busy_d <= 1'b0;
        else
            tx_busy_d <= tx_busy;
    end

    assign tx_done = tx_busy_d && !tx_busy;   // 1-clock pulse

    // =================================================
    // Enable rising-edge detection
    // =================================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            enable_d <= 1'b0;
        else
            enable_d <= enable;
    end

    assign enable_rise = enable & ~enable_d;
    
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            enable_l <= 1'b0;
        end
        else if (enable_rise) begin
            enable_l <= 1'b1;
        end
        // else: hold previous value (this is now a FLIP-FLOP, not a latch)
    end


    // =================================================
    // Delay before FIRST message only
    // =================================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            delay_cnt <= 26'd0;
            send_en   <= 1'b0;
        end
        else if (!enable_l || sent_once) begin
            delay_cnt <= 26'd0;
            send_en   <= 1'b0;
        end
        else if (delay_cnt < 26'd5_000_000) begin
            delay_cnt <= delay_cnt + 1'b1;
            send_en   <= 1'b0;
        end
        else begin
            delay_cnt <= 26'd0;
            send_en   <= 1'b1;   // one-cycle pulse
        end
    end

    // =================================================
    // MAIN FSM
    // =================================================
    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            tx_start           <= 1'b0;
            tx_data            <= 8'd0;
            char_idx           <= 5'd0;
            pending_send       <= 1'b0;
            sending_servo      <= 1'b0;
            sent_once          <= 1'b0;
            sent_servo         <= 1'b0;
            servo_done_latched <= 1'b0;
        end

        else if (enable_rise) begin
            tx_start           <= 1'b0;
            char_idx           <= 5'd0;
            pending_send       <= 1'b0;
            sending_servo      <= 1'b0;
            sent_once          <= 1'b0;
            sent_servo         <= 1'b0;
            servo_done_latched <= 1'b0;
        end

        else begin
            tx_start <= 1'b0;   // default

            // -----------------------------------------
            // Latch servo_done pulse
            // -----------------------------------------
            if (s_done)
                servo_done_latched <= 1'b1;

            // -----------------------------------------
            // Trigger END message (HIGHEST PRIORITY)
            // -----------------------------------------
            if (end_bt_enable && !end_sent_local && !pending_send) begin
                pending_send  <= 1'b1;
                sending_end   <= 1'b1;
                sending_servo <= 1'b0;
                char_idx      <= 5'd0;
            end


            // -----------------------------------------
            // Trigger FIRST sentence
            // -----------------------------------------
            if (send_en && !sent_once && !pending_send && !sending_end) begin
                pending_send  <= 1'b1;
                sending_servo <= 1'b0;
                char_idx      <= 5'd0;
            end

            // -----------------------------------------
            // Trigger SECOND sentence
            // -----------------------------------------
            if (servo_done_latched && !sent_servo && !pending_send && !sending_end) begin
                pending_send  <= 1'b1;
                sending_servo <= 1'b1;
                char_idx      <= 5'd0;
            end

            // -----------------------------------------
            // SEND CHARACTERS (UART SAFE)
            // -----------------------------------------
            if (pending_send && (tx_done || char_idx == 0)) begin

                if (sending_end) begin
                    case (char_idx)
                        0: tx_data <= "E";
                        1: tx_data <= "N";
                        2: tx_data <= "D";
                        3: tx_data <= "-";
                        4: tx_data <= "#";
                    endcase
                end
                else if (!sending_servo) begin
                    // ---------- FIRST MESSAGE ----------
                    case (char_idx)
                        0: tx_data <= "M";
                        1: tx_data <= "P";
                        2: tx_data <= "I";
                        3: tx_data <= "M";
                        4: tx_data <= "-";
                        5: tx_data <= payload1;
                        6: tx_data <= "-";
                        7: tx_data <= "#";
                        8: tx_data <= 8'h0D;
                        9: tx_data <= 8'h0A;
                    endcase
                end
                else begin
                    // ---------- SECOND MESSAGE ----------
                    case (char_idx)
                        0:  tx_data <= "M";
                        1:  tx_data <= "M";
                        2:  tx_data <= "-";
                        3:  tx_data <= payload1;
                        4:  tx_data <= "-";
                        5:  tx_data <= payload2;
                        6:  tx_data <= "-";
                        7:  tx_data <= "#";
                        8:  tx_data <= 8'h0D;
                        9:  tx_data <= 8'h0A;
                        10: tx_data <= "T";
                        11: tx_data <= "H";
                        12: tx_data <= "-";
                        13: tx_data <= payload1;
                        14: tx_data <= "-";
                        15: tx_data <= payload3 + 8'd48;
                        16: tx_data <= payload4 + 8'd48;
                        17: tx_data <= "-";
                        18: tx_data <= payload5 + 8'd48;
                        19: tx_data <= payload6 + 8'd48;
                        20: tx_data <= "-";
                        21: tx_data <= "#";
                        22: tx_data <= 8'h0D;
                        23: tx_data <= 8'h0A;
                    endcase
                end

                tx_start <= 1'b1;

                // -------------------------------------
                // END OF SENTENCE
                // -------------------------------------
                if ((sending_end && char_idx == 4) ||
                    (!sending_servo && !sending_end && char_idx == 9) ||
                    ( sending_servo && char_idx == 23))begin

                    char_idx     <= 5'd0;
                    pending_send <= 1'b0;

                    if (sending_end)
                        sending_end <= 1'b0;


                    if (sending_servo) begin
                        sent_servo         <= 1'b1;
                        sending_servo      <= 1'b0;
                        servo_done_latched <= 1'b0;
                    end
                    else begin
                        sent_once <= 1'b1;
                    end
                end
                else begin
                    char_idx <= char_idx + 1'b1;
                end
            end
        end
    end

endmodule
