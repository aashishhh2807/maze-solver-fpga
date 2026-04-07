
/*
# Team ID:          <1109>
# Theme:            Maze Solver bot
# Author List:      <Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan>
# Filename:         top.v
# File Description: Top module integrating maze navigation, motor control,
#                   ultrasonic sensing, dead-end detection, soil moisture,
#                   DHT11, and Bluetooth communication.
# Global variables: None
*/

/*
Purpose:
---
This module acts as the top-level controller for the autonomous maze solving robot.
It integrates:

- FSM based navigation logic
- Ultrasonic obstacle detection
- IR dead-end detection
- Encoder based distance tracking
- Motor PWM control with auto wall correction
- Servo-based sensor mechanism
- Soil moisture monitoring
- DHT11 temperature & humidity monitoring
- Bluetooth communication (UART TX/RX)

The robot follows left-hand/right-hand rule and dynamically switches
algorithm after visiting specified number of dead-ends.
*/

module top (
    input  wire clk_50M,
    input  wire reset,        // Active LOW

    // Ultrasonic
    input  wire echo_front,
    input  wire echo_right,
    input  wire echo_left,

    output wire trig_front,
    output wire trig_right,
    output wire trig_left,

    output wire op_front,
    output wire op_right,
    output wire op_left,

    output wire [15:0] dist_front,
    output wire [15:0] dist_right,
    output wire [15:0] dist_left,

    // IR
    input  wire ir_in1,
    input  wire ir_in2,
    output wire dead_end1,
    output wire dead_end2,

    // Quadrature encoders
    input  wire enc_left_A,
    input  wire enc_left_B,
    input  wire enc_right_A,
    input  wire enc_right_B,

    // Motor driver
    output wire IN1, IN2, IN3, IN4,
    output wire ENA, ENB,
    output wire servo_pwm,

    //Soil Moisture 
    input  wire dout,         // ADC serial data (ADC_SDAT)
    output wire adc_cs_n,     // ADC chip select
    output wire din,          // ADC channel select
    output wire adc_sck,      // ADC serial clock
    output wire [11:0] d_out_ch0, // 12-bit ADC output
    output reg  [7:0]  soil_status, // 'D' or 'M' (for STP / UART)
    
    //Bluetooth 
    input  wire bt_rx,
    output wire bt_tx,
    

    //DHT
    inout wire sensor,   // DHT11 data pin (with external pull-up)
    output wire [7:0] LED
);

// ==================================================
// PULSES, THRESHOLDS & SPEED
// =================================================

// FRONT_THRESH     : Minimum front distance required to consider path open
// SIDE_L_THRESH    : Minimum left distance to consider path open
// SIDE_R_THRESH    : Minimum right distance to consider path open

// UTURN_FRONT      : Front distance threshold to trigger U-turn
// UTURN_LEFT       : Left threshold for U-turn condition
// UTURN_RIGHT      : Right threshold for U-turn condition

// U_TURN_PULSES_OBS: Encoder ticks required for obstacle-based U-turn
// U_TURN_PULSES_SER: Encoder ticks required for sensor-based U-turn
// TURN_RIGHT_PULSES: Encoder ticks required for 90° right turn
// TURN_LEFT_PULSES : Encoder ticks required for 90° left turn
// CELL_TICKS       : Encoder ticks required to move one maze cell
// STOP_DELAY_CYCLES: Delay cycles before stop stabilization

// DIR_LEFT         : Direction constant for left rotation
// DIR_RIGHT        : Direction constant for right rotation

// PWM_FWD_LEFT     : Base PWM duty for left motor (forward)
// PWM_FWD_RIGHT    : Base PWM duty for right motor (forward)
// PWM_TURN         : PWM duty used during turning
// PWM_STOP         : Zero PWM (motor stop)

    localparam FRONT_THRESH   = 16'd350;
    localparam SIDE_L_THRESH  = 16'd350;
	localparam SIDE_R_THRESH  = 16'd350;

    localparam UTURN_FRONT     = 16'd200;
    localparam UTURN_LEFT      = 16'd200;
    localparam UTURN_RIGHT     = 16'd200;
    
  
    localparam U_TURN_PULSES_OBS  = 16'd3400; //16'd3400
    localparam U_TURN_PULSES_SER  = 16'd2900; //16'd2900
    localparam TURN_RIGHT_PULSES  = 16'd1775; //16'd1750; //1775
    localparam TURN_LEFT_PULSES   = 16'd1875; //16'd1800; //1875
    localparam CELL_TICKS         = 16'd4850; //16'd4850  //4850
    localparam STOP_DELAY_CYCLES  = 26'd100_000;   


    localparam DIR_LEFT  = 1'b0;
    localparam DIR_RIGHT = 1'b1;

    localparam [7:0] PWM_FWD_LEFT  = 8'd170; //8'd165 //155
    localparam [7:0] PWM_FWD_RIGHT = 8'd170; //8'd170 //160
	localparam [7:0] PWM_TURN      = 8'd180; //8'd180 //170
    localparam [7:0] PWM_STOP      = 8'd0;

// =================================================
// FSM STATES
// =================================================

// WAIT_FOR_START : Wait for Bluetooth start command
// RESET_TICKS    : Reset encoder base reference
// FORWARD        : Move forward one cell with auto correction
// DECIDE         : Decision making state (left/right/front/U-turn)
// TURN_LEFT      : Perform 90° left rotation
// TURN_RIGHT     : Perform 90° right rotation
// U_TURN         : Perform 180° rotation
// SENSOR_READ    : Activate servo & sensor reading at dead-end
// CELL_STOP      : Temporary motor stop state

    localparam FORWARD           = 4'd1;
    localparam RESET_TICKS       = 4'd2;
    localparam TURN_LEFT         = 4'd3;
    localparam TURN_RIGHT        = 4'd4;
    localparam U_TURN            = 4'd5;
    localparam SENSOR_READ       = 4'd6;
    localparam WAIT_FOR_START    = 4'd7;
    localparam DECIDE            = 4'd8;
    localparam CELL_STOP         = 4'd9;


    reg [3:0] state;
    reg [3:0] prev_state;
    reg [3:0] next_state_after_reset;
    reg turn_dir;
    reg [25:0] stop_delay_cnt;
    
    reg uturn_from_sensor; // flag to indicate if U-turn was triggered by sensor read

    //heading + Position (9X9)
    localparam NORTH = 2'd0;
    localparam EAST  = 2'd1;
    localparam SOUTH = 2'd2;
    localparam WEST  = 2'd3;

    reg [1:0] heading;
    reg [3:0] pos_x;
    reg [3:0] pos_y;

    wire cell_move_done;
    assign cell_move_done = (state == RESET_TICKS && prev_state == FORWARD);

// =================================================
// LED DEBUG
// =================================================
 
    assign LED[0] = right_hand_mode;
    assign LED[1] = (pos_x == 4'd2 && pos_y == 4'd7); 
    assign LED[2] = (pos_x == 4'd8 && pos_y == 4'd7); 
	assign LED[3] = (pos_x == 4'd4 && pos_y == 4'd0); 
	assign LED[4] = (pos_x == 4'd7 && pos_y == 4'd4); 
    assign LED[5] = (pos_x == 4'd1 && pos_y == 4'd0); 
	assign LED[6] = 1'b0; 
    assign LED[7] = (right_hand_mode == 1'b0);

    //assign LED[7:4] = pos_x;   // X coordinate
    //assign LED[3:0] = pos_y;   // Y coordinate

// =================================================
// ULTRASONIC & IR & SERVO
// =================================================
    ultrasonic U1 (
        .clk_50M(clk_50M), .reset(reset),
        .echo_rx(echo_front), .trig(trig_front),
        .op(op_front), .distance_out(dist_front)
    );

    ultrasonic U2 (
        .clk_50M(clk_50M), .reset(reset),
        .echo_rx(echo_right), .trig(trig_right),
        .op(op_right), .distance_out(dist_right)
    );

    ultrasonic U3 (
        .clk_50M(clk_50M), .reset(reset),
        .echo_rx(echo_left), .trig(trig_left),
        .op(op_left), .distance_out(dist_left)
    );

    IR IR1 (
        .clk_50M(clk_50M), .reset(reset),
        .ir_in(ir_in1), .dead_end(dead_end1)
    );

    IR IR2 (
        .clk_50M(clk_50M), .reset(reset),
        .ir_in(ir_in2), .dead_end(dead_end2)
    );
    
    servo_pwm SERVO (
    .clk_50M(clk_50M),.reset(reset),.enable(servo_enable),
    .servo_out(servo_pwm),.done(servo_done)
    );

// =================================================
// SENSOR FLAGS & Servo flags
// =================================================
    wire left_open  = (dist_left  > SIDE_L_THRESH);
    wire front_open = (dist_front > FRONT_THRESH);
    
    wire right_open_raw;
    assign right_open_raw = (dist_right > SIDE_R_THRESH);

    wire right_open;

    assign right_open =
        (pos_x == 4'd4 &&
        pos_y == 4'd0 &&
        visit_4_0_count == 2) ? 1'b0 : right_open_raw; //assuming right side as wall

    wire left_full_open  = (dist_left  > 16'd1000);
    wire front_full_open = (dist_front > 16'd2000);
    wire right_full_open = (dist_right > 16'd1000);

    wire u_left_open  = (dist_left  < UTURN_LEFT);
    wire u_front_open = (dist_front < UTURN_FRONT);
    wire u_right_open = (dist_right < UTURN_RIGHT);

    wire left_too_close;
    wire right_too_close;
    wire left_corr_zone;
    wire right_corr_zone;

    assign left_corr_zone  = (dist_left  < WALL_MAX);
    assign right_corr_zone = (dist_right < WALL_MAX);

    assign left_too_close  =
            left_corr_zone &&
            (dist_left < (WALL_REF - WALL_TOL));

    assign right_too_close =
            right_corr_zone &&
            (dist_right < (WALL_REF - WALL_TOL));

    wire servo_done;
    wire servo_enable;

    assign servo_enable = (state == SENSOR_READ);

// =================================================
// ENCODERS-TICKS
// =================================================
    wire [15:0] left_ticks, right_ticks;

    tick ENC_LEFT (
        .clk(clk_50M), .reset(reset),
        .encA(enc_left_A), .encB(enc_left_B),
        .count(left_ticks)
    );

    tick ENC_RIGHT (
        .clk(clk_50M), .reset(reset),
        .encA(enc_right_B), .encB(enc_right_A),
        .count(right_ticks)
    );

// =================================================
// BASE CAPTURE
// =================================================

    reg [15:0] base_left, base_right;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            base_left  <= 16'd0;
            base_right <= 16'd0;
        end
        else if (state == RESET_TICKS) begin
            base_left  <= left_ticks;
            base_right <= right_ticks;
        end
    end

    wire [15:0] rel_left_ticks  = left_ticks  - base_left;
    wire [15:0] rel_right_ticks = right_ticks - base_right;
    wire [15:0] avg_rel_ticks   = (rel_left_ticks + rel_right_ticks) >> 1;

// =================================================
// FSM
// =================================================
// state                 : Current FSM state
// prev_state            : Previous FSM state
// next_state_after_reset: Stores next state after tick reset
// turn_dir              : Stores turn direction (left/right)
// stop_delay_cnt        : Counter for stop stabilization

// heading               : Current robot heading (NORTH/EAST/SOUTH/WEST)
// pos_x                 : Current X coordinate in maze
// pos_y                 : Current Y coordinate in maze

// right_hand_mode       : 1 = Right-hand rule, 0 = Left-hand rule
// deadend_count         : Stores ASCII index of detected dead-end
// deadend_visited       : Bit map of visited dead-ends

// no_of_deadends        : Number of dead-ends received from Bluetooth
// run_done              : Indicates maze completion

// soil_status           : 'M' (Moist) or 'D' (Dry)
// T_int, rh_int         : DHT11 Temperature & Humidity integer values


    reg right_hand_mode; 
    
    wire forward_stable;
    reg [19:0] forward_cnt;
    localparam FORWARD_STABLE_CYCLES = 20'd500_000; // if it enters forward state, the ticks may be high at one cycle,
                                                    // so to avoid that we are using forward_stable;
    reg [19:0] decide_wait_cnt;
    localparam DECIDE_WAIT_MAX = 20'd500_000; //10ms

    reg forward_d;
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            forward_d <= 1'b0;
        else
            forward_d <= (state == FORWARD);
    end

    wire cell_done_edge = forward_d & (state != FORWARD);
    
    assign forward_stable = (forward_cnt >= FORWARD_STABLE_CYCLES); 

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state <= WAIT_FOR_START;
            prev_state <= WAIT_FOR_START;
            next_state_after_reset <= FORWARD;
            turn_dir <= DIR_LEFT;
            stop_delay_cnt <= 20'd0;
            forward_cnt <= 20'd0;
            
            heading <= NORTH;
            pos_x   <= 4'd4;
            pos_y   <= 4'd8;
        end 
        else if (run_done) begin
            state <= WAIT_FOR_START;
            prev_state <= WAIT_FOR_START;
        end
        else begin
            prev_state <= state;

            // -----------------------------
            // CELL MOVE DETECTION
            // -----------------------------

            if (cell_done_edge) begin
                case (heading)
                    NORTH: pos_y <= pos_y - 1;
                    SOUTH: pos_y <= pos_y + 1;
                    EAST : pos_x <= pos_x + 1;
                    WEST : pos_x <= pos_x - 1;
                endcase
            end

            // -------------------------------
            // Forward stability counter
            // -------------------------------
            if (state != FORWARD)
                forward_cnt <= 20'd0;
            else if (forward_cnt < FORWARD_STABLE_CYCLES)
                forward_cnt <= forward_cnt + 1'b1;

            case (state)

            // --------------------------------
            WAIT_FOR_START: begin
                if (start_bot)
                    state <= RESET_TICKS;
                else 
                    state <= WAIT_FOR_START;
            end
            
            // --------------------------------
            RESET_TICKS: begin
                state <= next_state_after_reset;
            end

            // --------------------------------
            FORWARD: begin

                // Emergency stop if too close to front wall
                if (dist_front <= 16'd160) begin
                    next_state_after_reset <= DECIDE;
                    state <= RESET_TICKS;
                end

                // Normal full cell movement
                else if (avg_rel_ticks >= CELL_TICKS && forward_stable) begin
                    next_state_after_reset <= DECIDE;
                    state <= RESET_TICKS;
                end

            end

            // --------------------------------
            DECIDE: begin

                if (decide_wait_cnt < DECIDE_WAIT_MAX) begin
                    decide_wait_cnt <= decide_wait_cnt + 1;
                end
                else begin
                    decide_wait_cnt <= 0;   // reset counter

                    if (dead_end1 && dead_end2 && !left_open && !right_open && (dist_front < 16'd300) && !deadend_already_visited) begin
                        state <= SENSOR_READ;
                    end

                    else if (u_front_open && u_left_open && u_right_open) begin
                        uturn_from_sensor <= 1'b0;
                        next_state_after_reset <= U_TURN;
                        state <= RESET_TICKS;
                    end

                    else begin
                        // ==================================================
                        // HAND RULE SELECTION
                        // ==================================================

                        if (right_hand_mode == 1'b0) begin
                            // ---------------- LEFT HAND RULE ----------------
                            if (left_open) begin
                                turn_dir <= DIR_LEFT;
                                next_state_after_reset <= TURN_LEFT;
                                state <= RESET_TICKS;
                            end
                            else if (front_open) begin
                                next_state_after_reset <= FORWARD;
                                state <= RESET_TICKS;
                            end
                            else if (right_open) begin
                                turn_dir <= DIR_RIGHT;
                                next_state_after_reset <= TURN_RIGHT;
                                state <= RESET_TICKS;
                            end
                            else begin
                                next_state_after_reset <= FORWARD;
                                state <= RESET_TICKS;
                            end
                        end
                        else begin
                            // ---------------- RIGHT HAND RULE ----------------
                            if (right_open) begin
                                turn_dir <= DIR_RIGHT;
                                next_state_after_reset <= TURN_RIGHT;
                                state <= RESET_TICKS;
                            end
                            else if (front_open) begin
                                next_state_after_reset <= FORWARD;
                                state <= RESET_TICKS;
                            end
                            else if (left_open) begin
                                turn_dir <= DIR_LEFT;
                                next_state_after_reset <= TURN_LEFT;
                                state <= RESET_TICKS;
                            end
                            else begin
                                next_state_after_reset <= FORWARD;
                                state <= RESET_TICKS;
                            end
                        end


                    end
                end
            end

            // --------------------------------
            TURN_LEFT: begin
                if (rel_right_ticks >= TURN_LEFT_PULSES) begin

                    case (heading)
                        NORTH: heading <= WEST;
                        WEST : heading <= SOUTH;
                        SOUTH: heading <= EAST;
                        EAST : heading <= NORTH;
                    endcase

                    next_state_after_reset <= FORWARD;
                    state <= RESET_TICKS;
                end
            end

            // --------------------------------
            TURN_RIGHT: begin
                if (rel_left_ticks >= TURN_RIGHT_PULSES) begin

                        case (heading)
                            NORTH: heading <= EAST;
                            EAST : heading <= SOUTH;
                            SOUTH: heading <= WEST;
                            WEST : heading <= NORTH;
                        endcase

                    next_state_after_reset <= FORWARD;
                    state <= RESET_TICKS;
                end
            end

            // --------------------------------
            SENSOR_READ: begin
                if (servo_done) begin
                    uturn_from_sensor <= 1'b1;
                    next_state_after_reset <= U_TURN;
                    state <= RESET_TICKS;
                end
            end

            // --------------------------------
            U_TURN: begin
                if (rel_right_ticks >=
                (uturn_from_sensor ? U_TURN_PULSES_SER : U_TURN_PULSES_OBS)) begin

                    case (heading)
                        NORTH: heading <= SOUTH;
                        SOUTH: heading <= NORTH;
                        EAST : heading <= WEST;
                        WEST : heading <= EAST;
                    endcase

                    next_state_after_reset <= FORWARD;
                    state <= RESET_TICKS;
                    turn_dir <= DIR_LEFT;
                end
            end

            endcase
        end
    end

// =================================================
// PWM CONTROLLER
// =================================================

    // Auto-correction parameters
    localparam [15:0] WALL_REF      = 16'd140;   // desired side distance
    localparam [15:0] WALL_TOL      = 16'd20;    // tolerance
    localparam [7:0]  PWM_CORR_STEP = 8'd7;
    localparam [15:0] WALL_MAX      = 16'd260;

    reg [7:0] duty_left, duty_right;   // correction strength

    always @(*) begin
        duty_left  = PWM_STOP;
        duty_right = PWM_STOP;

        if (run_done) begin
            duty_left  = PWM_STOP;
            duty_right = PWM_STOP;
        end 
        else begin
            case (state)

                WAIT_FOR_START: begin
                    duty_left  = PWM_STOP;
                    duty_right = PWM_STOP;
                end

                CELL_STOP: begin
                    duty_left  = PWM_STOP;
                    duty_right = PWM_STOP;
                end

                /*FORWARD: begin
                    duty_left  = PWM_FWD_LEFT;
                    duty_right = PWM_FWD_RIGHT;
                end*/

                // FORWARD with auto-correction
                FORWARD: begin

                    // Default: straight
                    duty_left  = PWM_FWD_LEFT;
                    duty_right = PWM_FWD_RIGHT;

                    // Too close to LEFT wall → steer RIGHT
                    if (left_too_close && !right_too_close) begin
                        duty_left  = PWM_FWD_LEFT  + PWM_CORR_STEP;
                        duty_right = PWM_FWD_RIGHT - PWM_CORR_STEP;
                    end

                    // Too close to RIGHT wall → steer LEFT
                    else if (right_too_close && !left_too_close) begin
                        duty_left  = PWM_FWD_LEFT  - PWM_CORR_STEP;
                        duty_right = PWM_FWD_RIGHT + PWM_CORR_STEP;
                    end
                end
               

                SENSOR_READ: begin
                    duty_left  = PWM_STOP;
                    duty_right = PWM_STOP;
                end

                TURN_LEFT: begin
                    duty_left  = PWM_TURN;
                    duty_right = PWM_TURN;
                end

                TURN_RIGHT: begin
                    duty_left  = PWM_TURN;
                    duty_right = PWM_TURN;
                end

                U_TURN: begin
                    duty_left  = PWM_FWD_LEFT;
                    duty_right = PWM_FWD_RIGHT;
                end

            endcase
        end
    end

// =================================================
// PWM GENERATORS
// =================================================
    wire pwm_left, pwm_right;

    pwm_generator PWM_L (
        .clk_50M(clk_50M), .reset(reset),
        .duty(duty_left), .pwm_out(pwm_left)
    );
    pwm_generator PWM_R (
        .clk_50M(clk_50M), .reset(reset),
        .duty(duty_right), .pwm_out(pwm_right)
    );

// =================================================
// MOTOR DRIVER (INS & ENS)
// =================================================
    reg right_fwd, right_rev;
    reg left_fwd,  left_rev;

    always @(*) begin
        // Default = both forward
        right_fwd = 1'b1;
        right_rev = 1'b0;
        left_fwd  = 1'b1;
        left_rev  = 1'b0;

        case (state)

            TURN_LEFT: begin
                // Left reverse, Right forward
                right_fwd = 1'b1;
                right_rev = 1'b0;
                left_fwd  = 1'b0;
                left_rev  = 1'b1;
            end

            TURN_RIGHT: begin
                // Left forward, Right reverse
                right_fwd = 1'b0;
                right_rev = 1'b1;
                left_fwd  = 1'b1;
                left_rev  = 1'b0;
            end

            U_TURN: begin
                // Rotate same as left turn
                right_fwd = 1'b1;
                right_rev = 1'b0;
                left_fwd  = 1'b0;
                left_rev  = 1'b1;
            end

        endcase
    end

    assign IN1 = right_fwd;
    assign IN2 = right_rev;
    assign IN3 = left_fwd;
    assign IN4 = left_rev;

    assign ENA = pwm_right;
    assign ENB = pwm_left;

// =================================================
// Soil Moisture
// =================================================
    reg [3:0] counter;

    always @(posedge clk_50M) begin
        counter <= counter + 1'b1;
    end

    // ADC clock (clk_50 / 16)
    assign adc_sck = counter[3];


    // ADC controller instance 
     adc_controller adc_inst (
        .dout       (dout),
        .adc_sck    (adc_sck),
        .adc_cs_n   (adc_cs_n),
        .din        (din),
        .d_out_ch0  (d_out_ch0)
    );
    
    wire [7:0] s_status;
    assign s_status = soil_status;

    always @(posedge clk_50M) begin
        if (servo_enable) begin
            if (d_out_ch0 <= 12'd1250)
                soil_status <= "M";   // 'M'
            else if (d_out_ch0 > 12'd1250)
                soil_status <= "D";   // 'D'
            else
                soil_status <= soil_status; // optional, can be omitted
        end
        // else: hold previous soil_status automatically
    end

// =================================================
// Deadend detection
// =================================================
    reg deadend_prev;
    reg deadend_event;
    reg deadend_busy;
    reg uturn_prev;
    reg [7:0] deadend_latched;
    reg [7:0] deadend_count;
    reg deadend_in_progress;
    
	wire uturn_event;
    wire bt_enable;

    reg bt_trigger;
    reg bt_triggered;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            bt_trigger   <= 1'b0;
            bt_triggered <= 1'b0;
        end
        else begin
            // Generate pulse only once when entering SENSOR_READ
            if (state == SENSOR_READ && !bt_triggered) begin
                bt_trigger   <= 1'b1;
                bt_triggered <= 1'b1;
            end
            else begin
                bt_trigger <= 1'b0;
            end

            // Reset trigger flag when leaving SENSOR_READ
            if (state != SENSOR_READ)
                bt_triggered <= 1'b0;
        end
    end


    assign uturn_event = (uturn_prev && state == RESET_TICKS);
    assign bt_enable = bt_trigger && !run_done;

    always @(posedge clk_50M or negedge reset) begin 
        if (!reset) begin
            deadend_prev  <= 1'b0;
            deadend_event <= 1'b0;
        end else begin
            deadend_event <= 1'b0;

            if ((dead_end1 && dead_end2 && !left_open && !right_open && !front_open) && !deadend_prev && !deadend_in_progress) 
                    deadend_event <= 1'b1;


            deadend_prev <= (dead_end1 && dead_end2 && !left_open && !right_open && !front_open);
        end
    end

    always @(posedge clk_50M or negedge reset) begin
    if (!reset)
        deadend_in_progress <= 1'b0;
    else if (deadend_event)
        deadend_in_progress <= 1'b1;  
    else if (uturn_event)
        deadend_in_progress <= 1'b0;   
    end


    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            deadend_busy <= 1'b0;
        else if (deadend_event)
            deadend_busy <= 1'b1;      
        else if (servo_done)
            deadend_busy <= 1'b0;      
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            uturn_prev <= 1'b0;
        else
            uturn_prev <= (state == U_TURN);
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            deadend_latched <= 8'h31;   // '1'
        else if (deadend_event)
            deadend_latched <= deadend_count;
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            deadend_count <= 8'h31;  // default '0'
        else begin
            case ({pos_x, pos_y})
                {4'd0,4'd5}: deadend_count <= 8'h31; // '1'
                {4'd2,4'd1}: deadend_count <= 8'h32; // '2'
                {4'd1,4'd2}: deadend_count <= 8'h33; // '3'
                {4'd0,4'd0}: deadend_count <= 8'h34; // '4'
                {4'd1,4'd7}: deadend_count <= 8'h35; // '5'
                {4'd3,4'd5}: deadend_count <= 8'h36; // '6'
                {4'd8,4'd8}: deadend_count <= 8'h37; // '7'
                {4'd7,4'd3}: deadend_count <= 8'h38; // '8'
                {4'd4,4'd1}: deadend_count <= 8'h39; // '9'
                default:     deadend_count <= deadend_count; // hold value
            endcase
        end
    end

  //Deadend visited logic
    reg [8:0] deadend_visited;   // 9 bits for 9 deadends
    wire [3:0] deadend_index;
    assign deadend_index = deadend_count - 8'h31;  // '1' → 0

    wire deadend_already_visited;
    assign deadend_already_visited = deadend_visited[deadend_index];

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            deadend_visited <= 9'b0;
        else if (servo_done)
            deadend_visited[deadend_index] <= 1'b1;
    end

// =================================================
// ALGORITHM SWITCHING
// =================================================

    reg [1:0] visit_4_0_count;
    reg [1:0] visit_4_0;

    reg cell_done_d;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            cell_done_d <= 1'b0;
        else
            cell_done_d <= cell_done_edge;
    end
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            visit_4_0_count <= 2'd0;

        else if (cell_arrived &&
                pos_x == 4'd4 &&
                pos_y == 4'd0)
            visit_4_0_count <= visit_4_0_count + 1'b1;
    end

    // TRUE when robot has ENTERED new cell
    wire cell_arrived;
    assign cell_arrived = cell_done_d;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            visit_4_0 <= 2'd0;

        else if (cell_done_edge &&
                pos_x == 4'd4 &&
                pos_y == 4'd0 &&
                visit_4_0 < 2'd2)
            visit_4_0 <= visit_4_0 + 1'b1;
    end

    reg sensor_read_d;
    reg [3:0] deadend_visit_count;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            sensor_read_d        <= 1'b0;
            deadend_visit_count  <= 4'd0;
            right_hand_mode      <= 1'b1;
        end
        else begin
            sensor_read_d <= (state == SENSOR_READ);

            // When leaving SENSOR_READ (one deadend finished)
            if (sensor_read_d && state != SENSOR_READ) begin
                deadend_visit_count <= deadend_visit_count + 1'b1;

                // Switch exactly when count reaches target
                if ((deadend_visit_count + 1'b1 == no_of_deadends) && (no_of_deadends != 4'd0))
                    right_hand_mode <= 1'b0;
            end
        end
    end

// =================================================
// BT_Transmitter
// =================================================

    wire tx_busy;
    wire tx_start;
    wire [7:0] tx_data;

    uart_tx UART (
        .clk_50M (clk_50M),
        .tx_start(tx_start),
        .data    (tx_data),
        .tx      (bt_tx),
        .tx_done (tx_busy)
    );

    bt_controller BT (
        .clk_50M (clk_50M),
        .reset   (reset),
        .enable  (bt_enable),        
        .end_bt_enable (end_bt_enable), 
        .payload1 (deadend_latched),
        .payload2 (s_status),
        .payload3 (T_int1),
        .payload4 (T_int2),
        .payload5 (rh_int1),
        .payload6 (rh_int2),
        .s_done   (servo_done),
        .tx_busy (tx_busy),
        .tx_data (tx_data),
        .tx_start(tx_start)
    );

// =================================================
// Co_ordinates update
// =================================================

    /*wire        coord_tx_start;
    wire [7:0]  coord_tx_data;
    wire        coord_tx_done;

    uart_tx UART_COORD (
    .clk_50M (clk_50M),
    .tx_start(coord_tx_start),
    .data    (coord_tx_data),
    .tx      (bt_tx),          // directly to Bluetooth TX pin
    .tx_done (coord_tx_done)
    );

    bt_command BT_COORD (
    .clk_50M (clk_50M),
    .reset_n (reset),

    .x (pos_x),   // your coordinate register
    .y (pos_y),

    .tx_start (coord_tx_start),
    .tx_data  (coord_tx_data),
    .tx_done  (coord_tx_done)
    );*/

//=================================================
// BT_Receiver
// =================================================
    wire start_bot;

    wire [7:0] rx_data;
    wire rx_done;
    wire parity_error;
    wire [3:0] cmd_num;

    reg  [3:0] no_of_deadends;

    // UART RX
    uart_rx rx (
        .clk(clk_50M),
        .reset_n(reset),
        .rx(bt_rx),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .parity_error(parity_error)
    );

    // Command controller
    bt_rx_controller cmd (
        .clk(clk_50M),
        .reset_n(reset),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .parity_error(parity_error),
        .start_bot(start_bot),
        .cmd_num(cmd_num)
    );

    // Store number when START command arrives
    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            no_of_deadends <= 4'd0;
        else if (start_bot)
            no_of_deadends <= cmd_num;
    end

// =================================================
// STOP CONDITION
// =================================================

    wire all_open;
    assign all_open = left_full_open && front_full_open && right_full_open;

    reg all_open_prev;
    reg end_event;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            all_open_prev <= 1'b0;
            end_event     <= 1'b0;
        end else begin
            end_event <= 1'b0;

            if (all_open && !all_open_prev)
                end_event <= 1'b1;   

            all_open_prev <= all_open;
        end
    end

    reg end_sent;               //For Bluetooth 

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            end_sent <= 1'b0;
        else if (end_event)
            end_sent <= 1'b1;   // lock forever
    end

    wire end_bt_enable;
    assign end_bt_enable = end_event && !end_sent;

    reg run_done;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset)
            run_done <= 1'b0;
        else if (end_event)
            run_done <= 1'b1;   // lock forever after END
    end

// =================================================
// DHT11
// =================================================

    wire [7:0] dht_T_integral;
    wire [7:0] dht_T_decimal;
    wire [7:0] dht_RH_integral;
    wire [7:0] dht_RH_decimal;
    wire [7:0] dht_Checksum;
    wire       dht_data_valid;

    dht DHT11 (
        .clk_50M     (clk_50M),
        .reset       (reset),
        .sensor      (sensor),   // <-- connect to FPGA pin

        .T_integral  (dht_T_integral),
        .T_decimal   (dht_T_decimal),
        .RH_integral (dht_RH_integral),
        .RH_decimal  (dht_RH_decimal),
        .Checksum    (dht_Checksum),
        .data_valid  (dht_data_valid)
    );

    wire [7:0] T_int;
    wire [7:0] rh_int;

    assign T_int  = dht_T_integral;
    assign rh_int = dht_RH_integral;


    /*wire [7:0] T_int = 8'd23;
    wire [7:0] rh_int = 8'd78;*/

    wire [7:0] T_int1, T_int2, rh_int1, rh_int2;

    assign T_int1 = T_int / 8'd10; 
    assign rh_int1 = rh_int / 8'd10; 
    assign T_int2 = T_int % 8'd10; 
    assign rh_int2 = rh_int % 8'd10;

endmodule