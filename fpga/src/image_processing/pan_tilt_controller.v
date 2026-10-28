`timescale 1ns/1ps
// Same clock domain as red_ball_tracking. No asynchronous telemetry inputs.
// PWM is positional: holding the last pulse width is NOT powering the servo off.
module pan_tilt_controller #(
    parameter integer CLK_HZ=74250000,
    parameter integer PERIOD_US=20000, TIMEOUT_US=100000,
    parameter integer HOME_US=1500,
    parameter integer PAN_MIN_US=1300, PAN_MAX_US=1700,
    parameter integer TILT_MIN_US=1300, TILT_MAX_US=1700,
    parameter integer START_PIXELS=48, STOP_PIXELS=32, FAST_PIXELS=128,
    parameter integer SLOW_STEP_US=2, FAST_STEP_US=8,
    parameter integer PAN_REVERSE=0, TILT_REVERSE=0
) (
    input wire clk, rst_n, armed, source_alive,
    input wire frame_update, target_valid,
    input wire [1:0] track_state,
    input wire signed [12:0] error_x, error_y,
    output reg pwm_pan, pwm_tilt,
    output reg [15:0] pan_us, tilt_us,
    output reg period_tick,
    output wire stale
);
    localparam integer AW=$clog2(CLK_HZ+1);
    localparam integer TW=$clog2(TIMEOUT_US+1);
    reg [AW-1:0] micro_acc;
    // Fractional clock divider: exact average microseconds even at 74.25 MHz.
    wire micro_tick=(micro_acc>=CLK_HZ-1000000);
    reg [15:0] phase_us;
    reg [TW-1:0] age_us;
    reg running, fresh;
    reg signed [1:0] direction_x, direction_y;
    reg [15:0] step_x, step_y;
    wire locked=source_alive && target_valid && track_state==2'd2;
    wire new_measurement=frame_update && locked;
    assign stale=(age_us>=TIMEOUT_US);

    function signed [1:0] direction;
        input signed [12:0] error;
        input signed [1:0] previous;
        begin
            if (error>START_PIXELS) direction=2'sd1;
            else if (error < -START_PIXELS) direction=-2'sd1;
            else if (previous==2'sd1 && error>STOP_PIXELS) direction=2'sd1;
            else if (previous==-2'sd1 && error < -STOP_PIXELS) direction=-2'sd1;
            else direction=2'sd0;
        end
    endfunction
    function [15:0] speed;
        input signed [12:0] error;
        begin
            speed=(error>=FAST_PIXELS || error<=-FAST_PIXELS) ? FAST_STEP_US : SLOW_STEP_US;
        end
    endfunction
    function [15:0] advance;
        input [15:0] old_us, delta_us;
        input signed [1:0] request;
        input integer reverse_axis, lower_us, upper_us;
        reg signed [17:0] candidate;
        begin
            candidate=$signed({2'b00,old_us});
            if (request==2'sd1)
                candidate=reverse_axis ? candidate-delta_us : candidate+delta_us;
            else if (request==-2'sd1)
                candidate=reverse_axis ? candidate+delta_us : candidate-delta_us;
            if (candidate<lower_us) advance=lower_us;
            else if (candidate>upper_us) advance=upper_us;
            else advance=candidate[15:0];
        end
    endfunction
    wire signed [1:0] next_x=direction(error_x,direction_x);
    wire signed [1:0] next_y=direction(error_y,direction_y);
    // A result arriving exactly at the period boundary supersedes old data.
    wire can_move=locked && (new_measurement ||
        (fresh && !stale && !(micro_tick && age_us==TIMEOUT_US-1)));
    wire signed [1:0] request_x=new_measurement ? next_x : direction_x;
    wire signed [1:0] request_y=new_measurement ? next_y : direction_y;
    wire [15:0] amount_x=new_measurement ? speed(error_x) : step_x;
    wire [15:0] amount_y=new_measurement ? speed(error_y) : step_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            micro_acc<=0; phase_us<=0; age_us<=TIMEOUT_US;
            running<=0; fresh<=0; direction_x<=0; direction_y<=0;
            step_x<=0; step_y<=0;
            pan_us<=HOME_US; tilt_us<=HOME_US;
            pwm_pan<=0; pwm_tilt<=0; period_tick<=0;
        end else if (!armed) begin
            micro_acc<=0; phase_us<=0; age_us<=TIMEOUT_US;
            running<=0; fresh<=0; direction_x<=0; direction_y<=0;
            step_x<=0; step_y<=0;
            pan_us<=HOME_US; tilt_us<=HOME_US;
            pwm_pan<=0; pwm_tilt<=0; period_tick<=0;
        end else if (!running) begin
            // A fresh enable always starts a complete neutral pulse. Disarming
            // can truncate a pulse; this explicit output-off action is immediate.
            running<=1; micro_acc<=0; phase_us<=0;
            pwm_pan<=1; pwm_tilt<=1; period_tick<=1;
        end else begin
            period_tick<=0;
            if (micro_tick) begin
                micro_acc<=micro_acc-(CLK_HZ-1000000);
                if (age_us<TIMEOUT_US) age_us<=age_us+1'b1;
            end else micro_acc<=micro_acc+1000000;

            if (!locked || stale) begin
                fresh<=0; direction_x<=0; direction_y<=0;
            end
            if (new_measurement) begin
                age_us<=0; fresh<=1;
                direction_x<=next_x; direction_y<=next_y;
                step_x<=speed(error_x); step_y<=speed(error_y);
            end
            if (micro_tick) begin
                if (phase_us==PERIOD_US-1) begin
                    phase_us<=0; period_tick<=1;
                    pwm_pan<=1; pwm_tilt<=1;
                    // Consume once, including a new result on this boundary.
                    fresh<=0;
                    if (can_move) begin
                        pan_us<=advance(pan_us,amount_x,request_x,PAN_REVERSE,PAN_MIN_US,PAN_MAX_US);
                        tilt_us<=advance(tilt_us,amount_y,request_y,TILT_REVERSE,TILT_MIN_US,TILT_MAX_US);
                    end
                end else begin
                    phase_us<=phase_us+1'b1;
                    if (phase_us==pan_us-1) pwm_pan<=0;
                    if (phase_us==tilt_us-1) pwm_tilt<=0;
                end
            end
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (CLK_HZ<1000000 || TIMEOUT_US<2 || PERIOD_US>65535 || PERIOD_US<2 ||
            PAN_MIN_US<1 || PAN_MIN_US>HOME_US || PAN_MAX_US<HOME_US || PAN_MAX_US>=PERIOD_US ||
            TILT_MIN_US<1 || TILT_MIN_US>HOME_US || TILT_MAX_US<HOME_US || TILT_MAX_US>=PERIOD_US ||
            STOP_PIXELS<0 || START_PIXELS<STOP_PIXELS || FAST_PIXELS<START_PIXELS ||
            SLOW_STEP_US<1 || FAST_STEP_US<SLOW_STEP_US || FAST_STEP_US>65535)
            $fatal(1,"pan_tilt_controller: invalid parameters");
    end
`endif
endmodule

// Camera VS crosses into the HDMI domain through two flops. A repeated DDR
// frame must not keep driving the servos when camera capture has stopped.
// This detects missing VS edges, not a frozen image with VS still toggling.
module camera_frame_guard #(
    parameter integer TIMEOUT_CYCLES=18562500 // 250 ms at 74.25 MHz
) (
    input wire clk, rst_n, camera_vs_async,
    output reg source_alive
);
    localparam integer CW=(TIMEOUT_CYCLES<2) ? 1 : $clog2(TIMEOUT_CYCLES+1);
    reg [1:0] vs_sync;
    reg vs_previous;
    reg [CW-1:0] age;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_sync<=0; vs_previous<=0; age<=TIMEOUT_CYCLES; source_alive<=0;
        end else begin
            vs_sync<={vs_sync[0],camera_vs_async}; vs_previous<=vs_sync[1];
            if(vs_sync[1] && !vs_previous) begin
                age<=0; source_alive<=1;
            end else if(age<TIMEOUT_CYCLES) begin
                age<=age+1'b1;
                if(age==TIMEOUT_CYCLES-1) source_alive<=0;
            end
        end
    end
endmodule
