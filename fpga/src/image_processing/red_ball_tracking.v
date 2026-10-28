`timescale 1ns/1ps
// Red-marker detection uses unmodified RGB. The overlay uses the identically
// timed displayed image. No crop, camera actuator, or servo output is implied.
module red_ball_tracking #(
    parameter integer WIDTH=1280, HEIGHT=720, H_TOTAL=1650,
    parameter integer SHOW_SERVO_DEBUG=0,
    parameter integer DISPLAY_MODE=0, BRIGHTNESS_OFFSET=24,
    parameter integer EDGE_THRESHOLD=96, MODE_FRAMES=300,
    parameter integer DEAD_ZONE=32,
    // Three live regions keep the six-direction contour logic within 20K resources.
    parameter integer CANDIDATES=3,
    parameter integer MIN_DIAMETER=12, MAX_DIAMETER=360, MIN_PIXELS=100,
    parameter integer RED_MIN=80, RED_MARGIN=32,
    parameter integer HOLD_FRAMES=45, CONFIRM_FRAMES=3, MATCH_RADIUS=128,
    parameter integer FRAME_TIMEOUT_CYCLES=7425000
) (
    input wire clk, rst_n,
    input wire [15:0] in_rgb565,
    input wire in_de, in_hs, in_vs,
    output wire [7:0] out_r, out_g, out_b,
    output wire out_de, out_hs, out_vs,
    output wire target_valid, frame_update,
    output wire [11:0] center_x, center_y,
    output wire signed [12:0] error_x, error_y,
    output wire centered,
    output wire [1:0] track_state,
    // Signed image-space follow requests, NOT physical servo PWM. Consumers
    // must act only on frame_update with target_valid and calibrate axis signs.
    output wire signed [1:0] follow_x, follow_y,
    input wire [15:0] servo_pan_us, servo_tilt_us,
    input wire [1:0] servo_pan_limit, servo_tilt_limit,
    input wire servo_enabled, servo_source_alive, servo_stale
);
    wire [11:0] bbox_left, bbox_top, bbox_right, bbox_bottom;
    wire measurement_update, measurement_valid;
    wire [11:0] measured_left, measured_top, measured_right, measured_bottom;
    wire [11:0] measured_x, measured_y;
    wire search_enable;
    wire [11:0] search_left, search_top, search_right, search_bottom;
    wire [19:0] pixel_count;
    wire diag_candidate_valid;
    wire [7:0] diag_flags,diag_candidate_flags;
    wire [12:0] diag_width,diag_height,diag_first,diag_last,diag_peak;
    wire [19:0] diag_pixels;
    wire [11:0] diag_left,diag_top,diag_right,diag_bottom;
    red_ball_detector #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .CANDIDATES(CANDIDATES),
        .MIN_DIAMETER(MIN_DIAMETER), .MAX_DIAMETER(MAX_DIAMETER),
        .MIN_PIXELS(MIN_PIXELS), .RED_MIN(RED_MIN), .RED_MARGIN(RED_MARGIN)
    ) detector (
        .clk(clk), .rst_n(rst_n), .in_rgb565(in_rgb565),
        .in_de(in_de), .in_vs(in_vs),
        .search_enable(search_enable), .search_left(search_left), .search_top(search_top),
        .search_right(search_right), .search_bottom(search_bottom),
        .frame_update(measurement_update), .target_valid(measurement_valid),
        .bbox_left(measured_left), .bbox_top(measured_top),
        .bbox_right(measured_right), .bbox_bottom(measured_bottom),
        .center_x(measured_x), .center_y(measured_y), .pixel_count(pixel_count),
        .diag_candidate_valid(diag_candidate_valid),
        .diag_flags(diag_flags), .diag_candidate_flags(diag_candidate_flags),
        .diag_width(diag_width), .diag_height(diag_height), .diag_pixels(diag_pixels),
        .diag_first(diag_first), .diag_last(diag_last), .diag_peak(diag_peak),
        .diag_left(diag_left), .diag_top(diag_top), .diag_right(diag_right), .diag_bottom(diag_bottom)
    );
    red_ball_lock #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .HOLD_FRAMES(HOLD_FRAMES),
        .CONFIRM_FRAMES(CONFIRM_FRAMES), .MATCH_RADIUS(MATCH_RADIUS),
        .FRAME_TIMEOUT_CYCLES(FRAME_TIMEOUT_CYCLES)
    ) lock_filter (
        .clk(clk), .rst_n(rst_n),
        .measurement_update(measurement_update), .measurement_valid(measurement_valid),
        .measurement_left(measured_left), .measurement_top(measured_top),
        .measurement_right(measured_right), .measurement_bottom(measured_bottom),
        .measurement_x(measured_x), .measurement_y(measured_y),
        .frame_update(frame_update), .target_valid(target_valid), .track_state(track_state),
        .bbox_left(bbox_left), .bbox_top(bbox_top),
        .bbox_right(bbox_right), .bbox_bottom(bbox_bottom),
        .center_x(center_x), .center_y(center_y),
        .search_enable(search_enable), .search_left(search_left), .search_top(search_top),
        .search_right(search_right), .search_bottom(search_bottom)
    );
    // Signed pixel error: positive is right/down. Invalid targets must never
    // command movement when a later actuator stage consumes this interface.
    localparam signed [12:0] SCREEN_X=WIDTH/2, SCREEN_Y=HEIGHT/2;
    assign error_x=target_valid ? $signed({1'b0,center_x})-SCREEN_X : 13'sd0;
    assign error_y=target_valid ? $signed({1'b0,center_y})-SCREEN_Y : 13'sd0;
    assign centered=target_valid && error_x>=-DEAD_ZONE && error_x<=DEAD_ZONE &&
                                   error_y>=-DEAD_ZONE && error_y<=DEAD_ZONE;
    assign follow_x=!target_valid ? 2'sd0 :
        (error_x>DEAD_ZONE ? 2'sd1 : (error_x < -DEAD_ZONE ? -2'sd1 : 2'sd0));
    assign follow_y=!target_valid ? 2'sd0 :
        (error_y>DEAD_ZONE ? 2'sd1 : (error_y < -DEAD_ZONE ? -2'sd1 : 2'sd0));

    wire [7:0] processed_r, processed_g, processed_b;
    wire processed_de, processed_hs, processed_vs;
    display_spatial_ops #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .H_TOTAL(H_TOTAL),
        .DISPLAY_MODE(DISPLAY_MODE), .BRIGHTNESS_OFFSET(BRIGHTNESS_OFFSET),
        .EDGE_THRESHOLD(EDGE_THRESHOLD), .MODE_FRAMES(MODE_FRAMES)
    ) processing (
        .clk(clk), .rst_n(rst_n), .in_rgb565(in_rgb565),
        .in_de(in_de), .in_hs(in_hs), .in_vs(in_vs),
        .out_r(processed_r), .out_g(processed_g), .out_b(processed_b),
        .out_de(processed_de), .out_hs(processed_hs), .out_vs(processed_vs),
        .active_mode()
    );
    // Metadata is produced at raw VS and captured by the overlay at delayed
    // VS, well before active video. It describes the previous displayed frame.
    red_ball_overlay #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .DEAD_ZONE(DEAD_ZONE),
        .SHOW_SERVO_DEBUG(SHOW_SERVO_DEBUG)) overlay (
        .clk(clk), .rst_n(rst_n),
        .in_r(processed_r), .in_g(processed_g), .in_b(processed_b),
        .in_de(processed_de), .in_hs(processed_hs), .in_vs(processed_vs),
        .target_valid(target_valid), .track_state(track_state),
        .bbox_left(bbox_left), .bbox_top(bbox_top),
        .bbox_right(bbox_right), .bbox_bottom(bbox_bottom),
        .center_x(center_x), .center_y(center_y),
        .servo_pan_us(servo_pan_us), .servo_tilt_us(servo_tilt_us),
        .servo_pan_limit(servo_pan_limit), .servo_tilt_limit(servo_tilt_limit),
        .servo_enabled(servo_enabled), .servo_source_alive(servo_source_alive), .servo_stale(servo_stale),
        .diag_candidate_valid(diag_candidate_valid),
        .diag_flags(diag_flags), .diag_candidate_flags(diag_candidate_flags),
        .diag_width(diag_width), .diag_height(diag_height), .diag_pixels(diag_pixels),
        .diag_left(diag_left), .diag_top(diag_top), .diag_right(diag_right), .diag_bottom(diag_bottom),
        .out_r(out_r), .out_g(out_g), .out_b(out_b),
        .out_de(out_de), .out_hs(out_hs), .out_vs(out_vs)
    );
endmodule
