`timescale 1ns/1ps

// HDMI-domain point operations. Pixel data and HS/VS/DE share three registers.
// DISPLAY_MODE: 0=RGB, 1=gray, 2=gray+brightness, 3=three columns, 4=auto.
module display_point_ops #(
    parameter integer WIDTH = 1280,
    parameter integer BRIGHTNESS_OFFSET = 24,
    parameter integer DISPLAY_MODE = 4,
    parameter integer MODE_FRAMES = 300
) (
    input wire clk,
    input wire rst_n,
    input wire [15:0] in_rgb565,
    input wire in_de,
    input wire in_hs,
    input wire in_vs,
    output reg [7:0] out_r,
    output reg [7:0] out_g,
    output reg [7:0] out_b,
    output reg out_de,
    output reg out_hs,
    output reg out_vs,
    output wire [1:0] active_mode
);
    localparam integer COUNT_BITS = (MODE_FRAMES < 2) ? 1 : $clog2(MODE_FRAMES);
    reg [COUNT_BITS-1:0] frame_count;
    reg [1:0] auto_mode;
    reg vs_previous;
    assign active_mode = (DISPLAY_MODE == 4) ? auto_mode : DISPLAY_MODE;

    // Commit mode changes during vertical blanking, once per VS rising edge.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_count <= 0;
            auto_mode <= 0;
            vs_previous <= 0;
        end else begin
            vs_previous <= in_vs;
            if (in_vs && !vs_previous) begin
                if (frame_count == MODE_FRAMES - 1) begin
                    frame_count <= 0;
                    auto_mode <= auto_mode + 1'b1;
                end else begin
                    frame_count <= frame_count + 1'b1;
                end
            end
        end
    end

    reg [15:0] rgb565_s0;
    reg de_s0, hs_s0, vs_s0;
    reg [1:0] mode_s0;
    wire [7:0] gray_now;
    rgb565_to_gray gray_converter (
        .in_rgb565(rgb565_s0), .out_gray(gray_now)
    );

    reg [7:0] gray_s1;
    reg [23:0] rgb_s1;
    reg de_s1, hs_s1, vs_s1;
    reg [1:0] mode_s1;
    reg [15:0] column;

    wire [7:0] bright_s1;
    brightness_adjust #(.OFFSET(BRIGHTNESS_OFFSET)) brightness_converter (
        .in_gray(gray_s1), .out_bright(bright_s1)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            column <= 0;
            rgb565_s0 <= 0;
            de_s0 <= 0;
            hs_s0 <= 1;
            vs_s0 <= 1;
            mode_s0 <= 0;
            gray_s1 <= 0;
            rgb_s1 <= 0;
            de_s1 <= 0;
            hs_s1 <= 1;
            vs_s1 <= 1;
            mode_s1 <= 0;
            {out_r, out_g, out_b} <= 0;
            out_de <= 0;
            out_hs <= 1;
            out_vs <= 1;
        end else begin
            if (!in_de)
                column <= 0;
            else
                column <= column + 1'b1;

            // Capture RAM output before the weighted arithmetic path.
            rgb565_s0 <= in_rgb565;
            de_s0 <= in_de;
            hs_s0 <= in_hs;
            vs_s0 <= in_vs;
            // Preserve the baseline RGB expansion in original mode.
            rgb_s1 <= {rgb565_s0[15:11], 3'b000,
                       rgb565_s0[10:5], 2'b00,
                       rgb565_s0[4:0], 3'b000};
            gray_s1 <= gray_now;
            de_s1 <= de_s0;
            hs_s1 <= hs_s0;
            vs_s1 <= vs_s0;
            mode_s1 <= mode_s0;
            if (active_mode == 3) begin
                if (column < WIDTH / 3) mode_s0 <= 0;
                else if (column < (WIDTH * 2) / 3) mode_s0 <= 1;
                else mode_s0 <= 2;
            end else begin
                mode_s0 <= active_mode;
            end

            out_de <= de_s1;
            out_hs <= hs_s1;
            out_vs <= vs_s1;
            if (!de_s1)
                {out_r, out_g, out_b} <= 24'd0;
            else begin
                case (mode_s1)
                    1: {out_r, out_g, out_b} <= {gray_s1, gray_s1, gray_s1};
                    2: {out_r, out_g, out_b} <= {bright_s1, bright_s1, bright_s1};
                    default: {out_r, out_g, out_b} <= rgb_s1;
                endcase
            end
        end
    end
endmodule
