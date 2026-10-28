`timescale 1ns/1ps
// Raster stream, bubbles allowed. Valid-only crop: no invented border pixels.
// Window byte 0 is top-left, byte 8 bottom-right. Coordinates are input centers.
module stream_window_3x3 #(
    parameter integer WIDTH = 64,
    parameter integer HEIGHT = 48
) (
    input wire clk, rst_n,
    input wire in_valid, in_sof,
    input wire [7:0] in_gray,
    output reg out_valid, out_sof, out_eof,
    output reg [15:0] out_x, out_y,
    output reg [71:0] out_window
);
    reg [7:0] bank0 [0:WIDTH-1];
    reg [7:0] bank1 [0:WIDTH-1];
    reg [15:0] x, y;
    wire [15:0] px = in_sof ? 16'd0 : x;
    wire [15:0] py = in_sof ? 16'd0 : y;
    reg [7:0] a0,a1,b0,b1,c0,c1;
    reg [7:0] read0,read1,pixel_s;
    reg valid_s;
    reg [15:0] x_s,y_s;
    wire [7:0] older = y_s[0] ? read1 : read0;
    wire [7:0] previous = y_s[0] ? read0 : read1;
    // RAM is deliberately not reset; the first two rows overwrite stale data.
    always @(posedge clk) begin
        if (rst_n && in_valid) begin
            read0 <= bank0[px];
            read1 <= bank1[px];
            if(py[0]) bank1[px] <= in_gray;
            else bank0[px] <= in_gray;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 0; y <= 0;
            valid_s<=0; x_s<=0; y_s<=0; pixel_s<=0;
            a0<=0; a1<=0; b0<=0; b1<=0; c0<=0; c1<=0;
            out_valid<=0; out_sof<=0; out_eof<=0;
            out_x<=0; out_y<=0; out_window<=0;
        end else begin
            out_valid <= 0; out_sof <= 0; out_eof <= 0;
            valid_s<=in_valid;
            if (in_valid) begin
                x_s<=px; y_s<=py; pixel_s<=in_gray;
                if (px == WIDTH-1) begin
                    x<=0;
                    y <= (py == HEIGHT-1) ? 16'd0 : py+1'b1;
                end else begin
                    x<=px+1'b1; y<=py;
                end
            end
            if(valid_s) begin
                if (x_s == 0) begin
                    a0<=0; b0<=0; c0<=0;
                end else begin
                    a0<=a1; b0<=b1; c0<=c1;
                end
                a1<=older; b1<=previous; c1<=pixel_s;
                if (x_s >= 2 && y_s >= 2) begin
                    out_window <= {pixel_s,c1,c0,previous,b1,b0,older,a1,a0};
                    out_valid <= 1;
                    out_sof <= (x_s == 2 && y_s == 2);
                    out_eof <= (x_s == WIDTH-1 && y_s == HEIGHT-1);
                    out_x <= x_s-1'b1; out_y <= y_s-1'b1;
                end
            end
        end
    end
endmodule
