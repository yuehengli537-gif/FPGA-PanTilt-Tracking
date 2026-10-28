`timescale 1ns/1ps
// Parallel Gaussian, raw Sobel and Gaussian->Sobel raster streams.
// WIDTH/HEIGHT >= 5. Outputs carry center coordinates in the ORIGINAL image.
module spatial_filter_top #(
    parameter integer WIDTH=64, HEIGHT=48, EDGE_THRESHOLD=96
) (
    input wire clk,rst_n,in_valid,in_sof,
    input wire [15:0] in_rgb565,
    output wire gauss_valid,gauss_sof,gauss_eof,
    output wire [15:0] gauss_x,gauss_y,
    output wire [7:0] gauss_gray,
    output wire sobel_valid,sobel_sof,sobel_eof,
    output wire [15:0] sobel_x,sobel_y,
    output wire [7:0] sobel_gray,
    output wire [10:0] sobel_magnitude,
    output wire sobel_edge,
    output wire smooth_valid,smooth_sof,smooth_eof,
    output wire [15:0] smooth_x,smooth_y,
    output wire [7:0] smooth_gray,
    output wire [10:0] smooth_magnitude,
    output wire smooth_edge
);
    wire [7:0] gray;
    wire wv,ws,we;
    wire [15:0] wx,wy;
    wire [71:0] window_pixels;
    rgb565_to_gray converter(.in_rgb565(in_rgb565),.out_gray(gray));
    stream_window_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window_raw(
        .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_sof(in_sof),.in_gray(gray),
        .out_valid(wv),.out_sof(ws),.out_eof(we),.out_x(wx),.out_y(wy),.out_window(window_pixels));
    gaussian_3x3 gaussian(
        .clk(clk),.rst_n(rst_n),.in_valid(wv),.in_sof(ws),.in_eof(we),
        .in_x(wx),.in_y(wy),.in_window(window_pixels),
        .out_valid(gauss_valid),.out_sof(gauss_sof),.out_eof(gauss_eof),
        .out_x(gauss_x),.out_y(gauss_y),.out_gray(gauss_gray));
    sobel_3x3 #(.EDGE_THRESHOLD(EDGE_THRESHOLD)) sobel(
        .clk(clk),.rst_n(rst_n),.in_valid(wv),.in_sof(ws),.in_eof(we),
        .in_x(wx),.in_y(wy),.in_window(window_pixels),
        .out_valid(sobel_valid),.out_sof(sobel_sof),.out_eof(sobel_eof),
        .out_x(sobel_x),.out_y(sobel_y),.out_gray(sobel_gray),
        .out_magnitude(sobel_magnitude),.out_edge(sobel_edge));
    wire swv,sws,swe;
    wire [15:0] swx,swy;
    wire [71:0] smooth_window;
    stream_window_3x3 #(.WIDTH(WIDTH-2),.HEIGHT(HEIGHT-2)) window_smoothed(
        .clk(clk),.rst_n(rst_n),.in_valid(gauss_valid),.in_sof(gauss_sof),.in_gray(gauss_gray),
        .out_valid(swv),.out_sof(sws),.out_eof(swe),.out_x(swx),.out_y(swy),.out_window(smooth_window));
    sobel_3x3 #(.EDGE_THRESHOLD(EDGE_THRESHOLD)) sobel_smoothed(
        .clk(clk),.rst_n(rst_n),.in_valid(swv),.in_sof(sws),.in_eof(swe),
        .in_x(swx+16'd1),.in_y(swy+16'd1),.in_window(smooth_window),
        .out_valid(smooth_valid),.out_sof(smooth_sof),.out_eof(smooth_eof),
        .out_x(smooth_x),.out_y(smooth_y),.out_gray(smooth_gray),
        .out_magnitude(smooth_magnitude),.out_edge(smooth_edge));
endmodule
