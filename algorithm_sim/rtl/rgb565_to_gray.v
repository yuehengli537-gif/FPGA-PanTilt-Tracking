`timescale 1ns/1ps

// RGB565 -> full-range 8-bit luma, rounded to the nearest integer.
// Purely combinational: gray = (77*R + 150*G + 29*B + 128) >> 8.
module rgb565_to_gray (
    input  wire [15:0] in_rgb565,
    output wire [7:0]  out_gray
);
    wire [7:0] red;
    wire [7:0] green;
    wire [7:0] blue;
    wire [15:0] weighted_sum;

    assign red   = {in_rgb565[15:11], in_rgb565[15:13]};
    assign green = {in_rgb565[10:5],  in_rgb565[10:9]};
    assign blue  = {in_rgb565[4:0],   in_rgb565[4:2]};
    assign weighted_sum = 16'd77 * red + 16'd150 * green
                        + 16'd29 * blue + 16'd128;
    assign out_gray = weighted_sum[15:8];
endmodule
