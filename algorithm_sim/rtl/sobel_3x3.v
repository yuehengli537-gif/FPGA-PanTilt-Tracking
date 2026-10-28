`timescale 1ns/1ps
// Gx=[-1 0 1;-2 0 2;-1 0 1], Gy=[-1 -2 -1;0 0 0;1 2 1].
// Four registers. Magnitude is |Gx|+|Gy|; display saturates to 255.
module sobel_3x3 #(
    parameter integer EDGE_THRESHOLD = 96
) (
    input wire clk, rst_n, in_valid, in_sof, in_eof,
    input wire [15:0] in_x, in_y,
    input wire [71:0] in_window,
    output reg out_valid, out_sof, out_eof,
    output reg [15:0] out_x, out_y,
    output reg [10:0] out_magnitude,
    output reg [7:0] out_gray,
    output reg out_edge
);
    wire [7:0] p[0:8];
    genvar i;
    generate for(i=0;i<9;i=i+1) begin: unpack
        assign p[i]=in_window[i*8+:8];
    end endgenerate
    reg [9:0] left_sum,right_sum,top_sum,bottom_sum;
    reg signed [10:0] gx,gy;
    wire [10:0] ax = gx[10] ? -gx : gx;
    wire [10:0] ay = gy[10] ? -gy : gy;
    reg [10:0] magnitude;
    reg [2:0] tag0,tag1,tag2;
    reg [31:0] xy0,xy1,xy2;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            left_sum<=0; right_sum<=0; top_sum<=0; bottom_sum<=0;
            gx<=0; gy<=0; magnitude<=0;
            tag0<=0; tag1<=0; tag2<=0; xy0<=0; xy1<=0; xy2<=0;
            out_valid<=0; out_sof<=0; out_eof<=0; out_x<=0; out_y<=0;
            out_magnitude<=0; out_gray<=0; out_edge<=0;
        end else begin
            left_sum <= {2'b0,p[0]}+{1'b0,p[3],1'b0}+{2'b0,p[6]};
            right_sum <= {2'b0,p[2]}+{1'b0,p[5],1'b0}+{2'b0,p[8]};
            top_sum <= {2'b0,p[0]}+{1'b0,p[1],1'b0}+{2'b0,p[2]};
            bottom_sum <= {2'b0,p[6]}+{1'b0,p[7],1'b0}+{2'b0,p[8]};
            gx <= $signed({1'b0,right_sum})-$signed({1'b0,left_sum});
            gy <= $signed({1'b0,bottom_sum})-$signed({1'b0,top_sum});
            magnitude <= ax+ay;
            out_magnitude <= magnitude;
            out_gray <= (magnitude>255) ? 8'd255 : magnitude[7:0];
            out_edge <= magnitude >= EDGE_THRESHOLD;
            tag0<={in_valid,in_sof,in_eof}; tag1<=tag0; tag2<=tag1;
            {out_valid,out_sof,out_eof}<=tag2;
            xy0<={in_x,in_y}; xy1<=xy0; xy2<=xy1; {out_x,out_y}<=xy2;
        end
    end
endmodule
