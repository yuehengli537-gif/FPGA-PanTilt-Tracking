`timescale 1ns/1ps
// Separable [1 2 1] x [1 2 1], round-to-nearest divide by 16.
// Three registers, including the output. No multipliers.
module gaussian_3x3 (
    input wire clk, rst_n, in_valid, in_sof, in_eof,
    input wire [15:0] in_x, in_y,
    input wire [71:0] in_window,
    output reg out_valid, out_sof, out_eof,
    output reg [15:0] out_x, out_y,
    output reg [7:0] out_gray
);
    wire [7:0] p[0:8];
    genvar i;
    generate for(i=0;i<9;i=i+1) begin: unpack
        assign p[i]=in_window[i*8+:8];
    end endgenerate
    reg [9:0] r0,r1,r2;
    reg [11:0] sum;
    reg [2:0] tag0,tag1;
    reg [31:0] xy0,xy1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r0<=0; r1<=0; r2<=0; sum<=0;
            tag0<=0; tag1<=0; xy0<=0; xy1<=0;
            out_valid<=0; out_sof<=0; out_eof<=0;
            out_x<=0; out_y<=0; out_gray<=0;
        end else begin
            r0 <= {2'b0,p[0]}+{1'b0,p[1],1'b0}+{2'b0,p[2]};
            r1 <= {2'b0,p[3]}+{1'b0,p[4],1'b0}+{2'b0,p[5]};
            r2 <= {2'b0,p[6]}+{1'b0,p[7],1'b0}+{2'b0,p[8]};
            sum <= {2'b0,r0}+{1'b0,r1,1'b0}+{2'b0,r2}+12'd8;
            out_gray <= sum[11:4];
            tag0<={in_valid,in_sof,in_eof}; tag1<=tag0;
            {out_valid,out_sof,out_eof}<=tag1;
            xy0<={in_x,in_y}; xy1<=xy0; {out_x,out_y}<=xy1;
        end
    end
endmodule
