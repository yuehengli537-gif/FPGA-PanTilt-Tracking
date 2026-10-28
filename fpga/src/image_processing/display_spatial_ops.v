`timescale 1ns/1ps
// Full HDMI timing is delayed along with point and neighborhood operations.
// 0 RGB,1 gray,2 bright,3 Gaussian,4 Sobel,5 Gaussian->Sobel,6 split,7 auto.
module display_spatial_ops #(
    parameter integer WIDTH=1280, HEIGHT=720, H_TOTAL=1650,
    parameter integer BRIGHTNESS_OFFSET=24, EDGE_THRESHOLD=96,
    parameter integer DISPLAY_MODE=7, MODE_FRAMES=300
) (
    input wire clk,rst_n,
    input wire [15:0] in_rgb565,
    input wire in_de,in_hs,in_vs,
    output reg [7:0] out_r,out_g,out_b,
    output reg out_de,out_hs,out_vs,
    output wire [2:0] active_mode
);
    localparam integer BASE_DELAY=2*H_TOTAL+12;
    localparam integer FRAME_BITS=(MODE_FRAMES<2)?1:$clog2(MODE_FRAMES);
    reg [FRAME_BITS-1:0] frame_count;
    reg [2:0] auto_mode;
    reg vs_previous;
    reg [15:0] x,y;
    assign active_mode=(DISPLAY_MODE==7)?auto_mode:DISPLAY_MODE;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0; y<=0; vs_previous<=0; frame_count<=0; auto_mode<=0;
        end else begin
            vs_previous<=in_vs;
            if(in_vs && !vs_previous) begin
                x<=0; y<=0;
                if(frame_count==MODE_FRAMES-1) begin
                    frame_count<=0;
                    auto_mode<=(auto_mode==5)?0:auto_mode+1'b1;
                end else frame_count<=frame_count+1'b1;
            end else if(in_de) begin
                if(x==WIDTH-1) begin
                    x<=0; y<=(y==HEIGHT-1)?0:y+1'b1;
                end else x<=x+1'b1;
            end
        end
    end
    wire sof=in_de && x==0 && y==0;
    // Register the DDR output before gray arithmetic and line-RAM writes.
    reg [15:0] rgb_s0;
    reg de_s0,hs_s0,vs_s0,sof_s0;
    reg [2:0] mode_s0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb_s0<=0; de_s0<=0; hs_s0<=0; vs_s0<=0; sof_s0<=0; mode_s0<=0;
        end else begin
            rgb_s0<=in_rgb565; de_s0<=in_de; hs_s0<=in_hs; vs_s0<=in_vs;
            sof_s0<=sof; mode_s0<=pixel_mode;
        end
    end
    wire gauss_valid,gauss_sof,gauss_eof,sobel_valid,sobel_sof,sobel_eof,sobel_edge;
    wire smooth_valid,smooth_sof,smooth_eof,smooth_edge;
    wire [15:0] gauss_x,gauss_y,sobel_x,sobel_y,smooth_x,smooth_y;
    wire [7:0] gauss_gray,sobel_gray,smooth_gray;
    wire [10:0] sobel_magnitude,smooth_magnitude;
    spatial_filter_top #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.EDGE_THRESHOLD(EDGE_THRESHOLD)) filters(
        .clk(clk),.rst_n(rst_n),.in_valid(de_s0),.in_sof(sof_s0),.in_rgb565(rgb_s0),
        .gauss_valid(gauss_valid),.gauss_sof(gauss_sof),.gauss_eof(gauss_eof),
        .gauss_x(gauss_x),.gauss_y(gauss_y),.gauss_gray(gauss_gray),
        .sobel_valid(sobel_valid),.sobel_sof(sobel_sof),.sobel_eof(sobel_eof),
        .sobel_x(sobel_x),.sobel_y(sobel_y),.sobel_gray(sobel_gray),
        .sobel_magnitude(sobel_magnitude),.sobel_edge(sobel_edge),
        .smooth_valid(smooth_valid),.smooth_sof(smooth_sof),.smooth_eof(smooth_eof),
        .smooth_x(smooth_x),.smooth_y(smooth_y),.smooth_gray(smooth_gray),
        .smooth_magnitude(smooth_magnitude),.smooth_edge(smooth_edge));
    wire [8:0] gauss_aligned,sobel_aligned;
    stream_delay #(.BITS(9),.DELAY(H_TOTAL+6)) gauss_delay(
        .clk(clk),.rst_n(rst_n),.in_data({gauss_valid,gauss_gray}),.out_data(gauss_aligned));
    stream_delay #(.BITS(9),.DELAY(H_TOTAL+5)) sobel_delay(
        .clk(clk),.rst_n(rst_n),.in_data({sobel_valid,sobel_gray}),.out_data(sobel_aligned));
    // Each delay samples registered filter data one clock after it is updated.
    // Thus the delay values above include that input sampling relationship.
    wire [7:0] gray_now;
    rgb565_to_gray gray_converter(.in_rgb565(rgb_s0),.out_gray(gray_now));
    wire [29:0] point_aligned;
    reg [2:0] pixel_mode;
    always @* begin
        pixel_mode=active_mode;
        if(active_mode==6) begin
            if(x<WIDTH/3) pixel_mode=0;
            else if(x<(WIDTH*2)/3) pixel_mode=3;
            else pixel_mode=5;
        end
    end
    stream_delay #(.BITS(30),.DELAY(BASE_DELAY)) point_delay(
        .clk(clk),.rst_n(rst_n),
        .in_data({de_s0,hs_s0,vs_s0,mode_s0,gray_now,rgb_s0}),.out_data(point_aligned));
    wire [7:0] bright;
    brightness_adjust #(.OFFSET(BRIGHTNESS_OFFSET)) bright_converter(
        .in_gray(point_aligned[23:16]),.out_bright(bright));
    // Smooth Sobel is already at BASE_DELAY. This final register is common
    // to every mode and to DE/HS/VS. Including input capture, overall delay
    // is BASE_DELAY+2 clocks (3314 for 720p H_TOTAL=1650).
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            {out_r,out_g,out_b}<=0; out_de<=0; out_hs<=1; out_vs<=1;
        end else begin
            out_de<=point_aligned[29]; out_hs<=point_aligned[28]; out_vs<=point_aligned[27];
            {out_r,out_g,out_b}<=0;
            if(point_aligned[29]) begin
                case(point_aligned[26:24])
                    1: {out_r,out_g,out_b}<={3{point_aligned[23:16]}};
                    2: {out_r,out_g,out_b}<={3{bright}};
                    3: if(gauss_aligned[8]) {out_r,out_g,out_b}<={3{gauss_aligned[7:0]}};
                    4: if(sobel_aligned[8]) {out_r,out_g,out_b}<={3{sobel_aligned[7:0]}};
                    5: if(smooth_valid) {out_r,out_g,out_b}<={3{smooth_gray}};
                    default: {out_r,out_g,out_b}<={point_aligned[15:11],3'b0,point_aligned[10:5],2'b0,point_aligned[4:0],3'b0};
                endcase
            end
        end
    end
endmodule
