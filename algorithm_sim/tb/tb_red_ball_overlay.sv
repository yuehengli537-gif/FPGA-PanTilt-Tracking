`timescale 1ns/1ps
module tb_red_ball_overlay;
    localparam W=160,H=128;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0,in_de=0,in_hs=0,in_vs=0,target_valid=0;
    reg [1:0] track_state=0;
    reg [7:0] in_r=8'h12,in_g=8'h34,in_b=8'h56;
    reg [11:0] bbox_left=90,bbox_top=85,bbox_right=110,bbox_bottom=105;
    reg [11:0] center_x=100,center_y=95;
    wire [7:0] out_r,out_g,out_b;
    wire out_de,out_hs,out_vs;
    integer checks=0;
    // Disabled diagnostics must ignore unknown/unconnected diagnostic inputs.
    red_ball_overlay #(.WIDTH(W),.HEIGHT(H),.DEAD_ZONE(8)) dut(
        .diag_candidate_valid(1'bx), .diag_flags(8'bx), .diag_candidate_flags(8'bx),
        .diag_width(13'bx), .diag_height(13'bx), .diag_pixels(20'bx),
        .diag_left(12'bx), .diag_top(12'bx), .diag_right(12'bx), .diag_bottom(12'bx),
        .servo_pan_us(16'bx), .servo_tilt_us(16'bx),
        .servo_pan_limit(2'bx), .servo_tilt_limit(2'bx),
        .servo_enabled(1'bx), .servo_source_alive(1'bx), .servo_stale(1'bx), .*);

    task step(input bit de,hs,vs);
        begin
            @(negedge clk);
            in_de=de; in_hs=hs; in_vs=vs;
            @(posedge clk); #1;
            if({out_de,out_hs,out_vs}!=={de,hs,vs})
                $fatal(1,"RGB control latency mismatch at check %0d",checks);
            if(!de && {out_r,out_g,out_b}!==24'h000000)
                $fatal(1,"Non-black blanking");
            checks=checks+1;
        end
    endtask
    task pixel_is(input [23:0] rgb,input integer px,py,frame);
        begin
            if({out_r,out_g,out_b}!==rgb)
                $fatal(1,"frame=%0d xy=%0d,%0d expected=%h actual=%h",frame,px,py,rgb,{out_r,out_g,out_b});
        end
    endtask
    task frame(input integer f);
        integer px,py;
        begin
            step(0,0,1); step(0,1,1); step(0,0,0);
            for(py=0;py<H;py=py+1) begin
                for(px=0;px<W;px=px+1) begin
                    // Change input metadata during an active frame. It must
                    // affect neither this frame's target box nor aim status.
                    if(f==0 && py==70 && px==0) begin
                        bbox_left=10; bbox_right=30; center_x=20;
                        track_state=3;
                    end
                    step(1,0,0);
                    if(px==150 && py==120) pixel_is(24'h123456,px,py,f);
                    if(px==8 && py==8) pixel_is(24'hc8f0ff,px,py,f);
                    if(px==80 && py==64)
                        pixel_is((f==2 || f>=4)?24'h606060:f==3?24'h20ff40:24'he0e0e0,px,py,f);
                    if(px==90 && py==85) pixel_is(f==0?24'h20ff40:24'h123456,px,py,f);
                    if(px==10 && py==85) pixel_is(f==1?24'h20ff40:24'h123456,px,py,f);
                    if(px==100 && py==95) pixel_is(f==0?24'hffff00:24'h123456,px,py,f);
                    if(px==20 && py==95) pixel_is(f==1?24'hffff00:24'h123456,px,py,f);
                    if(px==88 && py==68) pixel_is((f==2 || f>=4)?24'h606060:f==3?24'h20ff40:24'he0e0e0,px,py,f);
                    if(px==108 && py==64) pixel_is(f==0?24'hffff00:24'h123456,px,py,f);
                    if(px==52 && py==64) pixel_is(f==1?24'hffff00:24'h123456,px,py,f);
                    if(px==80 && py==92) pixel_is(f<2?24'hffff00:24'h123456,px,py,f);
                    if(px==90 && py==88) pixel_is(f==0?24'h20ff40:f==4?24'hffff00:24'h123456,px,py,f);
                    if(px==90 && py==86) pixel_is(f==0?24'h20ff40:24'h123456,px,py,f);
                    if(px==96 && py==85) pixel_is(f==0?24'h20ff40:f==4?24'hffff00:24'h123456,px,py,f);
                    if(px==100 && py==85) pixel_is(f==0?24'h20ff40:24'h123456,px,py,f);
                    if(f>=4 && px==8 && py==24) pixel_is(f==4?24'hffff00:f==5?24'h081820:24'h606060,px,py,f);
                    if(f>=4 && px==10 && py==24) pixel_is(f==5?24'h606060:24'h081820,px,py,f);
                end
                step(0,1,0); step(0,0,0);
            end
        end
    endtask
    initial begin
        repeat(3) @(posedge clk);
        @(negedge clk); rst_n=1; target_valid=1; track_state=2;
        frame(0); // original box remains after active-frame input update
        track_state=2;
        frame(1); // VS transfers new box and center
        target_valid=0; track_state=0;
        frame(2); // lost target clears stale bounding box and yellow cross
        target_valid=1; track_state=2; center_x=80; center_y=64;
        bbox_left=75; bbox_right=85; bbox_top=59; bbox_bottom=69;
        frame(3); // centered indication
        bbox_left=90; bbox_right=110; bbox_top=85; bbox_bottom=105;
        center_x=100; center_y=95;
        target_valid=0; track_state=3;frame(4);
        target_valid=1; track_state=1;frame(5);
        target_valid=1; track_state=0;frame(6);
        @(negedge clk); rst_n=0;
        #1;
        if({out_de,out_hs,out_vs,out_r,out_g,out_b}!==0) $fatal(1,"reset output mismatch");
        $display("PASS red_ball_overlay: %0d clock checks, frame snapshots, lost, centered and pixel alignment",checks);
        $finish;
    end
endmodule
