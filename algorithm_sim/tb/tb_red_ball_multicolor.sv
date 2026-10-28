`timescale 1ns/1ps
// Concrete RGB565 samples, not a guarantee over all real-world color names.
// Ten disks share a frame; every non-red disk is larger than the red disk.
module tb_red_ball_multicolor;
    localparam integer W=320,H=180,R=12,PUBLICATION_LATENCY=12;
    reg clk=0,rst_n=0,de=0,vs=0;
    reg [15:0] rgb=0;
    always #5 clk=~clk;
    wire update,valid;
    wire [11:0] left,top,right,bottom,cx,cy;
    wire [19:0] count;
    red_ball_detector #(.WIDTH(W),.HEIGHT(H)) dut(
        .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_vs(vs),
        .search_enable(1'b0),.search_left(12'd0),.search_top(12'd0),
        .search_right(12'd0),.search_bottom(12'd0),
        .frame_update(update),.target_valid(valid),
        .bbox_left(left),.bbox_top(top),.bbox_right(right),.bbox_bottom(bottom),
        .center_x(cx),.center_y(cy),.pixel_count(count));
    integer scene,px,py,k,index,slot,bx,by,radius,tx,ty,checks=0,trace;
    reg [15:0] pixel,color;
    function [15:0] palette(input integer i);
        case(i)
            0:palette=16'hf800; // saturated red
            1:palette=16'h07e0; // green
            2:palette=16'h001f; // blue
            3:palette=16'hffe0; // yellow
            4:palette=16'h07ff; // cyan
            5:palette=16'h8010; // purple
            6:palette=16'hffff; // white
            7:palette=16'h0000; // black
            8:palette=16'hfc00; // orange sample (255,130,0)
            9:palette=16'hfd18; // pink sample (255,162,198)
            default:palette=16'h8410;
        endcase
    endfunction
    function integer disk(input integer x,y,cx0,cy0,rad);
        disk=(x-cx0)*(x-cx0)+(y-cy0)*(y-cy0)<=rad*rad;
    endfunction
    function integer absval(input integer v); absval=v<0?-v:v;endfunction
    task tick(input [15:0] p,input bit pd,pv);
        begin @(negedge clk);rgb=p;de=pd;vs=pv;@(posedge clk);#1;end
    endtask
    task boundary(input bit expected);
        integer stage;
        begin
            for(stage=0;stage<=PUBLICATION_LATENCY;stage=stage+1)begin
                tick(0,0,stage<2);
                if(update!==((stage==PUBLICATION_LATENCY)?expected:1'b0))
                    $fatal(1,"publication stage%0d expected latency%0d",stage,PUBLICATION_LATENCY);
            end
            checks=checks+1;
            tick(0,0,0);if(update)$fatal(1,"wide update");
        end
    endtask
    initial begin
        trace=$fopen("multicolor_trace.csv","w");
        if(!trace) $fatal(1,"cannot write multicolor trace");
        $fdisplay(trace,"scene,target_slot,expected_x,expected_y,valid,center_x,center_y,pixels");
        repeat(3) tick(0,0,0);rst_n=1;boundary(0);
        for(scene=0;scene<16;scene=scene+1) begin
            slot=scene<10?scene:4;tx=(slot%5)*64+32;ty=(slot/5)*90+45;
            for(py=0;py<H;py=py+1) begin
                for(px=0;px<W;px=px+1) begin
                    pixel=16'h8410;
                    for(k=0;k<10;k=k+1) begin
                        index=(k-slot+10)%10;
                        bx=(k%5)*64+32;by=(k/5)*90+45;
                        radius=index==0?R:24;
                        color=palette(index);
                        if(index==0) begin
                            case(scene)
                                10:color=16'h8410; // no red, all nine distractor colors remain
                                11:color=16'h6104; // dim saturated red (99,32,33)
                                13:color=16'h4800; // red below brightness floor
                                14:color=16'hfbe0; // known orange-red ambiguity (255,125,0)
                                15:color=16'hfa0c; // known red-pink ambiguity (255,65,99)
                            endcase
                        end
                        if(disk(px,py,bx,by,radius)) pixel=color;
                    end
                    if(scene==12 && disk(px,py,tx+2,ty-2,3)) pixel=16'hffff;
                    tick(pixel,1,0);
                    if(update) $fatal(1,"metadata published inside active frame");
                end
                repeat(3) tick(0,0,0);
            end
            boundary(1);
            if(scene==10 || scene==13) begin
                if({valid,left,top,right,bottom,cx,cy,count}!==0)
                    $fatal(1,"no valid red expected scene%0d got center%0d,%0d",scene,cx,cy);
            end else begin
                if(!valid || absval(cx-tx)>(scene==12?1:0) || absval(cy-ty)>(scene==12?1:0) ||
                    absval(left-(tx-R))>1 || absval(right-(tx+R))>1 ||
                    absval(top-(ty-R+1))>1 || absval(bottom-(ty+R-1))>1 || count<100)
                    $fatal(1,"wrong selected ball scene%0d got valid%0d bbox%0d,%0d..%0d,%0d center%0d,%0d expected%0d,%0d",scene,valid,left,top,right,bottom,cx,cy,tx,ty);
            end
            checks=checks+1;
            $fdisplay(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",scene,slot,tx,ty,valid,cx,cy,count);
        end
        $fclose(trace);
        $display("PASS multicolor: %0d checks, ten simultaneous colored disks, ten target positions, larger non-red distractors, no red, dim red and white highlight",checks);
        $display("KNOWN LIMITATION: RGB565 FBE0 orange-red and FA0C red-pink also pass. Below-threshold dark red 4800 is missed. These are synthetic samples, not real-camera precision measurements.");
        $finish;
    end
endmodule
