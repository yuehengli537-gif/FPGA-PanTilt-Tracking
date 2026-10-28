`timescale 1ns/1ps
// Scene geometry is defined independently of the detector implementation.
// Tests object discrimination, bounded resource failure, and stream contracts.
module tb_red_ball_shape;
    localparam integer W=160,H=120, PUBLICATION_LATENCY=12;
    reg clk=0,rst_n=0,de=0,vs=0;
    always #5 clk=~clk;
    reg [15:0] rgb=0;
    reg search_enable=0;
    reg [11:0] search_left=0,search_top=0,search_right=W-1,search_bottom=H-1;
    wire update,valid;
    wire [11:0] left,top,right,bottom,cx,cy;
    wire [19:0] count;
    red_ball_detector #(.WIDTH(W),.HEIGHT(H),.MAX_DIAMETER(80)) dut(
        .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_vs(vs),
        .search_enable(search_enable),.search_left(search_left),.search_top(search_top),
        .search_right(search_right),.search_bottom(search_bottom),.frame_update(update),
        .target_valid(valid),.bbox_left(left),.bbox_top(top),.bbox_right(right),
        .bbox_bottom(bottom),.center_x(cx),.center_y(cy),.pixel_count(count));
    wire [92:0] published={valid,left,top,right,bottom,cx,cy,count};
    reg [92:0] frozen,prior_complete;
    integer checks=0,frames=0,scene=-1,px,py,trace,k;
    reg [15:0] pixel;
    function integer disk(input integer x,y,bx,by,r);
        disk=(x-bx)*(x-bx)+(y-by)*(y-by)<=r*r;
    endfunction
    function integer rect(input integer x,y,l,t,r,b);
        rect=x>=l && x<=r && y>=t && y<=b;
    endfunction
    function integer absolute(input integer value);
        absolute=value<0?-value:value;
    endfunction
    function [15:0] scene_pixel(input integer which,x,y);
        integer j;
        begin
            scene_pixel=16'h8410;
            case(which)
                0: if(disk(x,y,45,60,10))scene_pixel=16'hf800;
                1: if(rect(x,y,25,44,56,75))scene_pixel=16'hf800;
                2: if(rect(x,y,30,52,100,64))scene_pixel=16'hf800;
                3: if(rect(x,y,50,22,62,92))scene_pixel=16'hf800;
                4: begin
                    if(disk(x,y,38,60,10))scene_pixel=16'hf800;
                    if(rect(x,y,90,35,140,85))scene_pixel=16'hf800;
                end
                5: begin
                    if(rect(x,y,8,35,58,85))scene_pixel=16'hf800;
                    if(disk(x,y,112,60,10))scene_pixel=16'hf800;
                end
                6: begin
                    if(disk(x,y,38,60,10))scene_pixel=16'hf800;
                    if(x>=90)scene_pixel=16'hf800;
                end
                7: begin
                    if(disk(x,y,45,60,10))scene_pixel=16'hf800;
                    if(disk(x,y,48,57,3))scene_pixel=16'hffff;
                end
                8: begin
                    if(disk(x,y,20,60,10))scene_pixel=16'hf800;
                    if(disk(x,y,57,60,15))scene_pixel=16'h07e0;
                    if(disk(x,y,96,60,15))scene_pixel=16'h001f;
                    if(disk(x,y,135,60,15))scene_pixel=16'hffe0;
                end
                9: if(disk(x,y,60,60,18) && !disk(x,y,60,60,14))scene_pixel=16'hf800;
                10: if(rect(x,y,40,50,79,61) || rect(x,y,54,36,65,75))scene_pixel=16'hf800;
                11: if(rect(x,y,40,40,48,79) || rect(x,y,40,71,79,79))scene_pixel=16'hf800;
                12: begin
                    if(disk(x,y,35,60,8))scene_pixel=16'hf800;
                    if(disk(x,y,112,60,12))scene_pixel=16'hf800;
                end
                13: if(disk(x,y,10,40,10))scene_pixel=16'hf800;
                14: if(disk(x,y,W-11,80,10))scene_pixel=16'hf800;
                15: if(disk(x,y,80,H-11,10))scene_pixel=16'hf800;
                16: if(disk(x,y,W-3,H-3,12))scene_pixel=16'hf800;
                17: scene_pixel=16'hf800;
                18: if(disk(x,y,45,60,4))scene_pixel=16'hf800;
                19: if(disk(x,y,45,60,10))scene_pixel=16'h6104;
                20: for(j=0;j<5;j=j+1)begin
                    if(disk(x,y,16+30*j,60,8))scene_pixel=16'hf800;
                end
                21: scene_pixel=16'h07e0;
                22: if(disk(x,y,45,60,10) && x>=45)scene_pixel=16'hf800;
                23: begin
                    if(disk(x,y,35,60,10))scene_pixel=16'hf800;
                    if(disk(x,y,112,60,12))scene_pixel=16'hf800;
                end
                24: if(disk(x,y,80,10,10))scene_pixel=16'hf800;
                25: begin
                    if(disk(x,y,45,60,10))scene_pixel=16'hf800;
                    if(x==2 && y%5==0)scene_pixel=16'hf800;
                    if(y==5 && x>=3 && x<=5)scene_pixel=16'hf800;
                end
                // Near-square hollow outline cannot qualify by outer bounds alone.
                26: if(rect(x,y,40,40,79,79) && !rect(x,y,45,45,74,74))scene_pixel=16'hf800;
                // Close at the last column, then one column earlier. The latter
                // leaves a pending run association immediately before row_ready.
                27: if(disk(x,y,W-12,60,10))scene_pixel=16'hf800;
                28: if(disk(x,y,W-13,60,10))scene_pixel=16'hf800;
                // A real target still has a >=MIN_RUN span in the very last row;
                // the final pixel is immediately followed by VS (no line gap).
                29: if(disk(x,y,45,H-10,10))scene_pixel=16'hf800;
                // Exactly four simultaneous round candidates fit the budget.
                30: for(j=0;j<4;j=j+1)begin
                    if(disk(x,y,20+40*j,60,j==3?12:8))scene_pixel=16'hf800;
                end
                // Eight total regions, only four simultaneous: slots must retire
                // and be reused. Largest candidate occurs late, then early.
                31,32: for(j=0;j<4;j=j+1)begin
                    if(disk(x,y,20+40*j,25,(which==32 && j==0)?12:8))scene_pixel=16'hf800;
                    if(disk(x,y,20+40*j,85,(which==31 && j==3)?12:8))scene_pixel=16'hf800;
                end
                // Pole at y=-1 is absent, but y=0 contains nine real red pixels.
                // This distinguishes first-row ROI gating from a one-pixel pole
                // already discarded by MIN_RUN in an ordinary touching circle.
                33: if(disk(x,y,45,9,10))scene_pixel=16'hf800;
                default: scene_pixel=16'h8410;
            endcase
        end
    endfunction
    task tick(input [15:0] p,input bit pd,pv);
        begin @(negedge clk);rgb=p;de=pd;vs=pv;@(posedge clk);#1;end
    endtask
    task boundary(input bit expect_update);
        integer stage;
        begin
            // Raw VS sampled at N publishes exactly at N+12, including a
            // completed red run in the immediately preceding last pixel.
            for(stage=0;stage<=PUBLICATION_LATENCY;stage=stage+1)begin
                tick(0,0,stage<2);
                if(update!==((stage==PUBLICATION_LATENCY)?expect_update:1'b0))
                    $fatal(1,"publication stage%0d expected latency%0d",stage,PUBLICATION_LATENCY);
            end
            checks=checks+1;
            tick(0,0,0);if(update)$fatal(1,"publication must be one clock");
            frozen=published;
        end
    endtask
    task roi(input bit enabled,input integer l,t,r,b);
        begin search_enable=enabled;search_left=l;search_top=t;search_right=r;search_bottom=b;end
    endtask
    task draw(input integer which,input bit continuous,input integer change_roi);
        begin
            scene=which;
            for(py=0;py<H;py=py+1)begin
                for(px=0;px<W;px=px+1)begin
                    if(change_roi!=0 && py==5 && px==20)begin
                        if(change_roi==1)roi(1,95,40,130,80);
                        else roi(0,0,0,W-1,H-1);
                    end
                    pixel=scene_pixel(which,px,py);
                    tick(pixel,1,0);
                    if(update || published!==frozen)$fatal(1,"active-frame metadata change scene%0d at%0d,%0d",which,px,py);
                    if(!continuous && px==W/3 && py%7==0)begin
                        tick(16'hf800,0,0);
                        if(update || published!==frozen)$fatal(1,"DE gap became a pixel");
                    end
                end
                if(!continuous)repeat(3)tick(16'hf800,0,0);
            end
            boundary(1);frames=frames+1;
        end
    endtask
    task found(input integer x,y,radius);
        begin
            if(!valid || absolute(cx-x)>1 || absolute(cy-y)>1 ||
               absolute(left-(x-radius))>1 || absolute(right-(x+radius))>1 ||
               absolute(top-(y-radius))>1 || absolute(bottom-(y+radius))>1 ||
               count<100 || count>(2*radius+1)*(2*radius+1))
                $fatal(1,"scene%0d expected circle center%0d,%0d R%0d got valid%0d bbox%0d,%0d..%0d,%0d center%0d,%0d count%0d",scene,x,y,radius,valid,left,top,right,bottom,cx,cy,count);
            checks=checks+1;
            $fdisplay(trace,"%0d,1,%0d,%0d,%0d,%0d,%0d,%0d",scene,x,y,valid,cx,cy,count);
        end
    endtask
    task absent;
        begin
            if(published!==0)$fatal(1,"scene%0d must not lock non-ball/overflow: %h",scene,published);
            checks=checks+1;
            $fdisplay(trace,"%0d,0,0,0,%0d,%0d,%0d,%0d",scene,valid,cx,cy,count);
        end
    endtask
    initial begin
        trace=$fopen("shape_trace.csv","w");
        if(!trace)$fatal(1,"cannot open shape trace");
        $fdisplay(trace,"scene,expected_valid,expected_x,expected_y,valid,center_x,center_y,pixels");
        repeat(3)tick(0,0,0);rst_n=1;
        repeat(20)tick(16'hf800,1,0);boundary(0);
        draw(0,0,0);found(45,60,10);
        draw(1,0,0);absent();
        draw(2,0,0);absent();
        draw(3,0,0);absent();
        draw(4,0,0);found(38,60,10);
        draw(5,0,0);found(112,60,10);
        draw(6,0,0);found(38,60,10);
        draw(7,0,0);found(45,60,10);
        draw(8,0,0);found(20,60,10);
        draw(9,0,0);absent();
        draw(10,0,0);absent();
        draw(11,0,0);absent();
        draw(12,0,0);found(112,60,12);
        draw(13,0,0);found(10,40,10);
        draw(14,1,0);found(W-11,80,10);
        draw(15,1,0);found(80,H-11,10);
        draw(16,1,0);absent();
        draw(17,1,0);absent();
        draw(18,0,0);absent();
        draw(19,0,0);found(45,60,10);
        draw(20,0,0);absent(); // Five simultaneous eligible objects exceed capacity.
        draw(0,0,0);found(45,60,10); // Overflow must not poison following frames.
        draw(21,0,0);absent();
        draw(22,0,0);absent(); // Partial sphere is uncertain, tracker must HOLD.
        draw(24,0,0);found(80,10,10);
        draw(25,0,0);found(45,60,10);
        draw(26,0,0);absent();
        draw(27,1,0);found(W-12,60,10);
        draw(28,1,0);found(W-13,60,10);
        draw(29,1,0);found(45,H-10,10);
        draw(30,0,0);found(140,60,12);
        draw(31,0,0);found(140,85,12);
        draw(32,0,0);found(20,25,12);
        draw(33,1,0);found(45,9,10);
        if(top!=0 || bottom!=18 || count!=315)
            $fatal(1,"first raster row was omitted or duplicated count%0d top%0d bottom%0d",count,top,bottom);
        checks=checks+1;
        // An exact inclusive ROI retains all 315 pixels in the disk's 19
        // qualifying rows. An off-by-one at any ROI edge changes that total.
        roi(1,35,0,55,18);boundary(0);
        draw(33,1,0);found(45,9,10);
        if(left!=35 || right!=55 || top!=0 || bottom!=18 || count!=315)
            $fatal(1,"inclusive first-row ROI changed disk geometry or pixel count");
        checks=checks+1;
        // ROI is latched at VS. Live updates cannot split one frame's decision.
        roi(1,20,40,50,80);boundary(0);
        draw(23,0,1);found(35,60,10);
        draw(23,0,0);found(112,60,12);
        roi(1,20,40,50,80);boundary(0);
        draw(23,0,2);found(35,60,10);
        draw(23,0,0);found(112,60,12);
        // Cropping a red wall to a square search window is not evidence of a ball.
        roi(1,30,40,60,70);boundary(0);
        draw(17,1,0);absent();
        roi(1,60,40,30,70);boundary(0);
        draw(23,0,0);absent();
        roi(1,0,80,W-1,20);boundary(0);
        draw(23,0,0);absent(); // Reversed vertical bounds are empty too.
        roi(0,0,0,W-1,H-1);boundary(0);
        draw(0,0,0);found(45,60,10);
        // An incomplete frame cannot publish over a trusted result.
        prior_complete=published;
        repeat(W*4)tick(16'hf800,1,0);boundary(0);
        if(published!==prior_complete)$fatal(1,"incomplete frame replaced target");
        @(negedge clk);rst_n=0;#1;
        if(published!==0 || update)$fatal(1,"reset failed to clear detection");
        @(negedge clk);rst_n=1;
        repeat(W*2)tick(16'hf800,1,0);boundary(0);absent();
        draw(0,0,0);found(45,60,10);
        $fclose(trace);
        $display("PASS red_ball_shape: %0d checks, %0d complete frames; squares/stripes/rings/cross/L rejection, same-row larger distractors, colors, highlight, bounds, overflow, ROI freeze, DE gaps, incomplete frames and reset",checks,frames);
        $display("LIMITATION: circular flat objects can pass; this is a 2-D geometric classifier, not semantic sphere recognition.");
        $finish;
    end
    initial begin #1000000000;$fatal(1,"shape bench timeout");end
endmodule
