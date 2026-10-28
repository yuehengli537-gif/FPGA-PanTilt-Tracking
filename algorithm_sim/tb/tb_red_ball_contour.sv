`timescale 1ns/1ps
// Independent visible silhouettes, never DUT-internal shape scores.
// Regression for a flat square with highlight/edge dropout passing V6d.
module tb_red_ball_contour #(parameter integer CANDIDATES=4);
    localparam integer W=256,H=160,PUBLICATION_LATENCY=12;
    reg clk=0,rst_n=0,de=0,vs=0;
    reg [15:0] rgb=0;
    always #5 clk=~clk;
    wire update,valid;
    wire [11:0] left,top,right,bottom,cx,cy;
    wire [19:0] count;
    wire [7:0] diag_flags;
    wire [92:0] published={valid,left,top,right,bottom,cx,cy,count};
    reg [92:0] frozen;
    integer scene=-1,checks=0,frames=0,trace,px,py,k;
    red_ball_detector #(.WIDTH(W),.HEIGHT(H),.MAX_DIAMETER(128),.CANDIDATES(CANDIDATES)) dut(
        .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_vs(vs),
        .search_enable(1'b0),.search_left(12'd0),.search_top(12'd0),
        .search_right(12'd255),.search_bottom(12'd159),
        .frame_update(update),.target_valid(valid),.bbox_left(left),.bbox_top(top),
        .bbox_right(right),.bbox_bottom(bottom),.center_x(cx),.center_y(cy),.pixel_count(count),
        .diag_flags(diag_flags));

    function integer abs_i(input integer v);
        abs_i=v<0?-v:v;
    endfunction
    function integer disk(input integer x,y,bx,by,r);
        disk=(x-bx)*(x-bx)+(y-by)*(y-by)<=r*r;
    endfunction
    function integer rect(input integer x,y,l,t,r,b);
        rect=x>=l && x<=r && y>=t && y<=b;
    endfunction
    // Fixed-point rotations of an actual square; no trigonometry in the DUT.
    function integer rotated_square(input integer x,y,bx,by,r,c,s);
        integer u,v;
        begin
            u=(x-bx)*c+(y-by)*s;v=-(x-bx)*s+(y-by)*c;
            rotated_square=abs_i(u)<=1000*r && abs_i(v)<=1000*r;
        end
    endfunction
    function integer damaged_square(input integer x,y,bx,by,r,notch);
        integer dx,dy;
        begin
            dx=x-bx;dy=y-by;
            // Physical square remains visible: the color mask loses central
            // glare and two rows of corner color. Fill/taper alone accept it.
            damaged_square=abs_i(dx)<=r && abs_i(dy)<=r &&
                dx*dx+dy*dy>(r/2)*(r/2) &&
                !(abs_i(dy)>r-2 && abs_i(dx)>(2*r)/3) &&
                !(notch && dx>r-7 && abs_i(dy-8)<=5);
        end
    endfunction
    function integer glossy_ball(input integer x,y,bx,by,r,finger);
        begin
            glossy_ball=(disk(x,y,bx,by,r) && !disk(x,y,bx+r/4,by-r/4,r/4)) ||
                (finger && rect(x,y,bx+r-4,by-3,bx+r+3,by+3));
        end
    endfunction
    function [15:0] scene_pixel(input integer which,x,y);
        integer hit,j;
        begin
            hit=0;
            case(which)
                0: hit=disk(x,y,70,76,28);
                1: hit=damaged_square(x,y,80,75,36,0);
                2: hit=damaged_square(x,y,170,80,32,1);
                3: hit=glossy_ball(x,y,70,76,28,0);
                4: hit=glossy_ball(x,y,150,76,28,1);
                // A larger wrong shape must not mask the smaller true target.
                5: hit=disk(x,y,45,80,18) || damaged_square(x,y,175,80,36,0);
                6: hit=damaged_square(x,y,75,80,36,1) || disk(x,y,205,80,18);
                7: hit=rotated_square(x,y,90,80,28,996,87); // 5 deg
                8: hit=rotated_square(x,y,150,80,28,966,259); // 15 deg
                9: hit=rotated_square(x,y,100,80,28,906,423); // 25 deg
                10: hit=rotated_square(x,y,150,80,28,707,707); // 45 deg
                11: hit=glossy_ball(x,y,28,60,28,0); // left edge
                12: hit=glossy_ball(x,y,W-29,90,28,0); // right edge
                13: hit=glossy_ball(x,y,100,28,28,0); // top edge
                14: hit=glossy_ball(x,y,160,H-29,28,0); // bottom edge
                15: hit=disk(x,y,45,H-28,28); // final row contains a run
                16: hit=damaged_square(x,y,36,75,36,0); // bad shape at left edge
                17: hit=damaged_square(x,y,W-37,80,36,0); // right edge
                18: hit=disk(x,y,70,75,12); // small legitimate target
                19: hit=glossy_ball(x,y,125,75,40,1); // observed ~80px object
                // Slot retirement/reuse: the valid smaller ball is later.
                20: hit=damaged_square(x,y,60,36,28,0) || disk(x,y,180,126,18);
                21: hit=disk(x,y,60,35,18) || damaged_square(x,y,180,118,28,0);
                22: begin
                    for(j=0;j<4;j=j+1)
                        hit=hit || damaged_square(x,y,30+64*j,35,20,0);
                    hit=hit || disk(x,y,128,122,20);
                end
                // A top-reaching highlight creates two initially separate
                // ball lobes, beside one already-active plain square. Three
                // slots must merge the lobes and still select the real ball.
                24: hit=(disk(x,y,60,80,28) && !rect(x,y,56,0,65,66)) ||
                    rect(x,y,144,44,216,116);
                default: hit=0;
            endcase
            scene_pixel=hit?16'hf800:16'h8410;
        end
    endfunction
    task tick(input [15:0] p,input bit pd,pv);
        begin @(negedge clk);rgb=p;de=pd;vs=pv;@(posedge clk);#1;end
    endtask
    task boundary(input bit expected);
        integer stage;
        begin
            for(stage=0;stage<=PUBLICATION_LATENCY;stage=stage+1)begin
                tick(0,0,stage<2);
                if(update!==((stage==PUBLICATION_LATENCY)?expected:1'b0))
                    $fatal(1,"contour scene%0d publication stage%0d expected N+12",scene,stage);
            end
            tick(0,0,0);
            if(update)$fatal(1,"contour publication longer than one clock");
            frozen=published;checks=checks+1;
        end
    endtask
    task draw(input integer which,input bit continuous);
        begin
            scene=which;
            for(py=0;py<H;py=py+1)begin
                for(px=0;px<W;px=px+1)begin
                    tick(scene_pixel(which,px,py),1,0);
                    if(update || published!==frozen)
                        $fatal(1,"contour scene%0d changed metadata during frame",which);
                    if(!continuous && px==W/3 && py%9==0)begin
                        tick(16'hf800,0,0);
                        if(update || published!==frozen)$fatal(1,"contour DE bubble changed metadata");
                    end
                end
                if(!continuous)repeat(3)tick(16'hf800,0,0);
            end
            boundary(1);frames=frames+1;
        end
    endtask
    task found(input integer expected_x,expected_y,tolerance);
        begin
            $fdisplay(trace,"%0d,1,%0d,%0d,%0d,%0d,%0d,%0d",scene,expected_x,expected_y,valid,cx,cy,count);
            if(!valid || abs_i(cx-expected_x)>tolerance || abs_i(cy-expected_y)>tolerance || count<100)
                $fatal(1,"contour scene%0d expected ball near%0d,%0d got valid%0d center%0d,%0d count%0d",scene,expected_x,expected_y,valid,cx,cy,count);
            checks=checks+1;
        end
    endtask
    task absent;
        begin
            $fdisplay(trace,"%0d,0,0,0,%0d,%0d,%0d,%0d",scene,valid,cx,cy,count);
            if(published!==0)
                $fatal(1,"contour scene%0d non-ball accepted: valid%0d bbox%0d,%0d..%0d,%0d center%0d,%0d pixels%0d",scene,valid,left,top,right,bottom,cx,cy,count);
            checks=checks+1;
        end
    endtask
    initial begin
        trace=$fopen("contour_trace.csv","w");
        if(!trace)$fatal(1,"cannot open contour trace");
        $fdisplay(trace,"scene,expected_valid,expected_x,expected_y,valid,center_x,center_y,pixels");
        repeat(3)tick(0,0,0);rst_n=1;boundary(0);
        draw(0,0);found(70,76,1);
        draw(1,0);absent();
        draw(2,1);absent();
        draw(3,0);found(70,76,1);
        draw(4,1);found(151,76,2);
        draw(5,0);found(45,80,1);
        draw(6,1);found(205,80,1);
        for(k=7;k<=10;k=k+1)begin draw(k,1);absent();end
        draw(11,0);found(28,60,1);
        draw(12,1);found(W-29,90,1);
        draw(13,0);found(100,28,1);
        draw(14,1);found(160,H-29,1);
        draw(15,1);found(45,H-28,1);
        draw(16,0);absent();
        draw(17,1);absent();
        draw(18,0);found(70,75,1);
        draw(19,1);found(126,75,2);
        draw(20,1);found(180,126,1);
        draw(21,0);found(60,35,1);
        draw(22,1);
        if(CANDIDATES<4)begin
            // Four simultaneously visible regions exceed the configured bank.
            // Even a later good ball cannot make that incomplete frame trusted.
            absent();
            if(!diag_flags[1])$fatal(1,"contour limited-bank overflow flag missing");
            checks=checks+1;
        end else found(128,122,1);
        draw(23,1);absent();
        draw(24,1);found(60,80,1);
        $fclose(trace);
        $display("PASS red_ball_contour: %0d checks, %0d frames, %0d candidate slots; damaged flat squares, rotations, glossy balls, light finger attachment, larger distractors, slot reuse/overflow, edges, DE bubbles and N+12 publication",checks,frames,CANDIDATES);
        $display("LIMITATION: synthetic silhouette regressions do not prove 3-D sphere semantics or arbitrary illumination tolerance.");
        $finish;
    end
    initial begin #1000000000;$fatal(1,"contour bench timeout");end
endmodule
