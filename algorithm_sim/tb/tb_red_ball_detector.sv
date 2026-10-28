`timescale 1ns/1ps
// Behavioral scenes have geometric expectations, independent of run bookkeeping.
// Run at 64x48 and a larger raster to check DE/blanking and coordinate handling.
module tb_red_ball_detector #(
    parameter integer WIDTH = 64,
    parameter integer HEIGHT = 48
);
    localparam integer PUBLICATION_LATENCY=12;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg [15:0] rgb = 0;
    reg de = 0, vs = 0;
    reg search_enable = 0;
    reg [11:0] search_left = 0, search_top = 0;
    reg [11:0] search_right = 0, search_bottom = 0;
    wire update, valid;
    wire [11:0] left, top, right, bottom, cx, cy;
    wire [19:0] count;
    red_ball_detector #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .MAX_DIAMETER(32)
    ) dut (
        .clk(clk), .rst_n(rst_n), .in_rgb565(rgb), .in_de(de), .in_vs(vs),
        .search_enable(search_enable), .search_left(search_left), .search_top(search_top),
        .search_right(search_right), .search_bottom(search_bottom),
        .frame_update(update), .target_valid(valid),
        .bbox_left(left), .bbox_top(top), .bbox_right(right), .bbox_bottom(bottom),
        .center_x(cx), .center_y(cy), .pixel_count(count)
    );

    integer checks = 0;
    integer px, py, scene, r2, d1, d2;
    reg [15:0] pixel;
    reg [92:0] frozen;
    wire [92:0] published = {valid,left,top,right,bottom,cx,cy,count};
    task tick;
        input [15:0] p;
        input pde, pvs;
        begin
            @(negedge clk); rgb = p; de = pde; vs = pvs;
            @(posedge clk); #1;
        end
    endtask
    task start_frame;
        input expected_update;
        integer stage;
        begin
            for(stage=0;stage<=PUBLICATION_LATENCY;stage=stage+1)begin
                tick(0,0,stage<2);
                if(update!==((stage==PUBLICATION_LATENCY)?expected_update:1'b0))
                    $fatal(1,"VS stage%0d expected publication latency%0d",stage,PUBLICATION_LATENCY);
            end
            checks = checks + 1;
            tick(0,0,0);
            if (update !== 0) $fatal(1,"frame_update wider than one cycle");
            frozen = published;
        end
    endtask
    function integer absval;
        input integer v;
        begin absval = v < 0 ? -v : v; end
    endfunction
    function integer disk;
        input integer x,y,dx,dy,radius;
        begin disk = ((x-dx)*(x-dx)+(y-dy)*(y-dy)) <= radius*radius; end
    endfunction
    task expect_target;
        input integer el,et,er,eb,ecx,ecy,tolerance;
        begin
            if (valid !== 1 || absval(left-el)>tolerance || absval(top-et)>tolerance ||
                absval(right-er)>tolerance || absval(bottom-eb)>tolerance ||
                absval(cx-ecx)>tolerance || absval(cy-ecy)>tolerance || count<100)
                $fatal(1,"scene %0d target got valid=%b bbox=%0d,%0d..%0d,%0d center=%0d,%0d count=%0d expected bbox=%0d,%0d..%0d,%0d center=%0d,%0d tol=%0d",
                    scene,valid,left,top,right,bottom,cx,cy,count,el,et,er,eb,ecx,ecy,tolerance);
            checks = checks + 1;
        end
    endtask
    task expect_lost;
        begin
            if (published !== 0) $fatal(1,"scene %0d should clear absent/invalid target, got %h",scene,published);
            checks = checks + 1;
        end
    endtask
    task set_search;
        input enabled;
        input integer sl,st,sr,sb;
        begin
            search_enable=enabled;
            search_left=sl; search_top=st; search_right=sr; search_bottom=sb;
        end
    endtask
    // Geometric circle fixture replaces the old accepted-square ROI fixture.
    task expect_circle;
        input integer x0,y0,radius;
        integer gx,gy,area;
        begin
            expect_target(x0-radius,y0-radius+1,x0+radius,y0+radius-1,x0,y0,0);
            area=0;
            for(gy=-radius+1;gy<radius;gy=gy+1)
                for(gx=-radius;gx<=radius;gx=gx+1)
                    if(gx*gx+gy*gy<=radius*radius) area=area+1;
            if(count!==area) $fatal(1,"circle scene %0d retained area %0d expected %0d",scene,count,area);
            checks=checks+1;
        end
    endtask
    task draw_frame;
        input integer which;
        begin
            scene = which;
            for (py=0; py<HEIGHT; py=py+1) begin
                for (px=0; px<WIDTH; px=px+1) begin
                    pixel = 16'h2104;
                    case (which)
                        // Centered red disk, plus isolated pixels and a short red run.
                        0: begin
                            if (disk(px,py,WIDTH/2,HEIGHT/2,8)) pixel=16'hf800;
                            if ((py==2 && px>=1 && px<=3) || (px==1 && py%7==0)) pixel=16'hf800;
                        end
                        // Green, blue, gray, white, yellow and magenta: none are red-dominant.
                        1: case ((px/8)%6)
                            0:pixel=16'h07e0; 1:pixel=16'h001f; 2:pixel=16'h8410;
                            3:pixel=16'hffff; 4:pixel=16'hffe0; 5:pixel=16'hf81f;
                        endcase
                        // Reacquire a moved disk.
                        2: if (disk(px,py,12,12,8)) pixel=16'hc800;
                        // Two vertically separated disks: strongest/larger disk wins.
                        3: begin
                            if (disk(px,py,12,9,7)) pixel=16'hf800;
                            if (disk(px,py,WIDTH-14,HEIGHT-13,10)) pixel=16'hf800;
                        end
                        // Last row and last pixel red: incomplete shape still flushes before VS.
                        4: if (disk(px,py,WIDTH-3,HEIGHT-3,12)) pixel=16'hf800;
                        // Severe left/top clipping is insufficient evidence for first lock.
                        5: if (disk(px,py,2,2,12)) pixel=16'hf800;
                        // One-pixel noise must not join into a broad target.
                        6: if ((px%3==0) && (py%2==0)) pixel=16'hf800;
                        // Too small, long thin stripe, and uniform oversized red background.
                        7: if (disk(px,py,WIDTH/2,HEIGHT/2,4)) pixel=16'hf800;
                        8: if (py>=5 && py<9) pixel=16'hf800;
                        9: pixel=16'hf800;
                        // Red below RED_MIN must not pass even with black green/blue.
                        10: pixel=16'h4800;
                        // Two horizontal round candidates: the larger complete disk wins.
                        11: begin
                            if (disk(px,py,10,HEIGHT/2,6)) pixel=16'hf800;
                            if (disk(px,py,WIDTH-14,HEIGHT/2,9)) pixel=16'hf800;
                        end
                        // Very red but bright green/blue must not wrap the threshold addition.
                        12: pixel = px%2 ? 16'hfff8 : 16'hcfff;
                        // Typical warm skin would pass an absolute red margin alone.
                        13: pixel=16'hcc6d; // expanded RGB = 206,142,107
                        14: pixel=16'hfc00; // orange RGB = 255,130,0
                        // Dim but saturated red still passes; include a white highlight.
                        15: begin
                            if (disk(px,py,WIDTH/2,HEIGHT/2,8)) pixel=16'h6104;
                            if (disk(px,py,WIDTH/2+2,HEIGHT/2-2,2)) pixel=16'hffff;
                        end
                        // Smaller tracked disk and a larger distant round distractor
                        // occupy the same rows. ROI must filter before row selection.
                        16,17,18: begin
                            if(disk(px,py,12,HEIGHT/2,8)) pixel=16'hf800;
                            if(disk(px,py,WIDTH-14,HEIGHT/2,10)) pixel=16'hf800;
                            // Change live controls well before either object is
                            // scanned. Frozen controls must still govern this frame.
                            if(px==WIDTH/2 && py==HEIGHT/4) begin
                                if(which==17)
                                    set_search(1,WIDTH-26,HEIGHT/2-12,WIDTH-2,HEIGHT/2+12);
                                if(which==18)
                                    search_enable=0;
                            end
                        end
                        // Complete circles touch inclusive ROI/raster edges.
                        19: if(disk(px,py,8,8,8)) pixel=16'hf800;
                        20: if(disk(px,py,WIDTH-9,HEIGHT-9,8)) pixel=16'hf800;
                        default: pixel=0;
                    endcase
                    tick(pixel,1,0);
                    if (update !== 0 || published !== frozen)
                        $fatal(1,"outputs changed during active frame scene=%0d x=%0d y=%0d",scene,px,py);
                    // An optional DE pause is not a pixel and must not reset a run.
                    if ((px==WIDTH/3) && (py%3==1)) tick(16'hf800,0,0);
                end
                // Scene 4 has continuous DE between rows and VS immediately
                // after the final pixel, exercising row-pipeline flush order.
                if(py<HEIGHT-1 && which!=4) repeat(3) tick(16'hf800,0,0);
            end
            start_frame(1);
        end
    endtask

    initial begin
        repeat(3) tick(0,0,0);
        rst_n=1;
        // Data before the first VS must be ignored.
        repeat(20) tick(16'hf800,1,0);
        start_frame(0);
        draw_frame(0);
        expect_target(WIDTH/2-8,HEIGHT/2-7,WIDTH/2+8,HEIGHT/2+7,WIDTH/2,HEIGHT/2,0);
        if (count < 180 || count > 210) $fatal(1,"disk area is implausible: %0d",count);
        draw_frame(1); expect_lost();
        draw_frame(2); expect_target(4,5,20,19,12,12,0);
        draw_frame(3);
        expect_target(WIDTH-24,HEIGHT-22,WIDTH-4,HEIGHT-4,WIDTH-14,HEIGHT-13,0);
        // Severe clipping is not enough evidence for first acquisition.
        // Tracker-level tests cover HOLD and reacquisition after such rejection.
        draw_frame(4); expect_lost();
        draw_frame(5); expect_lost();
        draw_frame(6); expect_lost();
        draw_frame(7); expect_lost();
        draw_frame(8); expect_lost();
        draw_frame(9); expect_lost();
        draw_frame(10); expect_lost();
        draw_frame(11);
        expect_target(WIDTH-23,HEIGHT/2-8,WIDTH-5,HEIGHT/2+8,WIDTH-14,HEIGHT/2,0);
        draw_frame(12); expect_lost();
        draw_frame(13); expect_lost();
        draw_frame(14); expect_lost();
        draw_frame(15);
        // A highlight may split a row into short red fragments. Fragments below
        // MIN_RUN are discarded, so center/bounds allow one pixel quantization.
        expect_target(WIDTH/2-8,HEIGHT/2-7,WIDTH/2+8,HEIGHT/2+7,WIDTH/2,HEIGHT/2,1);
        draw_frame(0);
        expect_target(WIDTH/2-8,HEIGHT/2-7,WIDTH/2+8,HEIGHT/2+7,WIDTH/2,HEIGHT/2,0);

        // A short/interrupted frame must neither update nor replace a prior result.
        repeat(WIDTH*4) tick(16'hf800,1,0);
        start_frame(0);
        expect_target(WIDTH/2-8,HEIGHT/2-7,WIDTH/2+8,HEIGHT/2+7,WIDTH/2,HEIGHT/2,0);
        repeat(WIDTH*3) tick(16'hf800,1,0);
        @(negedge clk); rst_n=0;
        #1;
        if (published !== 0 || update !== 0) $fatal(1,"asynchronous reset did not clear result");
        @(negedge clk); rst_n=1;
        repeat(WIDTH*(HEIGHT-3)) tick(16'hf800,1,0);
        start_frame(0);
        expect_lost();
        draw_frame(2); expect_target(4,5,20,19,12,12,0);
        draw_frame(1); expect_lost();

        // Two genuine disks replace the former accepted red rectangles. ROI
        // decisions, frame-frozen bounds and exact retained pixel counts remain tested.
        set_search(1,2,HEIGHT/2-10,22,HEIGHT/2+10);
        start_frame(0);
        draw_frame(16); expect_circle(12,HEIGHT/2,8);
        draw_frame(17); expect_circle(12,HEIGHT/2,8);
        draw_frame(16); expect_circle(WIDTH-14,HEIGHT/2,10);

        set_search(1,2,HEIGHT/2-10,22,HEIGHT/2+10);
        start_frame(0);
        draw_frame(18); expect_circle(12,HEIGHT/2,8);
        draw_frame(16); expect_circle(WIDTH-14,HEIGHT/2,10);

        // Complete circle bounds must include the raster and ROI edges.
        set_search(1,0,0,16,16);
        start_frame(0);
        draw_frame(19); expect_circle(8,8,8);
        set_search(1,WIDTH-17,HEIGHT-17,WIDTH-1,HEIGHT-1);
        start_frame(0);
        draw_frame(20); expect_circle(WIDTH-9,HEIGHT-9,8);
        set_search(1,WIDTH-17,HEIGHT-17,WIDTH+20,HEIGHT+20);
        start_frame(0);
        draw_frame(20); expect_circle(WIDTH-9,HEIGHT-9,8);

        // A square crop of a uniform red wall must never become a false ball.
        set_search(1,0,0,11,11);
        start_frame(0);
        draw_frame(9); expect_lost();
        set_search(1,WIDTH-12,HEIGHT-12,WIDTH-1,HEIGHT-1);
        start_frame(0);
        draw_frame(9); expect_lost();

        set_search(1,21,0,10,HEIGHT-1);
        start_frame(0);
        draw_frame(16); expect_lost();
        set_search(0,21,0,10,HEIGHT-1);
        start_frame(0);
        draw_frame(16); expect_circle(WIDTH-14,HEIGHT/2,10);
        $display("PASS red ball detector %0dx%0d: %0d checks; color/shape, clipping, DE gaps, loss/reacquire, reset, ROI distractors/boundaries and frame-frozen controls",WIDTH,HEIGHT,checks);
        $finish;
    end
    initial begin
        #1000000000;
        $fatal(1,"timeout");
    end
endmodule
