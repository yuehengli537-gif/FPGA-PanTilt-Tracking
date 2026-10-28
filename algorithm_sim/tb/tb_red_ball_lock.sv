`timescale 1ns/1ps
module tb_red_ball_lock;
    parameter integer SINGLE=0;
    localparam integer CF=SINGLE?1:3, HF=SINGLE?1:8, TIMEOUT=32;
    reg clk=0,rst_n=0,measurement_update=0,measurement_valid=0;
    reg [11:0] measurement_left=0,measurement_top=0,measurement_right=0,measurement_bottom=0;
    reg [11:0] measurement_x=0,measurement_y=0;
    wire frame_update,target_valid,search_enable;
    wire [1:0] track_state;
    wire [11:0] bbox_left,bbox_top,bbox_right,bbox_bottom,center_x,center_y;
    wire [11:0] search_left,search_top,search_right,search_bottom;
    integer checks=0,i;
    always #5 clk=~clk;
    red_ball_lock #(.WIDTH(200),.HEIGHT(120),.CONFIRM_FRAMES(CF),.HOLD_FRAMES(HF),
        .MATCH_RADIUS(16),.FRAME_TIMEOUT_CYCLES(TIMEOUT)) dut(.*);
    task check(input bit condition,input string name);
        begin
            if(!condition) $fatal(1,"lock check%0d %s: state=%0d valid=%0d center=%0d,%0d",checks,name,track_state,target_valid,center_x,center_y);
            checks=checks+1;
        end
    endtask
    task expect_state(input integer state);
        begin
            check(track_state===state[1:0],"state");
            check(target_valid===(state==2),"only TRACK is valid");
            check(search_enable===((state==2)||(state==3)),"ROI gating");
            check(search_left<=search_right && search_right<200 && search_top<=search_bottom && search_bottom<120,"bounded ROI");
            if(state==0) check({bbox_left,bbox_top,bbox_right,bbox_bottom,center_x,center_y}===0,"clear expired memory");
        end
    endtask
    task measure(input bit valid,input integer x,y);
        begin
            @(negedge clk);
            measurement_update=1;measurement_valid=valid;
            measurement_x=x;measurement_y=y;
            measurement_left=x-10;measurement_right=x+10;
            measurement_top=y-10;measurement_bottom=y+10;
            @(posedge clk);#1;check(!frame_update,"no early publication");
            @(negedge clk);measurement_update=0;
            @(posedge clk);#1;check(frame_update,"publish exactly one cycle after sampling");
        end
    endtask
    task idle;
        begin @(negedge clk);measurement_update=0;@(posedge clk);#1;end
    endtask
    task acquire(input integer x,y);
        begin
            for(i=1;i<=CF;i=i+1) begin
                measure(1,x,y);expect_state(i==CF?2:1);
            end
            check(center_x==x && center_y==y,"acquisition has unsmoothed measured center");
        end
    endtask
    initial begin
        repeat(3) @(negedge clk);
        rst_n=1;
        expect_state(0);
        acquire(40,30);
        measure(1,44,40);expect_state(2);
        check(center_x==42 && center_y==35,"half smoothing");
        measure(0,0,0);expect_state(SINGLE?0:3);
        if(!SINGLE) begin
            check(center_x==42 && center_y==35,"remember last trusted center");
            measure(1,180,100);expect_state(3);
            check(center_x==42 && center_y==35,"far distractor cannot steal hold");
            measure(1,44,36);expect_state(3);
            measure(0,0,0);expect_state(3); // interrupted confirmation resets count
            measure(1,44,36);expect_state(3);
            measure(1,44,36);expect_state(3);
            measure(1,44,36);expect_state(2);
            check(center_x==44 && center_y==36,"reacquire adopts current measurement");
        end else acquire(44,36);
        for(i=1;i<=HF;i=i+1) begin
            measure(0,0,0);expect_state(i==HF?0:3);
        end
        if(!SINGLE) begin
            measure(1,40,30);expect_state(1);
            measure(1,180,100);expect_state(1); // candidate jump starts again
            measure(1,180,100);expect_state(1);
            measure(0,0,0);expect_state(0); // missing candidate cancels acquisition
        end
        acquire(180,100);
        check(search_right==199 && search_bottom==119,"ROI clips right/bottom edges");
        // Live measurement_valid may stay high forever after a stream failure.
        // A watchdog must expire without relying on it changing.
        for(i=1;i<TIMEOUT;i=i+1) begin idle();check(target_valid,"not before watchdog deadline");end
        idle();expect_state(0);check(frame_update,"timeout publishes invalidation");
        repeat(5) begin idle();check(!frame_update,"watchdog is one-shot");end
        acquire(10,10);
        check(search_left==0 && search_top==0,"ROI clips left/top edges");
        // New measurement on the exact deadline takes priority over timeout.
        for(i=1;i<TIMEOUT;i=i+1) idle();
        measure(1,10,10);expect_state(2);
        @(negedge clk);rst_n=0;#1;
        check(track_state==0 && !target_valid && !frame_update && center_x==0,"asynchronous reset");
        $display("PASS red_ball_lock SINGLE=%0d: %0d checks; confirmation, smoothing, short/long loss, interrupted recovery, distractor, bounded ROI, watchdog boundary and reset",SINGLE,checks);
        $finish;
    end
endmodule
