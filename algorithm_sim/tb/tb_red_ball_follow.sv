`timescale 1ns/1ps
// Ideal image-space camera model ONLY: this checks command direction, stopping
// during occlusion, confirmation, and recovery. There is no servo PWM, motor,
// inertia, wiring, lens calibration, or physical pan/tilt model in this test.
module tb_red_ball_follow;
    localparam integer WIDTH=160, HEIGHT=120, H_TOTAL=200, DEAD_ZONE=6;
    localparam [1:0] LOST=0, ACQUIRE=1, LOCKED=2, HOLD=3;
    reg clk=0, rst_n=0, de=0, hs=0, vs=0;
    reg [15:0] rgb=0;
    always #5 clk=~clk;
    wire [7:0] r,g,b;
    wire ode,ohs,ovs,valid,update,centered;
    wire [11:0] cx,cy;
    wire signed [12:0] ex,ey;
    wire [1:0] state;
    wire signed [1:0] fx,fy;
    red_ball_tracking #(
        .WIDTH(WIDTH),.HEIGHT(HEIGHT),.H_TOTAL(H_TOTAL),.DISPLAY_MODE(0),
        .DEAD_ZONE(DEAD_ZONE),.MAX_DIAMETER(40),.MIN_PIXELS(100),
        .HOLD_FRAMES(6),.CONFIRM_FRAMES(3),.MATCH_RADIUS(48)
    ) dut (
        .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_hs(hs),.in_vs(vs),
        .out_r(r),.out_g(g),.out_b(b),.out_de(ode),.out_hs(ohs),.out_vs(ovs),
        .target_valid(valid),.frame_update(update),.center_x(cx),.center_y(cy),
        .error_x(ex),.error_y(ey),.centered(centered),.track_state(state),
        .follow_x(fx),.follow_y(fy)
    );

    integer trace,trial,f,x,y,px,py,dx,dy;
    integer world_x,world_y,camera_x,camera_y,old_camera_x,old_camera_y;
    integer drawn_x,drawn_y,completed_x,completed_y,completed_frame;
    integer hide_remaining,initial_measurements,occluded_updates,recovery_measurements;
    integer stable_frames,updates,positive_x,negative_x,positive_y,negative_y;
    integer total_updates=0,total_checks=0,expected_fx,expected_fy;
    reg active,drawn_visible,completed_visible,prepared,first_lock,recovered;
    reg [15:0] pixel;

    function integer magnitude;
        input integer value;
        begin magnitude=value<0 ? -value : value; end
    endfunction

    task consume_update;
        begin
            if(completed_frame<0) $fatal(1,"published before first complete frame");
            if(valid !== (state==LOCKED)) $fatal(1,"valid/state disagreement");
            expected_fx=valid ? (ex>DEAD_ZONE ? 1 : (ex < -DEAD_ZONE ? -1 : 0)) : 0;
            expected_fy=valid ? (ey>DEAD_ZONE ? 1 : (ey < -DEAD_ZONE ? -1 : 0)) : 0;
            if(fx!==expected_fx[1:0] || fy!==expected_fy[1:0])
                $fatal(1,"follow sign or dead zone incorrect trial=%0d frame=%0d",trial,completed_frame);
            if(!valid && (ex!=0 || ey!=0 || centered || fx!=0 || fy!=0))
                $fatal(1,"unconfirmed/missing target requested movement");
            if(valid && (ex!=$signed({1'b0,cx})-WIDTH/2 || ey!=$signed({1'b0,cy})-HEIGHT/2))
                $fatal(1,"image error does not match reported center");

            if(!first_lock) begin
                initial_measurements=initial_measurements+1;
                if(initial_measurements<3 && (state!==ACQUIRE || valid))
                    $fatal(1,"target locked before three visible frames");
                if(initial_measurements==3 && (state!==LOCKED || !valid))
                    $fatal(1,"target did not lock on third visible frame");
                if(initial_measurements>3) $fatal(1,"initial lock failed");
            end
            if(!completed_visible) begin
                occluded_updates=occluded_updates+1;
                if(state!==HOLD || valid || fx!=0 || fy!=0)
                    $fatal(1,"camera must stop as soon as hidden frame is published");
            end else if(occluded_updates==2 && !recovered) begin
                recovery_measurements=recovery_measurements+1;
                if(recovery_measurements<3 && (state!==HOLD || valid))
                    $fatal(1,"resumed before three matching visible frames");
                if(recovery_measurements==3) begin
                    if(state!==LOCKED || !valid) $fatal(1,"failed three-frame recovery");
                    recovered=1;
                end
            end

            old_camera_x=camera_x; old_camera_y=camera_y;
            if(valid) begin
                // Positive request moves the virtual camera right/down, which
                // moves a stationary world's image left/up on the next frame.
                camera_x=camera_x+4*$signed(fx);
                camera_y=camera_y+4*$signed(fy);
                if(fx==1) positive_x=positive_x+1;
                if(fx==-1) negative_x=negative_x+1;
                if(fy==1) positive_y=positive_y+1;
                if(fy==-1) negative_y=negative_y+1;
                if(!first_lock) begin
                    first_lock=1;
                    hide_remaining=2;
                end
            end
            if(state!=LOCKED && (camera_x!=old_camera_x || camera_y!=old_camera_y))
                $fatal(1,"ideal camera moved while acquiring, holding, or lost");
            if(valid && recovered && centered && fx==0 && fy==0 &&
               magnitude(world_x-camera_x-WIDTH/2)<=DEAD_ZONE &&
               magnitude(world_y-camera_y-HEIGHT/2)<=DEAD_ZONE)
                stable_frames=stable_frames+1;
            else stable_frames=0;

            $fdisplay(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                trial,completed_frame,completed_visible,completed_x,completed_y,state,valid,
                cx,cy,ex,ey,$signed(fx),$signed(fy),old_camera_x,old_camera_y,
                camera_x,camera_y,world_x-camera_x-WIDTH/2,world_y-camera_y-HEIGHT/2);
            updates=updates+1; total_updates=total_updates+1; total_checks=total_checks+1;
        end
    endtask

    task tick;
        input [15:0] p;
        input pd,ph,pv;
        begin
            @(negedge clk); rgb=p; de=pd; hs=ph; vs=pv;
            @(posedge clk); #1;
            if(rst_n && update) consume_update();
        end
    endtask

    task run_trial;
        input integer wx,wy;
        begin
            @(negedge clk); rst_n=0;
            repeat(6) tick(0,0,0,0);
            world_x=wx; world_y=wy; camera_x=0; camera_y=0;
            completed_frame=-1; completed_visible=0; completed_x=0; completed_y=0;
            first_lock=0; recovered=0; hide_remaining=0;
            initial_measurements=0; occluded_updates=0; recovery_measurements=0;
            stable_frames=0; updates=0;
            positive_x=0; negative_x=0; positive_y=0; negative_y=0;
            rst_n=1;
            for(f=0;f<40 && stable_frames<5;f=f+1) begin
                prepared=0;
                for(y=0;y<HEIGHT+8;y=y+1) begin
                    for(x=0;x<H_TOTAL;x=x+1) begin
                        active=(y>=4 && y<HEIGHT+4 && x>=4 && x<WIDTH+4);
                        px=x-4; py=y-4; pixel=0;
                        if(active) begin
                            // The frame-start result has already updated the
                            // camera. Hold this new view constant for all pixels.
                            if(!prepared) begin
                                drawn_x=world_x-camera_x; drawn_y=world_y-camera_y;
                                drawn_visible=(hide_remaining==0);
                                if(hide_remaining>0) hide_remaining=hide_remaining-1;
                                prepared=1;
                                if(drawn_x<12 || drawn_x>=WIDTH-12 || drawn_y<12 || drawn_y>=HEIGHT-12)
                                    $fatal(1,"model drove target outside view");
                            end
                            pixel=16'h8410;
                            dx=px-drawn_x; dy=py-drawn_y;
                            if(drawn_visible && dx*dx+dy*dy<=144) pixel=16'hf800;
                        end
                        tick(pixel,active,x<2,y==0);
                    end
                end
                completed_frame=f; completed_visible=drawn_visible;
                completed_x=drawn_x; completed_y=drawn_y;
            end
            if(stable_frames<5 || !recovered || occluded_updates!=2 || recovery_measurements!=3)
                $fatal(1,"trial %0d did not settle/recover by40frames: stable=%0d hidden=%0d recovery=%0d offset=%0d,%0d",trial,stable_frames,occluded_updates,recovery_measurements,camera_x,camera_y);
            if(trial==0 && (positive_x==0 || negative_y==0)) $fatal(1,"right/up direction untested");
            if(trial==1 && (negative_x==0 || positive_y==0)) $fatal(1,"left/down direction untested");
            $display("PASS ideal follow trial=%0d frames=%0d updates=%0d final_actual_error=%0d,%0d hidden=2 recovery=3 stable=%0d",
                trial,f,updates,world_x-camera_x-WIDTH/2,world_y-camera_y-HEIGHT/2,stable_frames);
        end
    endtask

    initial begin
        trace=$fopen("follow_model_trace.csv","w");
        if(!trace) $fatal(1,"cannot open ideal follow trace");
        $fdisplay(trace,"trial,frame,visible,image_x,image_y,state,valid,center_x,center_y,error_x,error_y,follow_x,follow_y,camera_x_before,camera_y_before,camera_x_after,camera_y_after,actual_error_x,actual_error_y");
        trial=0; run_trial(120,30);
        trial=1; run_trial(40,90);
        $fclose(trace);
        $display("PASS ideal image-space follow model: two directions, two-frame occlusion, three-frame recovery, dead-zone convergence; updates=%0d checks=%0d; NO servo/PWM/physical validation",total_updates,total_checks);
        $finish;
    end
    initial begin
        #1000000000;
        $fatal(1,"ideal follow model timeout");
    end
endmodule
