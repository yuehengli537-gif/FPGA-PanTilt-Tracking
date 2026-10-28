`timescale 1ns/1ps
// Real pixels -> detector -> lock -> source guard -> CDC -> UART controller ->
// independently measured PWM. Only the raster and remote UART host are models.
module tb_camera_servo_pixels;
    localparam integer WIDTH=160, HEIGHT=160, H_TOTAL=176, V_TOTAL=168;
    localparam integer CONTROL_HZ=4000000, VIDEO_HZ=3000000;
    localparam integer CYCLES_PER_US=CONTROL_HZ/1000000;
    localparam integer PWM_CYCLES=CYCLES_PER_US*20000;
    localparam real BIT_NS=8680.555556;
    reg clk=0,video_clk=0,rst_n=0;
    always #125 clk=~clk;
    always #166.666667 video_clk=~video_clk;
    reg [15:0] rgb=0;
    reg de=0,hs=0,vs=0;
    reg source_running=1,visible=1;
    integer requested_x=140,requested_y=80;
    wire camera_vs;
    // The source VS is deliberately off the receiving clock edge. The display
    // raster keeps running when source_running=0, as a repeated DDR frame can.
    assign #83 camera_vs=source_running && vs;
    wire alive,valid,frame_update;
    wire [1:0] state;
    wire signed [12:0] error_x,error_y;
    wire rx,tx,pan,tilt,enabled,auto_mode,timeout_hold,moving;
    wire [11:0] current_pan,current_tilt,target_pan,target_tilt;
    camera_frame_guard #(.TIMEOUT_CYCLES(VIDEO_HZ*30/1000)) guard(
        .clk(video_clk),.rst_n(rst_n),.camera_vs_async(camera_vs),.source_alive(alive));
    red_ball_tracking #(
        .WIDTH(WIDTH),.HEIGHT(HEIGHT),.H_TOTAL(H_TOTAL),.DISPLAY_MODE(0),
        .MAX_DIAMETER(40),.HOLD_FRAMES(5),.CONFIRM_FRAMES(3),.MATCH_RADIUS(128),
        .FRAME_TIMEOUT_CYCLES(VIDEO_HZ/10)
    ) tracker(
        .clk(video_clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_hs(hs),.in_vs(vs),
        .out_r(),.out_g(),.out_b(),.out_de(),.out_hs(),.out_vs(),
        .target_valid(valid),.frame_update(frame_update),.center_x(),.center_y(),
        .error_x(error_x),.error_y(error_y),.centered(),.track_state(state),
        .follow_x(),.follow_y(),.servo_pan_us({4'd0,current_pan}),
        .servo_tilt_us({4'd0,current_tilt}),.servo_pan_limit(2'd0),.servo_tilt_limit(2'd0),
        .servo_enabled(enabled),.servo_source_alive(alive),.servo_stale(1'b0));
    camera_servo_uart #(.CLK_HZ(CONTROL_HZ),.BAUD(115200)) dut(
        .clk(clk),.rst_n(rst_n),.video_clk(video_clk),.video_source_alive(alive),
        .video_frame_update(frame_update),.video_target_valid(valid),.video_track_state(state),
        .video_error_x(error_x),.video_error_y(error_y),.uart_rx(rx),.uart_tx(tx),
        .servo_pan_pwm(pan),.servo_tilt_pwm(tilt),.enabled(enabled),.auto_mode(auto_mode),
        .timeout_hold(timeout_hold),.moving(moving),.current_pan_us(current_pan),
        .current_tilt_us(current_tilt),.target_pan_us(target_pan),.target_tilt_us(target_tilt));
    uart_host_model #(.BIT_NS(BIT_NS)) host(.to_dut(rx),.from_dut(tx));

    integer frame_number=0,px,py,draw_x,draw_y,dx,dy;
    reg draw_visible,active;
    initial begin
        wait(rst_n);
        forever begin
            // Snapshot requested image changes at a raster boundary.
            draw_x=requested_x;draw_y=requested_y;draw_visible=visible;
            for(py=0;py<V_TOTAL;py=py+1)begin
                for(px=0;px<H_TOTAL;px=px+1)begin
                    @(negedge video_clk);
                    active=py>=4 && py<HEIGHT+4 && px>=4 && px<WIDTH+4;
                    dx=px-4-draw_x;dy=py-4-draw_y;
                    rgb=active ? ((draw_visible && dx*dx+dy*dy<=81) ? 16'hf800 : 16'h8410) : 16'd0;
                    de=active;hs=px<2;vs=py==0;
                end
            end
            frame_number=frame_number+1;
        end
    end
    integer tracker_updates=0,cdc_updates=0,acquire_updates=0,hold_updates=0,lost_updates=0;
    always @(posedge video_clk)begin
        #1;
        if(rst_n && frame_update)begin
            tracker_updates=tracker_updates+1;
            if(valid !== (state==2))$fatal(1,"tracker validity/state disagree");
            if(state==1)acquire_updates=acquire_updates+1;
            if(state==3)hold_updates=hold_updates+1;
            if(state==0)lost_updates=lost_updates+1;
        end
    end
    always @(posedge clk)begin
        #1;
        if(rst_n && dut.measurement_update)cdc_updates=cdc_updates+1;
    end

    // Verify actual output edges, period, pulse width, and no mid-pulse command
    // changes. Direction checks below require both register and measured pulse.
    longint tick=0,pan_rise=-1,tilt_rise=-1;
    integer expected_pan=1500,expected_tilt=1500,measured_pan=1500,measured_tilt=1500;
    integer pwm_pulses=0;
    reg previous_pan=0,previous_tilt=0;
    always @(posedge clk)begin
        #1;tick=tick+1;
        if(!rst_n || !enabled)begin
            if(pan!==0 || tilt!==0)$fatal(1,"PWM active while disabled");
            previous_pan=0;previous_tilt=0;pan_rise=-1;tilt_rise=-1;
        end else begin
            if(pan && !previous_pan)begin
                if(pan_rise>=0 && tick-pan_rise!=PWM_CYCLES)$fatal(1,"pan PWM period");
                if(!tilt)$fatal(1,"pan/tilt start not aligned");
                pan_rise=tick;expected_pan=current_pan;expected_tilt=current_tilt;
            end else if(pan_rise>=0 && (current_pan!=expected_pan || current_tilt!=expected_tilt))
                $fatal(1,"command changed inside PWM frame");
            if(tilt && !previous_tilt)begin
                if(tilt_rise>=0 && tick-tilt_rise!=PWM_CYCLES)$fatal(1,"tilt PWM period");
                tilt_rise=tick;
            end
            if(!pan && previous_pan)begin
                if(tick-pan_rise!=expected_pan*CYCLES_PER_US)$fatal(1,"pan PWM width");
                measured_pan=(tick-pan_rise)/CYCLES_PER_US;pwm_pulses=pwm_pulses+1;
            end
            if(!tilt && previous_tilt)begin
                if(tick-tilt_rise!=expected_tilt*CYCLES_PER_US)$fatal(1,"tilt PWM width");
                measured_tilt=(tick-tilt_rise)/CYCLES_PER_US;pwm_pulses=pwm_pulses+1;
            end
            previous_pan=pan;previous_tilt=tilt;
        end
    end

    integer sequence_number=0,base=0,rpc_count=0;
    task rpc(input [7:0] command,input [63:0] payload,input [7:0] expected_type);
        integer attempts,n;reg [15:0] crc;
        begin
            base=host.received_count;sequence_number=(sequence_number+1)&255;
            host.send_packet(command,sequence_number,payload,0,1,0);
            attempts=0;
            while(host.received_count<base+16 && attempts<4000)begin #(BIT_NS);attempts=attempts+1;end
            if(host.received_count!=base+16)$fatal(1,"pixel-chain UART timeout/length");
            if(host.received[base]!=8'ha5 || host.received[base+1]!=8'h5a ||
               host.received[base+2]!=1 || host.received[base+3]!=expected_type ||
               host.received[base+4]!=sequence_number || host.received[base+5][7:4]!=0)
                $fatal(1,"pixel-chain UART header/type/sequence/flags");
            crc=16'hffff;
            for(n=2;n<14;n=n+1)crc=host.crc_byte(crc,host.received[base+n]);
            if({host.received[base+14],host.received[base+15]}!=crc)$fatal(1,"pixel-chain UART CRC");
            #(BIT_NS*3);rpc_count=rpc_count+1;
        end
    endtask
    task frames(input integer count);
        integer end_frame;
        begin end_frame=frame_number+count;wait(frame_number>=end_frame);#5000;end
    endtask
    function integer absval(input integer value);absval=value<0 ? -value : value;endfunction
    task await_target(input integer x,y);
        integer waited;
        begin
            waited=0;
            while((!valid || state!=2 || absval(error_x-x)>1 || absval(error_y-y)>1) && waited<30)begin
                frames(1);waited=waited+1;
            end
            if(!valid || state!=2 || absval(error_x-x)>1 || absval(error_y-y)>1)
                $fatal(1,"pixel target did not lock: wanted %0d,%0d got state%0d error%0d,%0d",x,y,state,error_x,error_y);
            #5000;
        end
    endtask
    task check_auto_reply;
        begin
            rpc(8'h02,0,8'h90);
            if(host.received[base+5][3:2]!=2'b10 || !auto_mode || timeout_hold)
                $fatal(1,"auto mode lost during camera test");
        end
    endtask
    integer saved_pan,saved_tilt;
    task check_frozen;
        begin
            if(current_pan!=saved_pan || current_tilt!=saved_tilt ||
               target_pan!=saved_pan || target_tilt!=saved_tilt ||
               measured_pan!=saved_pan || measured_tilt!=saved_tilt || moving)
                $fatal(1,"invalid camera moved PWM: saved%0d,%0d now%0d,%0d",saved_pan,saved_tilt,current_pan,current_tilt);
        end
    endtask
    task direction_trial(input integer x,y);
        begin
            rpc(8'h12,0,8'h90);
            requested_x=WIDTH/2+x;requested_y=HEIGHT/2+y;
            await_target(x,y);
            saved_pan=current_pan;saved_tilt=current_tilt;
            rpc(8'h20,1,8'h90);
            if(!host.received[base+5][3])$fatal(1,"SET_MODE reply missing auto flag");
            frames(6);check_auto_reply();
            if((x>0 && (current_pan<=saved_pan || measured_pan<=saved_pan)) ||
               (x<0 && (current_pan>=saved_pan || measured_pan>=saved_pan)) ||
               (x==0 && (current_pan!=saved_pan || measured_pan!=saved_pan)) ||
               (y>0 && (current_tilt<=saved_tilt || measured_tilt<=saved_tilt)) ||
               (y<0 && (current_tilt>=saved_tilt || measured_tilt>=saved_tilt)) ||
               (y==0 && (current_tilt!=saved_tilt || measured_tilt!=saved_tilt)))
                $fatal(1,"pixel-to-PWM direction mismatch error%0d,%0d before%0d,%0d after%0d,%0d",x,y,saved_pan,saved_tilt,current_pan,current_tilt);
            $display("PASS real pixels direction error=%0d,%0d command=%0d,%0d measured=%0d,%0d",x,y,current_pan,current_tilt,measured_pan,measured_tilt);
        end
    endtask
    integer before_acquire,before_hold,before_lost,before_updates,before_cdc;
    initial begin
        #3000;rst_n=1;#100000;
        rpc(8'h01,0,8'h81);
        if(host.received[base+12]!=3 || enabled || pan || tilt)$fatal(1,"HELLO capability/boot state");
        rpc(8'h14,{16'd1700,16'd1300,16'd1700,16'd1300},8'h91);
        rpc(8'h11,1,8'h90);
        if(!host.received[base+5][0])$fatal(1,"ENABLE was not acknowledged");
        direction_trial(60,0);
        direction_trial(-60,0);
        direction_trial(0,60);
        direction_trial(0,-60);

        before_hold=hold_updates;before_lost=lost_updates;
        visible=0;wait(state==3 && !valid);#5000;
        saved_pan=current_pan;saved_tilt=current_tilt;
        frames(8);check_frozen();check_auto_reply();
        if(state!=0 || hold_updates<=before_hold || lost_updates<=before_lost)
            $fatal(1,"pixel occlusion did not exercise both HOLD and LOST");
        $display("PASS real blank pixels produce HOLD/LOST and freeze both PWM outputs");
        before_acquire=acquire_updates;
        visible=1;await_target(0,-60);
        if(acquire_updates-before_acquire<2)$fatal(1,"relock bypassed three-frame confirmation");
        frames(5);check_auto_reply();
        if(current_tilt>=saved_tilt || measured_tilt>=saved_tilt)$fatal(1,"pixel reacquisition did not resume PWM");
        $display("PASS three-frame real-pixel reacquisition resumes auto without another mode command");

        // Continue a valid displayed target while only real source VS stops.
        // This catches a missing source guard even though detector/lock stay live.
        before_updates=tracker_updates;before_cdc=cdc_updates;
        source_running=0;wait(!alive);#10000;
        saved_pan=current_pan;saved_tilt=current_tilt;
        frames(7);check_frozen();check_auto_reply();
        if(!valid || state!=2 || tracker_updates<=before_updates || cdc_updates<=before_cdc)
            $fatal(1,"source-stall test did not retain active repeated display frames");
        $display("PASS stopped camera VS freezes PWM despite continuing valid display/CDC updates");

        // Clear the historical target while the source is still absent, then
        // restore source frames and require the detector's real relock sequence.
        visible=0;frames(8);check_frozen();check_auto_reply();
        if(state!=0 || alive)$fatal(1,"source-off loss precondition failed");
        before_acquire=acquire_updates;
        source_running=1;visible=1;await_target(0,-60);
        if(!alive || acquire_updates-before_acquire<2)$fatal(1,"source recovery bypassed relock");
        frames(5);check_auto_reply();
        if(current_tilt>=saved_tilt || measured_tilt>=saved_tilt)$fatal(1,"source recovery did not resume PWM");
        $display("PASS camera VS recovery plus fresh pixel relock resumes automatic PWM");

        rpc(8'h12,0,8'h90);saved_pan=current_pan;saved_tilt=current_tilt;
        frames(4);check_frozen();
        if(auto_mode || host.received[base+5][3])$fatal(1,"UART HOLD did not exit auto");
        rpc(8'h11,0,8'h90);
        if(enabled || pan || tilt || auto_mode)$fatal(1,"final disable failed");
        $display("PASS tb_camera_servo_pixels: pixels/tracker/guard/CDC/UART/PWM frames=%0d tracker_updates=%0d cdc_updates=%0d RPCs=%0d measured_pulses=%0d",frame_number,tracker_updates,cdc_updates,rpc_count,pwm_pulses);
        $finish;
    end
    initial begin #3000000000;$fatal(1,"pixel-to-UART-PWM integration timeout");end
endmodule
