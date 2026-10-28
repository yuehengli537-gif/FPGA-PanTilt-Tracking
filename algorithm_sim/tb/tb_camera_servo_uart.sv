`timescale 1ns/1ps
module tb_camera_servo_uart;
    reg clk=0,video_clk=0,rst_n=0,video_running=1;
    always #18.518519 clk=~clk;
    always #6.734007 if(video_running)video_clk=~video_clk;
    reg source_alive=1,frame_update=0,target_valid=0;
    reg [1:0] track_state=0;
    reg signed [12:0] error_x=0,error_y=0;
    wire rx,tx,pan,tilt,enabled,auto_mode,timeout_hold,moving;
    wire [11:0] current_pan,current_tilt,target_pan,target_tilt;
    uart_host_model #(.BIT_NS(8680.555556)) host(.to_dut(rx),.from_dut(tx));
    camera_servo_uart dut(
        .clk(clk),.rst_n(rst_n),.video_clk(video_clk),.video_source_alive(source_alive),
        .video_frame_update(frame_update),.video_target_valid(target_valid),
        .video_track_state(track_state),.video_error_x(error_x),.video_error_y(error_y),
        .uart_rx(rx),.uart_tx(tx),.servo_pan_pwm(pan),.servo_tilt_pwm(tilt),
        .enabled(enabled),.auto_mode(auto_mode),.timeout_hold(timeout_hold),.moving(moving),
        .current_pan_us(current_pan),.current_tilt_us(current_tilt),
        .target_pan_us(target_pan),.target_tilt_us(target_tilt));
    integer base=0,sequence_number=0,saved_pan,saved_tilt,i;
    integer measurement_count=0,saved_count;
    reg [28:0] latest_measurement;
    always @(posedge clk)begin
        #1;
        if(dut.measurement_update)begin
            latest_measurement=dut.measurement;measurement_count=measurement_count+1;
        end
    end
    // Local reply checker accepts the new auto-mode bit while still checking
    // framing, protocol version, sequence, packet CRC and all reserved bits.
    task rpc(input [7:0] command,input [63:0] payload,input [7:0] expected_type);
        integer guard,n;reg [15:0] crc;
        begin
            base=host.received_count;sequence_number=(sequence_number+1)&255;
            host.send_packet(command,sequence_number,payload,0,1,0);
            guard=0;
            while(host.received_count<base+16 && guard<4000)begin
                #8680.555556;guard=guard+1;
            end
            if(host.received_count!=base+16)$fatal(1,"wrapper UART reply timeout/length");
            if(host.received[base]!=8'ha5 || host.received[base+1]!=8'h5a ||
               host.received[base+2]!=1 || host.received[base+3]!=expected_type ||
               host.received[base+4]!=sequence_number || host.received[base+5][7:4]!=0)
                $fatal(1,"wrapper UART header/type/sequence/flags");
            crc=16'hffff;
            for(n=2;n<14;n=n+1)crc=host.crc_byte(crc,host.received[base+n]);
            if({host.received[base+14],host.received[base+15]}!=crc)$fatal(1,"wrapper UART CRC");
            #26041.666668;
        end
    endtask
    task publish(input valid,input [1:0] state,input signed [12:0] x,y);
        begin
            // Match tracker registered outputs: update and payload all change
            // together with NBA on the video edge, rather than artificial
            // stable data half a cycle before the event.
            @(posedge video_clk);
            target_valid<=valid;track_state<=state;error_x<=x;error_y<=y;frame_update<=1;
            @(posedge video_clk);frame_update<=0;
            #1000;
        end
    endtask
    task moving_frames(input integer count);
        integer f;
        begin
            for(f=0;f<count;f=f+1)begin publish(1,2,13'sd180,13'sd140);#20000000;end
        end
    endtask
    task expect_stationary(input integer duration_ns);
        begin
            saved_pan=current_pan;saved_tilt=current_tilt;
            #(duration_ns);
            if(current_pan!=saved_pan || current_tilt!=saved_tilt ||
               target_pan!=saved_pan || target_tilt!=saved_tilt || moving)
                $fatal(1,"camera invalid/stale/HOLD did not freeze command position");
        end
    endtask
    initial begin
        #1000;rst_n=1;#100000;
        rpc(8'h01,0,8'h81);
        if(host.received[base+12]!=3 || enabled || auto_mode || pan || tilt)
            $fatal(1,"wrapper boot capability3/disabled startup");
        publish(1,2,-13'sd190,13'sd145);
        if(latest_measurement!=={1'b1,2'd2,-13'sd190,13'sd145})
            $fatal(1,"registered tracker event/data snapshot mismatch");
        if(enabled || pan || tilt)$fatal(1,"camera measurement enabled outputs");
        rpc(8'h14,{16'd2500,16'd1500,16'd2500,16'd500},8'h91);
        rpc(8'h11,1,8'h90);
        publish(1,2,13'sd180,13'sd140);
        rpc(8'h20,1,8'h90);
        if(!auto_mode || !host.received[base+5][3])$fatal(1,"auto mode command/flag");
        moving_frames(3);
        if(current_pan<=1500 || current_tilt<=1500)$fatal(1,"valid video did not drive both axes");
        $display("PASS camera wrapper actual27MHz UART115200 + registered tracker snapshot + dual auto PWM");

        publish(0,1,13'sd180,13'sd140);
        expect_stationary(40000000);
        if(!auto_mode)$fatal(1,"brief occlusion lost selected auto mode");
        publish(0,0,0,0);expect_stationary(20000000);
        saved_pan=current_pan;saved_tilt=current_tilt;
        moving_frames(2);
        if(current_pan<=saved_pan || current_tilt<=saved_tilt)$fatal(1,"reacquisition did not resume");
        $display("PASS HOLD/LOST freeze, fresh LOCK resumes");

        // Source guard must invalidate without waiting for another frame.
        saved_count=measurement_count;
        @(negedge video_clk);source_alive=0;
        #1000;
        if(measurement_count<=saved_count || latest_measurement!==29'd0)
            $fatal(1,"source_alive falling did not publish invalidation");
        expect_stationary(40000000);
        source_alive=1;expect_stationary(20000000);
        moving_frames(1);
        $display("PASS camera source guard invalidates without frame event");

        // A frozen camera clock never renews freshness. After the 100 ms
        // timeout both targets hold; merely restarting clock is insufficient.
        publish(1,2,13'sd180,13'sd140);
        @(negedge video_clk);video_running=0;
        #110000000;expect_stationary(40000000);
        video_running=1;expect_stationary(20000000);
        saved_pan=current_pan;
        moving_frames(1);
        if(current_pan<=saved_pan)$fatal(1,"fresh frame after camera restart did not resume");
        $display("PASS stopped camera timeout and fresh-only restart");

        rpc(8'h12,0,8'h90);
        if(auto_mode)$fatal(1,"HOLD did not exit auto mode");
        saved_pan=current_pan;saved_tilt=current_tilt;
        moving_frames(2);
        if(current_pan!=saved_pan || current_tilt!=saved_tilt)$fatal(1,"camera overrode manual HOLD");
        rpc(8'h11,0,8'h90);
        if(enabled || auto_mode || pan || tilt)$fatal(1,"disable failed");
        publish(1,2,13'sd200,13'sd200);
        #22000000;
        if(enabled || pan || tilt)$fatal(1,"camera enabled disabled outputs");
        $display("PASS UART HOLD/disable precedence over camera updates");
        #3;rst_n=0;#2;
        if(enabled || auto_mode || pan || tilt)$fatal(1,"common reset did not disable outputs");
        $display("PASS tb_camera_servo_uart: serial/CDC/auto/occlusion/stall/stop");
        $finish;
    end
    initial begin #1000000000;$fatal(1,"wrapper integration test timeout");end
endmodule
