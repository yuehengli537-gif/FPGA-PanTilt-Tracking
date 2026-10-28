`timescale 1ns/1ps
module tb_red_ball_pipeline;
    parameter integer WIDTH=320, HEIGHT=180, H_TOTAL=400, FRAME_COUNT=26;
    localparam integer DELAY=2*H_TOTAL+15, QUEUE=DELAY+1;
    reg clk=0, rst_n=0, de=0, hs=0, vs=0;
    reg [15:0] rgb=0;
    always #5 clk=~clk;
    wire [7:0] r,g,b;
    wire ode,ohs,ovs,valid,update,centered;
    wire [11:0] cx,cy;
    wire signed [12:0] ex,ey;
    wire [1:0] state;
    wire signed [1:0] fx,fy;
    red_ball_tracking #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.H_TOTAL(H_TOTAL),
        .DEAD_ZONE(6),.MAX_DIAMETER(64),.HOLD_FRAMES(6),
        .CONFIRM_FRAMES(3),.MATCH_RADIUS(32)) dut (
        .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_hs(hs),.in_vs(vs),
        .out_r(r),.out_g(g),.out_b(b),.out_de(ode),.out_hs(ohs),.out_vs(ovs),
        .target_valid(valid),.frame_update(update),.center_x(cx),.center_y(cy),
        .error_x(ex),.error_y(ey),.centered(centered),.track_state(state),
        .follow_x(fx),.follow_y(fy)
    );
    reg [2:0] controls[0:QUEUE-1];
    reg [15:0] pixels[0:QUEUE-1];
    reg check_pixel[0:QUEUE-1];
    reg [23:0] expected_rgb;
    integer cycle=0,checks=0,rgb_checks=0,updates=0,write_slot,read_slot;
    integer previous_frame=-1, expected_x,expected_y,dx,dy;
    integer expected_state, expected_fx, expected_fy, trace;
    integer f,x,y,px,py,j;
    reg active, sample, expected_valid;
    reg [15:0] p;
    reg previous_valid=0;
    reg [1:0] previous_state=0;
    reg [11:0] previous_cx=0,previous_cy=0;
    function integer ball_x;
        input integer frame;
        begin
            if(frame>=16) ball_x=(WIDTH*3)/4;
            else if(frame>=6) ball_x=WIDTH/4+12;
            else if(frame==3) ball_x=WIDTH/4+8;
            else ball_x=WIDTH/4;
        end
    endfunction
    function integer ball_y;
        input integer frame;
        begin
            ball_y=HEIGHT/2;
        end
    endfunction
    function integer present;
        input integer frame;
        begin
            present=(frame<=3 || (frame>=6 && frame<=9) || frame>=16);
        end
    endfunction
    function integer state_for;
        input integer frame;
        begin
            case(frame)
                0,1,17,18:state_for=1;
                2,3,8,9,19,23,24,25:state_for=2;
                4,5,6,7,10,11,12,13,14,20,21,22:state_for=3;
                default:state_for=0;
            endcase
        end
    endfunction
    task tick;
        input [15:0] pixel;
        input pd,ph,pv,verify_rgb;
        begin
            write_slot=cycle%QUEUE;
            controls[write_slot]={pd,ph,pv};
            pixels[write_slot]=pixel;
            check_pixel[write_slot]=verify_rgb;
            @(negedge clk); rgb=pixel; de=pd; hs=ph; vs=pv;
            @(posedge clk); #1;
            if(cycle>=DELAY) begin
                read_slot=(cycle-DELAY)%QUEUE;
                if({ode,ohs,ovs}!==controls[read_slot])
                    $fatal(1,"timing mismatch at cycle%0d expected%0b got%0b",cycle,controls[read_slot],{ode,ohs,ovs});
                checks=checks+3;
                if(!ode && {r,g,b}!==24'd0) $fatal(1,"nonblack blanking");
                if(check_pixel[read_slot]) begin
                    expected_rgb={pixels[read_slot][15:11],3'b0,pixels[read_slot][10:5],2'b0,pixels[read_slot][4:0],3'b0};
                    if({r,g,b}!==expected_rgb) $fatal(1,"original image changed outside overlay cycle%0d",cycle);
                    rgb_checks=rgb_checks+1;
                end
            end
            if(update) begin
                expected_state=state_for(previous_frame);
                expected_valid=(expected_state==2);
                if(state!==expected_state[1:0]) $fatal(1,"state frame%0d got%0d expected%0d",previous_frame,state,expected_state);
                if(valid!==expected_valid) $fatal(1,"valid mismatch frame%0d",previous_frame);
                if(expected_valid) begin
                    expected_x=ball_x(previous_frame); expected_y=ball_y(previous_frame);
                    if(previous_frame==3) expected_x=WIDTH/4+4;
                    // Half-ball frame20 is deliberately uncertain: HOLD keeps
                    // the full-ball center, then frames21..23 reconfirm it.
                    if(cx!=expected_x || cy!=expected_y) $fatal(1,"center frame%0d got%0d,%0d expected%0d,%0d",previous_frame,cx,cy,expected_x,expected_y);
                    if(ex!=expected_x-WIDTH/2 || ey!=expected_y-HEIGHT/2) $fatal(1,"signed error mismatch");
                    if(centered!==((ex>=-6)&&(ex<=6)&&(ey>=-6)&&(ey<=6))) $fatal(1,"centered mismatch");
                end else begin
                    if(ex!=0 || ey!=0 || centered || fx!=0 || fy!=0)
                        $fatal(1,"occluded/unconfirmed target commands movement frame%0d",previous_frame);
                    if(expected_state==3 && (cx!=previous_cx || cy!=previous_cy))
                        $fatal(1,"HOLD changed remembered position");
                    if(expected_state==0 && (cx!=0 || cy!=0))
                        $fatal(1,"LOST retains expired position");
                end
                expected_fx=expected_valid ? (ex>6 ? 1 : (ex < -6 ? -1 : 0)) : 0;
                expected_fy=expected_valid ? (ey>6 ? 1 : (ey < -6 ? -1 : 0)) : 0;
                if(fx!=expected_fx || fy!=expected_fy) $fatal(1,"follow direction mismatch");
                $fdisplay(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",previous_frame,present(previous_frame),ball_x(previous_frame),ball_y(previous_frame),state,valid,cx,cy,ex,fx,fy);
                updates=updates+1;
            end else if({state,valid,cx,cy}!=={previous_state,previous_valid,previous_cx,previous_cy})
                $fatal(1,"metadata changed outside publication pulse");
            previous_state=state; previous_valid=valid; previous_cx=cx; previous_cy=cy;
            cycle=cycle+1;
        end
    endtask
    initial begin
        trace=$fopen("occlusion_trace.csv","w");
        if(!trace) $fatal(1,"cannot open trace");
        $fdisplay(trace,"frame,visible,ball_x,ball_y,state,valid,center_x,center_y,error_x,follow_x,follow_y");
        repeat(3) @(negedge clk);
        rst_n=1;
        for(f=0;f<FRAME_COUNT;f=f+1) begin
            previous_frame=f-1;
            for(y=0;y<HEIGHT+8;y=y+1) begin
                for(x=0;x<H_TOTAL;x=x+1) begin
                    active=(y>=4 && y<HEIGHT+4 && x>=4 && x<WIDTH+4);
                    px=x-4; py=y-4;
                    p=0; sample=0;
                    if(active) begin
                        // Neutral background: quantized RGB565 cannot pass red gate.
                        p=16'h8410;
                        dx=px-ball_x(f); dy=py-ball_y(f);
                        if(present(f) && dx*dx+dy*dy<=144) p=16'hf800;
                        // A half occluder must pause movement rather than shift
                        // the actuator toward the visible half's bbox center.
                        if(f==20 && px<ball_x(f)) p=16'h8410;
                        // A larger red object passes far from the remembered
                        // target on the SAME rows. ROI must keep the real ball.
                        dx=px-(WIDTH*3)/4; dy=py-HEIGHT/2;
                        if(f==9 && dx*dx+dy*dy<=400) p=16'hf800;
                        // Border pixels never intersect a target, HUD, or center aids.
                        sample=(px<4 || px>=WIDTH-4 || py>=HEIGHT-4) &&
                            !(px>=8 && px<200 && py>=8 && py<40);
                    end
                    tick(p,active,x<2,y==0,sample);
                end
            end
        end
        for(j=0;j<DELAY+8;j=j+1) tick(0,0,0,0,0);
        if(updates!=FRAME_COUNT-1) $fatal(1,"missing/extra frame update%0d",updates);
        if(rgb_checks<WIDTH*4) $fatal(1,"insufficient original-pixel checks");
        $fclose(trace);
        $display("PASS pipeline %0dx%0d frames=%0d latency=%0d control_checks=%0d passthrough_pixels=%0d updates=%0d",WIDTH,HEIGHT,FRAME_COUNT,DELAY,checks,rgb_checks,updates);
        $finish;
    end
endmodule
