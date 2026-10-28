`timescale 1ns/1ps
// Measurement sampled at N is published at N+1. HOLD keeps historical
// coordinates only for search/display: target_valid is always zero in HOLD.
module red_ball_lock #(
    parameter integer WIDTH=1280, HEIGHT=720,
    parameter integer HOLD_FRAMES=45, CONFIRM_FRAMES=3, MATCH_RADIUS=128,
    parameter integer FRAME_TIMEOUT_CYCLES=7425000
)(
    input wire clk, rst_n,
    input wire measurement_update, measurement_valid,
    input wire [11:0] measurement_left, measurement_top, measurement_right, measurement_bottom,
    input wire [11:0] measurement_x, measurement_y,
    output reg frame_update, target_valid,
    output reg [1:0] track_state,
    output reg [11:0] bbox_left, bbox_top, bbox_right, bbox_bottom, center_x, center_y,
    output wire search_enable,
    output wire [11:0] search_left, search_top, search_right, search_bottom
);
    localparam [1:0] LOST=0, ACQUIRE=1, TRACK=2, HOLD=3;
    localparam integer CL=(CONFIRM_FRAMES<1)?1:CONFIRM_FRAMES;
    localparam integer HL=(HOLD_FRAMES<1)?1:HOLD_FRAMES;
    localparam integer TL=(FRAME_TIMEOUT_CYCLES<1)?1:FRAME_TIMEOUT_CYCLES;
    localparam integer CB=(CL<2)?1:$clog2(CL+1);
    localparam integer HB=(HL<2)?1:$clog2(HL+1);
    localparam integer WB=(TL<2)?1:$clog2(TL+1);
    localparam [CB-1:0] CONFIRM_PREVIOUS=CL-1;
    localparam [HB-1:0] HOLD_PREVIOUS=HL-1;
    localparam [WB-1:0] WATCHDOG_LAST=TL-1;
    localparam [12:0] RADIUS_U=MATCH_RADIUS, LAST_X=WIDTH-1, LAST_Y=HEIGHT-1;
    reg pending_update, valid_q;
    reg [11:0] left_q,top_q,right_q,bottom_q,x_q,y_q;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pending_update<=0; valid_q<=0;
            left_q<=0;top_q<=0;right_q<=0;bottom_q<=0;x_q<=0;y_q<=0;
        end else begin
            pending_update<=measurement_update;
            if(measurement_update) begin
                valid_q<=measurement_valid;
                left_q<=measurement_left;top_q<=measurement_top;
                right_q<=measurement_right;bottom_q<=measurement_bottom;
                x_q<=measurement_x;y_q<=measurement_y;
            end
        end
    end
    reg [11:0] candidate_x,candidate_y;
    reg [CB-1:0] confirm_count;
    reg [HB-1:0] hold_count;
    reg [WB-1:0] watchdog_count;
    reg watchdog_expired;
    wire [12:0] trusted_dx=(x_q>=center_x)?{1'b0,x_q}-{1'b0,center_x}:{1'b0,center_x}-{1'b0,x_q};
    wire [12:0] trusted_dy=(y_q>=center_y)?{1'b0,y_q}-{1'b0,center_y}:{1'b0,center_y}-{1'b0,y_q};
    wire [12:0] candidate_dx=(x_q>=candidate_x)?{1'b0,x_q}-{1'b0,candidate_x}:{1'b0,candidate_x}-{1'b0,x_q};
    wire [12:0] candidate_dy=(y_q>=candidate_y)?{1'b0,y_q}-{1'b0,candidate_y}:{1'b0,candidate_y}-{1'b0,y_q};
    wire near_trusted=trusted_dx<=RADIUS_U && trusted_dy<=RADIUS_U;
    wire near_candidate=candidate_dx<=RADIUS_U && candidate_dy<=RADIUS_U;
    wire [12:0] smooth_x_sum={1'b0,center_x}+{1'b0,x_q};
    wire [12:0] smooth_y_sum={1'b0,center_y}+{1'b0,y_q};
    wire acquire_complete=valid_q && ((CL==1) || (near_candidate && confirm_count>=CONFIRM_PREVIOUS));
    wire hold_complete=valid_q && near_trusted && ((CL==1) ||
        (confirm_count!=0 && near_candidate && confirm_count>=CONFIRM_PREVIOUS));
    wire watchdog_fire=!measurement_update && !pending_update && !watchdog_expired && watchdog_count==WATCHDOG_LAST;
    wire [12:0] expanded_right={1'b0,bbox_right}+RADIUS_U;
    wire [12:0] expanded_bottom={1'b0,bbox_bottom}+RADIUS_U;
    wire [12:0] reduced_left={1'b0,bbox_left}-RADIUS_U;
    wire [12:0] reduced_top={1'b0,bbox_top}-RADIUS_U;
    assign search_enable=track_state==TRACK || track_state==HOLD;
    assign search_left=!search_enable?12'd0:({1'b0,bbox_left}>RADIUS_U?reduced_left[11:0]:12'd0);
    assign search_top=!search_enable?12'd0:({1'b0,bbox_top}>RADIUS_U?reduced_top[11:0]:12'd0);
    assign search_right=(!search_enable || expanded_right>LAST_X)?LAST_X[11:0]:expanded_right[11:0];
    assign search_bottom=(!search_enable || expanded_bottom>LAST_Y)?LAST_Y[11:0]:expanded_bottom[11:0];

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            frame_update<=0;target_valid<=0;track_state<=LOST;
            bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
            candidate_x<=0;candidate_y<=0;confirm_count<=0;hold_count<=0;
            watchdog_count<=0;watchdog_expired<=0;
        end else begin
            frame_update<=0;
            // An incoming update on the deadline wins over expiration.
            if(measurement_update || pending_update) begin
                watchdog_count<=0;watchdog_expired<=0;
            end else if(!watchdog_expired) begin
                if(watchdog_fire) watchdog_expired<=1;
                else watchdog_count<=watchdog_count+1'b1;
            end
            if(watchdog_fire) begin
                frame_update<=1;target_valid<=0;track_state<=LOST;
                bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
                candidate_x<=0;candidate_y<=0;confirm_count<=0;hold_count<=0;
            end else if(pending_update) begin
                frame_update<=1;
                case(track_state)
                    LOST: begin
                        target_valid<=0;hold_count<=0;confirm_count<=0;
                        bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
                        if(valid_q) begin
                            candidate_x<=x_q;candidate_y<=y_q;
                            if(CL==1) begin
                                track_state<=TRACK;target_valid<=1;
                                bbox_left<=left_q;bbox_top<=top_q;bbox_right<=right_q;bbox_bottom<=bottom_q;
                                center_x<=x_q;center_y<=y_q;
                            end else begin track_state<=ACQUIRE;confirm_count<=1;end
                        end
                    end
                    ACQUIRE: begin
                        target_valid<=0;
                        if(!valid_q) begin
                            track_state<=LOST;confirm_count<=0;candidate_x<=0;candidate_y<=0;
                        end else if(acquire_complete) begin
                            track_state<=TRACK;target_valid<=1;confirm_count<=0;
                            bbox_left<=left_q;bbox_top<=top_q;bbox_right<=right_q;bbox_bottom<=bottom_q;
                            center_x<=x_q;center_y<=y_q;
                        end else begin
                            candidate_x<=x_q;candidate_y<=y_q;
                            if(near_candidate) confirm_count<=confirm_count+1'b1;
                            else confirm_count<=1;
                        end
                    end
                    TRACK: begin
                        confirm_count<=0;
                        if(valid_q && near_trusted) begin
                            target_valid<=1;hold_count<=0;
                            bbox_left<=left_q;bbox_top<=top_q;bbox_right<=right_q;bbox_bottom<=bottom_q;
                            center_x<=smooth_x_sum[12:1];center_y<=smooth_y_sum[12:1];
                        end else begin
                            target_valid<=0;
                            if(HL==1) begin
                                track_state<=LOST;hold_count<=0;
                                bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
                                candidate_x<=0;candidate_y<=0;
                            end else begin track_state<=HOLD;hold_count<=1;end
                        end
                    end
                    HOLD: begin
                        target_valid<=0;
                        if(hold_complete) begin
                            track_state<=TRACK;target_valid<=1;confirm_count<=0;hold_count<=0;
                            bbox_left<=left_q;bbox_top<=top_q;bbox_right<=right_q;bbox_bottom<=bottom_q;
                            center_x<=x_q;center_y<=y_q;
                        end else if(hold_count>=HOLD_PREVIOUS) begin
                            track_state<=LOST;confirm_count<=0;hold_count<=0;
                            bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
                            candidate_x<=0;candidate_y<=0;
                        end else begin
                            hold_count<=hold_count+1'b1;
                            if(valid_q && near_trusted) begin
                                candidate_x<=x_q;candidate_y<=y_q;
                                if(confirm_count!=0 && near_candidate) confirm_count<=confirm_count+1'b1;
                                else confirm_count<=1;
                            end else confirm_count<=0;
                        end
                    end
                    default: begin
                        track_state<=LOST;target_valid<=0;
                        bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;center_x<=0;center_y<=0;
                        candidate_x<=0;candidate_y<=0;confirm_count<=0;hold_count<=0;
                    end
                endcase
            end
        end
    end
endmodule
