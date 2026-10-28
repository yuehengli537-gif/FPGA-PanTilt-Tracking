// PT20 v1 fixed-size request/reply controller. Command positions are pulse
// widths, not measured shaft angles. Only complete CRC-checked packets act.
module servo_uart_control #(
    parameter integer CLK_HZ=27000000,
    parameter integer WATCHDOG_MS=1000,
    parameter integer RX_TIMEOUT_MS=20,
    parameter integer AUTO_SUPPORT=0,
    parameter integer AUTO_PAN_REVERSE=0,
    parameter integer AUTO_TILT_REVERSE=0,
    parameter integer AUTO_STALE_MS=100
)(
    input wire clk,rst_n,
    input wire [7:0] rx_data,input wire rx_valid,rx_error,
    output wire [7:0] tx_data,output wire tx_valid,input wire tx_ready,
    output reg servo_pan_pwm,servo_tilt_pwm,
    output reg enabled,timeout_hold,
    // Coherent camera snapshot, already synchronized into clk by the caller.
    input wire measurement_update,target_valid,
    input wire [1:0] track_state,
    input wire signed [12:0] error_x,error_y,
    output reg auto_mode,
    output wire moving,
    output reg [11:0] current_pan_us,current_tilt_us,target_pan_us,target_tilt_us
);
    localparam integer CYCLES_PER_US=CLK_HZ/1000000;
    localparam integer FRAME_CYCLES=CYCLES_PER_US*20000;
    localparam integer FRAME_BITS=$clog2(FRAME_CYCLES);
    localparam integer RX_TIMEOUT_CYCLES=(CLK_HZ/1000)*RX_TIMEOUT_MS;
    localparam integer RX_TIMER_BITS=(RX_TIMEOUT_CYCLES>1)?$clog2(RX_TIMEOUT_CYCLES):1;
    localparam integer WATCHDOG_CYCLES=(CLK_HZ/1000)*WATCHDOG_MS;
    localparam integer WD_BITS=(WATCHDOG_CYCLES>1)?$clog2(WATCHDOG_CYCLES):1;
    localparam integer HOME_US=1500;
    localparam integer STEP_US=2;
    reg [11:0] pan_min_us,pan_max_us,tilt_min_us,tilt_max_us;
    reg [FRAME_BITS-1:0] frame_cycle;
    reg pwm_started;
    reg [WD_BITS-1:0] watchdog_count;
    localparam integer AUTO_STALE_CYCLES=(CLK_HZ/1000)*AUTO_STALE_MS;
    localparam integer AUTO_AGE_BITS=(AUTO_STALE_CYCLES>1)?$clog2(AUTO_STALE_CYCLES+1):1;
    reg [AUTO_AGE_BITS-1:0] measurement_age;
    reg measured_locked;
    reg signed [12:0] measured_error_x,measured_error_y;
    reg [1:0] pan_direction,tilt_direction; // 0=stop,1=positive,2=negative
    wire camera_update=(AUTO_SUPPORT!=0)&&measurement_update;
    wire effective_locked=camera_update?(target_valid&&(track_state==2'd2)):measured_locked;
    wire signed [12:0] effective_error_x=camera_update?error_x:measured_error_x;
    wire signed [12:0] effective_error_y=camera_update?error_y:measured_error_y;
    wire camera_fresh=camera_update||(measurement_age<AUTO_STALE_CYCLES);
    wire auto_can_move=(AUTO_SUPPORT!=0)&&auto_mode&&effective_locked&&camera_fresh&&!timeout_hold;

    function [1:0] follow_direction;
        input [1:0] previous;
        input signed [12:0] error;
        begin
            if(error>=13'sd48)follow_direction=1;
            else if(error<=-13'sd48)follow_direction=2;
            else if(previous==1&&error>13'sd32)follow_direction=1;
            else if(previous==2&&error< -13'sd32)follow_direction=2;
            else follow_direction=0;
        end
    endfunction
    wire [1:0] next_pan_direction=follow_direction(pan_direction,effective_error_x);
    wire [1:0] next_tilt_direction=follow_direction(tilt_direction,effective_error_y);
    function [11:0] follow_step;
        input [11:0] position,minimum,maximum;
        input [1:0] direction;
        input signed [12:0] error;
        input reverse;
        reg [11:0] step;
        begin
            step=(error>=13'sd128||error<=-13'sd128)?12'd8:12'd2;
            if(direction==0)follow_step=position;
            else if((direction==1)^reverse)begin
                if(position+step>=maximum)follow_step=maximum;
                else follow_step=position+step;
            end else begin
                if(position<=minimum+step)follow_step=minimum;
                else follow_step=position-step;
            end
        end
    endfunction
    wire [11:0] auto_pan_next=follow_step(current_pan_us,pan_min_us,pan_max_us,
        next_pan_direction,effective_error_x,AUTO_PAN_REVERSE!=0);
    wire [11:0] auto_tilt_next=follow_step(current_tilt_us,tilt_min_us,tilt_max_us,
        next_tilt_direction,effective_error_y,AUTO_TILT_REVERSE!=0);
    assign moving=enabled && ((current_pan_us!=target_pan_us)||(current_tilt_us!=target_tilt_us)||
        (auto_can_move&&((auto_pan_next!=current_pan_us)||(auto_tilt_next!=current_tilt_us))));

    // Age and payload update independently of mode so mode entry can use the
    // most recent coherent measurement. Boot and any stream stall are stale.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            measurement_age<=AUTO_STALE_CYCLES;measured_locked<=0;
            measured_error_x<=0;measured_error_y<=0;
        end else if(camera_update)begin
            measurement_age<=0;measured_locked<=target_valid&&(track_state==2'd2);
            measured_error_x<=error_x;measured_error_y<=error_y;
        end else if(measurement_age<AUTO_STALE_CYCLES)
            measurement_age<=measurement_age+1'b1;
    end

    function [15:0] crc16_byte;
        input [15:0] crc;input [7:0] value;
        integer bit_number;reg [15:0] tmp;
        begin
            tmp=crc^{value,8'h00};
            for(bit_number=0;bit_number<8;bit_number=bit_number+1)
                if(tmp[15])tmp=(tmp<<1)^16'h1021;else tmp=tmp<<1;
            crc16_byte=tmp;
        end
    endfunction

    reg [3:0] rx_index;
    reg [7:0] rx_fields[0:11]; // packet bytes 2..13
    reg [15:0] rx_crc;
    reg [7:0] rx_crc_high;
    reg [RX_TIMER_BITS-1:0] rx_timeout;
    reg command_valid;
    reg [7:0] command_type,command_seq;
    reg [63:0] command_payload;
    reg reply_pending,reply_start,reply_sending;
    reg [7:0] reply_type,reply_seq,reply_error,reply_command;
    reg [7:0] reply_bytes[0:13];
    reg [3:0] tx_index;
    reg [15:0] tx_crc;
    wire reply_busy=reply_pending||reply_start||reply_sending||!tx_ready;
    integer reset_byte;
    // A framing error, inter-byte timeout or busy transmitter discards a
    // partial request. A repeated A5 can still begin the two-byte sync word.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            rx_index<=0;rx_crc<=16'hffff;rx_crc_high<=0;rx_timeout<=0;
            command_valid<=0;command_type<=0;command_seq<=0;command_payload<=0;
            for(reset_byte=0;reset_byte<12;reset_byte=reset_byte+1)rx_fields[reset_byte]<=0;
        end else begin
            command_valid<=0;
            if(reply_busy||rx_error)begin rx_index<=0;rx_timeout<=0;rx_crc<=16'hffff;end
            else if(rx_valid)begin
                rx_timeout<=0;
                case(rx_index)
                    0:if(rx_data==8'ha5)begin rx_index<=1;rx_crc<=16'hffff;end
                    1:if(rx_data==8'h5a)rx_index<=2;
                      else if(rx_data!=8'ha5)rx_index<=0;
                    14:begin rx_crc_high<=rx_data;rx_index<=15;end
                    15:begin
                        rx_index<=0;
                        if(rx_fields[0]==8'h01 && rx_fields[3]==0 &&
                           {rx_crc_high,rx_data}==rx_crc)begin
                            command_valid<=1;command_type<=rx_fields[1];command_seq<=rx_fields[2];
                            command_payload<={rx_fields[11],rx_fields[10],rx_fields[9],rx_fields[8],
                                              rx_fields[7],rx_fields[6],rx_fields[5],rx_fields[4]};
                        end else if(rx_data==8'ha5)begin rx_index<=1;rx_crc<=16'hffff;end
                    end
                    default:begin
                        rx_fields[rx_index-2]<=rx_data;
                        rx_crc<=crc16_byte(rx_crc,rx_data);
                        rx_index<=rx_index+1'b1;
                    end
                endcase
            end else if(rx_index!=0)begin
                if(rx_timeout==RX_TIMEOUT_CYCLES-1)begin
                    rx_index<=0;rx_timeout<=0;rx_crc<=16'hffff;
                end else rx_timeout<=rx_timeout+1'b1;
            end else rx_timeout<=0;
        end
    end

    // The reply is a snapshot: flags and all four words are latched together.
    // Its CRC is accumulated once per accepted outgoing byte, never from live
    // position registers. While sending, incoming requests are discarded.
    assign tx_valid=reply_sending;
    assign tx_data=(tx_index<14)?reply_bytes[tx_index]:
                   (tx_index==14)?tx_crc[15:8]:tx_crc[7:0];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin reply_sending<=0;tx_index<=0;tx_crc<=16'hffff;end
        else if(reply_start)begin reply_sending<=1;tx_index<=0;tx_crc<=16'hffff;end
        else if(reply_sending&&tx_ready)begin
            if(tx_index>=2&&tx_index<=13)tx_crc<=crc16_byte(tx_crc,reply_bytes[tx_index]);
            if(tx_index==15)begin reply_sending<=0;tx_index<=0;end
            else tx_index<=tx_index+1'b1;
        end
    end

    wire [15:0] request_word0=command_payload[15:0];
    wire [15:0] request_word1=command_payload[31:16];
    wire [15:0] request_word2=command_payload[47:32];
    wire [15:0] request_word3=command_payload[63:48];
    integer reply_byte;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            enabled<=0;timeout_hold<=0;auto_mode<=0;pan_direction<=0;tilt_direction<=0;
            servo_pan_pwm<=0;servo_tilt_pwm<=0;
            current_pan_us<=HOME_US;current_tilt_us<=HOME_US;
            target_pan_us<=HOME_US;target_tilt_us<=HOME_US;
            pan_min_us<=1300;pan_max_us<=1700;tilt_min_us<=1460;tilt_max_us<=1540;
            frame_cycle<=0;pwm_started<=0;watchdog_count<=0;
            reply_pending<=0;reply_start<=0;reply_type<=0;reply_seq<=0;reply_error<=0;reply_command<=0;
            for(reply_byte=0;reply_byte<14;reply_byte=reply_byte+1)reply_bytes[reply_byte]<=0;
        end else begin
            reply_start<=0;
            if(!auto_mode||!auto_can_move)begin pan_direction<=0;tilt_direction<=0;end
            else if(camera_update)begin
                pan_direction<=next_pan_direction;tilt_direction<=next_tilt_direction;
            end
            if(!enabled)begin
                servo_pan_pwm<=0;servo_tilt_pwm<=0;frame_cycle<=0;pwm_started<=0;
            end else if(!pwm_started)begin
                // ENABLE from off starts both axes with a complete neutral pulse.
                frame_cycle<=0;pwm_started<=1;servo_pan_pwm<=1;servo_tilt_pwm<=1;
            end else if(frame_cycle==FRAME_CYCLES-1)begin
                frame_cycle<=0;servo_pan_pwm<=1;servo_tilt_pwm<=1;
                if(auto_mode)begin
                    if(auto_can_move)begin
                        // One bounded step from the pulse actually being sent;
                        // there is no distant queued auto target to run toward.
                        current_pan_us<=auto_pan_next;current_tilt_us<=auto_tilt_next;
                        target_pan_us<=auto_pan_next;target_tilt_us<=auto_tilt_next;
                        pan_direction<=next_pan_direction;tilt_direction<=next_tilt_direction;
                    end else begin
                        target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                    end
                end else begin
                if(current_pan_us<target_pan_us)begin
                    if(current_pan_us+STEP_US>=target_pan_us)current_pan_us<=target_pan_us;
                    else current_pan_us<=current_pan_us+STEP_US;
                end else if(current_pan_us>target_pan_us)begin
                    if(current_pan_us<=target_pan_us+STEP_US)current_pan_us<=target_pan_us;
                    else current_pan_us<=current_pan_us-STEP_US;
                end
                if(current_tilt_us<target_tilt_us)begin
                    if(current_tilt_us+STEP_US>=target_tilt_us)current_tilt_us<=target_tilt_us;
                    else current_tilt_us<=current_tilt_us+STEP_US;
                end else if(current_tilt_us>target_tilt_us)begin
                    if(current_tilt_us<=target_tilt_us+STEP_US)current_tilt_us<=target_tilt_us;
                    else current_tilt_us<=current_tilt_us-STEP_US;
                end
                end
            end else begin
                frame_cycle<=frame_cycle+1'b1;
                if(frame_cycle==current_pan_us*CYCLES_PER_US-1)servo_pan_pwm<=0;
                if(frame_cycle==current_tilt_us*CYCLES_PER_US-1)servo_tilt_pwm<=0;
            end

            // Invalid/HOLD/LOST snapshots or staleness cancel auto movement
            // immediately, including a measurement arriving on a PWM boundary.
            if(auto_mode&&!auto_can_move)begin
                target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;
            end
            if(command_valid)watchdog_count<=0;
            else if(watchdog_count<WATCHDOG_CYCLES-1)watchdog_count<=watchdog_count+1'b1;
            else if(enabled&&!timeout_hold)begin
                timeout_hold<=1;auto_mode<=0;pan_direction<=0;tilt_direction<=0;
                target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                // At a frame boundary HOLD wins over the normal next step.
                current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;
            end

            if(command_valid)begin
                reply_pending<=1;reply_seq<=command_seq;reply_command<=command_type;
                reply_type<=8'h90;reply_error<=0;
                case(command_type)
                    8'h01:if(command_payload==0)reply_type<=8'h81;
                          else begin reply_type<=8'he0;reply_error<=2;end
                    8'h02:if(command_payload!=0)begin reply_type<=8'he0;reply_error<=2;end
                    8'h10:if(!enabled)begin reply_type<=8'he0;reply_error<=3;end
                          else if(command_payload[63:32]!=0 ||
                                  request_word0<pan_min_us || request_word0>pan_max_us ||
                                  request_word1<tilt_min_us || request_word1>tilt_max_us)begin
                              reply_type<=8'he0;reply_error<=2;
                          end else begin
                              target_pan_us<=request_word0[11:0];target_tilt_us<=request_word1[11:0];timeout_hold<=0;
                              auto_mode<=0;pan_direction<=0;tilt_direction<=0;
                              // Manual takeover wins over a simultaneous auto step.
                              if(auto_mode)begin current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;end
                          end
                    8'h11:if(command_payload[63:8]!=0||command_payload[7:0]>1)begin
                              reply_type<=8'he0;reply_error<=2;
                          end else begin
                              timeout_hold<=0;
                              if(command_payload[0])begin
                                  if(!enabled)begin
                                      enabled<=1;auto_mode<=0;pan_direction<=0;tilt_direction<=0;current_pan_us<=HOME_US;current_tilt_us<=HOME_US;
                                      target_pan_us<=HOME_US;target_tilt_us<=HOME_US;frame_cycle<=0;pwm_started<=0;
                                  end
                              end else begin
                                  enabled<=0;auto_mode<=0;pan_direction<=0;tilt_direction<=0;servo_pan_pwm<=0;servo_tilt_pwm<=0;frame_cycle<=0;pwm_started<=0;
                                  target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                                  current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;
                              end
                          end
                    8'h12:if(!enabled)begin reply_type<=8'he0;reply_error<=3;end
                          else if(command_payload!=0)begin reply_type<=8'he0;reply_error<=2;end
                          else begin
                              target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                              current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;timeout_hold<=0;
                              auto_mode<=0;pan_direction<=0;tilt_direction<=0;
                          end
                    8'h13:if(!enabled)begin reply_type<=8'he0;reply_error<=3;end
                          else if(command_payload!=0)begin reply_type<=8'he0;reply_error<=2;end
                          else begin
                              target_pan_us<=HOME_US;target_tilt_us<=HOME_US;timeout_hold<=0;
                              auto_mode<=0;pan_direction<=0;tilt_direction<=0;
                              if(auto_mode)begin current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;end
                          end
                    8'h14:if(enabled||request_word0<500||request_word0>HOME_US||
                                  request_word1<HOME_US||request_word1>2500||
                                  request_word2<500||request_word2>HOME_US||
                                  request_word3<HOME_US||request_word3>2500)begin
                              reply_type<=8'he0;reply_error<=2;
                          end else begin
                              pan_min_us<=request_word0[11:0];pan_max_us<=request_word1[11:0];
                              tilt_min_us<=request_word2[11:0];tilt_max_us<=request_word3[11:0];reply_type<=8'h91;
                          end
                    8'h15:if(command_payload==0)reply_type<=8'h91;
                          else begin reply_type<=8'he0;reply_error<=2;end
                    8'h20:if(command_payload[63:8]!=0)begin reply_type<=8'he0;reply_error<=2;end
                          else if(command_payload[7:0]>1||
                                  (command_payload[0]&&(AUTO_SUPPORT==0)))begin
                              reply_type<=8'he0;reply_error<=4;
                          end else if(command_payload[0]&&!enabled)begin reply_type<=8'he0;reply_error<=3;end
                          else begin
                              auto_mode<=command_payload[0];timeout_hold<=0;pan_direction<=0;tilt_direction<=0;
                              // Mode changes hold this exact pulse; new movement
                              // starts only at a subsequent complete PWM frame.
                              target_pan_us<=current_pan_us;target_tilt_us<=current_tilt_us;
                              current_pan_us<=current_pan_us;current_tilt_us<=current_tilt_us;
                          end
                    default:begin reply_type<=8'he0;reply_error<=1;end
                endcase
            end
            if(reply_pending)begin
                reply_pending<=0;reply_start<=1;
                reply_bytes[0]<=8'ha5;reply_bytes[1]<=8'h5a;reply_bytes[2]<=1;
                reply_bytes[3]<=reply_type;reply_bytes[4]<=reply_seq;
                reply_bytes[5]<={4'b0000,auto_mode,timeout_hold,moving,enabled};
                for(reply_byte=6;reply_byte<14;reply_byte=reply_byte+1)reply_bytes[reply_byte]<=0;
                case(reply_type)
                    8'h81:begin
                        reply_bytes[6]<=8'h50;reply_bytes[7]<=8'h54;reply_bytes[8]<=8'h32;reply_bytes[9]<=8'h30;
                        reply_bytes[10]<=1;reply_bytes[11]<=0;reply_bytes[12]<=(AUTO_SUPPORT!=0)?3:1;reply_bytes[13]<=0;
                    end
                    8'h90:begin
                        reply_bytes[6]<=current_pan_us[7:0];reply_bytes[7]<={4'b0,current_pan_us[11:8]};
                        reply_bytes[8]<=current_tilt_us[7:0];reply_bytes[9]<={4'b0,current_tilt_us[11:8]};
                        reply_bytes[10]<=target_pan_us[7:0];reply_bytes[11]<={4'b0,target_pan_us[11:8]};
                        reply_bytes[12]<=target_tilt_us[7:0];reply_bytes[13]<={4'b0,target_tilt_us[11:8]};
                    end
                    8'h91:begin
                        reply_bytes[6]<=pan_min_us[7:0];reply_bytes[7]<={4'b0,pan_min_us[11:8]};
                        reply_bytes[8]<=pan_max_us[7:0];reply_bytes[9]<={4'b0,pan_max_us[11:8]};
                        reply_bytes[10]<=tilt_min_us[7:0];reply_bytes[11]<={4'b0,tilt_min_us[11:8]};
                        reply_bytes[12]<=tilt_max_us[7:0];reply_bytes[13]<={4'b0,tilt_max_us[11:8]};
                    end
                    8'he0:begin reply_bytes[6]<=reply_error;reply_bytes[7]<=reply_command;end
                    default:begin end
                endcase
            end
        end
    end
endmodule
