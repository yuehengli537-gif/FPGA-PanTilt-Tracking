`timescale 1ns/1ps
// Default video uses one registered stage; optional diagnostics use two.
// All RGB and sync signals share the selected latency. Detection belongs to
// the preceding frame; metadata is frozen at VS rising for a tear-free frame.
// Direction labels describe the target's position in the original image.
module red_ball_overlay #(
    parameter integer WIDTH=1280, HEIGHT=720, DEAD_ZONE=32,
    parameter integer SHOW_SERVO_DEBUG=0
) (
    input wire clk, rst_n,
    input wire [7:0] in_r, in_g, in_b,
    input wire in_de, in_hs, in_vs,
    input wire target_valid,
    input wire [1:0] track_state,
    input wire [11:0] bbox_left, bbox_top, bbox_right, bbox_bottom,
    input wire [11:0] center_x, center_y,
    // PWM commands only, not measured servo angle or presence feedback.
    input wire [15:0] servo_pan_us, servo_tilt_us,
    input wire [1:0] servo_pan_limit, servo_tilt_limit, // 0:none, 1:LOW, 2:HIGH, 3:OFF
    input wire servo_enabled, servo_source_alive, servo_stale,
    // Runtime-only shape diagnostic inputs; ignored unless SHOW_SERVO_DEBUG=2.
    input wire diag_candidate_valid,
    input wire [7:0] diag_flags, diag_candidate_flags,
    input wire [12:0] diag_width, diag_height,
    input wire [19:0] diag_pixels,
    input wire [11:0] diag_left, diag_top, diag_right, diag_bottom,
    output reg [7:0] out_r, out_g, out_b,
    output reg out_de, out_hs, out_vs
);
    localparam integer CX=WIDTH/2, CY=HEIGHT/2;
    localparam integer AX0=CX-DEAD_ZONE-20, AX1=CX+DEAD_ZONE+20;
    localparam integer AY0=CY-DEAD_ZONE-20, AY1=CY+DEAD_ZONE+20;
    localparam [2:0] AX0_LOW=AX0, AX1_LOW=AX1, AY0_LOW=AY0, AY1_LOW=AY1;
    localparam [23:0] GREEN=24'h20ff40, YELLOW=24'hffff00;
    localparam [23:0] WHITE=24'he0e0e0, GRAY=24'h606060;
    reg [11:0] x,y;
    reg previous_vs, valid_q;
    reg [1:0] state_q;
    reg [11:0] left_q,top_q,right_q,bottom_q,cx_q,cy_q;
    reg raw_valid_q;
    reg [11:0] raw_left_q,raw_top_q,raw_right_q,raw_bottom_q;
    // Decode the immutable status text at the same VS edge as its metadata.
    // Use incoming values here, not the previous frame's *_q snapshot. This
    // removes state/direction -> text decoding from the one-cycle font path.
    reg [127:0] status_text_q,status_text_next;
    wire status_tracking=target_valid && track_state==2;
    wire status_left=status_tracking && center_x<CX-DEAD_ZONE;
    wire status_right=status_tracking && center_x>CX+DEAD_ZONE;
    wire status_up=status_tracking && center_y<CY-DEAD_ZONE;
    wire status_down=status_tracking && center_y>CY+DEAD_ZONE;
    wire status_centered=status_tracking && !(status_left || status_right || status_up || status_down);
    always @* begin
        status_text_next="                ";
        if(track_state==3) status_text_next="HOLD            ";
        else if(track_state==1) status_text_next="SEARCH          ";
        else if(!status_tracking) status_text_next="LOST            ";
        else if(status_centered) status_text_next="CENTER          ";
        else begin
            if(status_left) status_text_next="LEFT            ";
            else if(status_right) status_text_next="RIGHT           ";
            if(status_up) status_text_next[79:64]="UP";
            else if(status_down) status_text_next[79:48]="DOWN";
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0; y<=0; previous_vs<=0; valid_q<=0; state_q<=0;
            left_q<=0; top_q<=0; right_q<=0; bottom_q<=0;
            cx_q<=0; cy_q<=0;status_text_q<="LOST            ";
            raw_valid_q<=0;raw_left_q<=0;raw_top_q<=0;raw_right_q<=0;raw_bottom_q<=0;
        end else begin
            previous_vs<=in_vs;
            if(in_vs && !previous_vs) begin
                x<=0; y<=0; valid_q<=target_valid; state_q<=track_state;
                left_q<=bbox_left; top_q<=bbox_top;
                right_q<=bbox_right; bottom_q<=bbox_bottom;
                cx_q<=center_x; cy_q<=center_y;status_text_q<=status_text_next;
                raw_valid_q<=diag_candidate_valid;
                raw_left_q<=diag_left;raw_top_q<=diag_top;
                raw_right_q<=diag_right;raw_bottom_q<=diag_bottom;
            end else if(in_de) begin
                if(x==WIDTH-1) begin
                    x<=0; y<=(y==HEIGHT-1)?0:y+1'b1;
                end else x<=x+1'b1;
            end
        end
    end

    // A frame snapshot is converted during blanking/early rows. Sixteen
    // shift/add-3 cycles replace combinational division in the pixel path.
    // The first debug row starts at y=40, well after conversion completes.
    reg [35:0] pan_shift, tilt_shift, error_shift;
    reg [19:0] pan_bcd, tilt_bcd, error_bcd;
    reg [19:0] flags_shift,candidate_flags_shift;
    reg [11:0] flags_bcd,candidate_flags_bcd;
    reg [4:0] conversion_left;
    reg [1:0] pan_limit_q, tilt_limit_q;
    reg servo_enabled_q, source_alive_q, stale_q, error_negative_q;
    wire [15:0] incoming_error_magnitude=(target_valid && track_state==2) ?
        ((center_y<CY) ? CY-{1'b0,center_y} : {1'b0,center_y}-CY) : 16'd0;
    function [35:0] decimal_step;
        input [35:0] value;
        reg [35:0] adjusted;
        integer digit;
        begin
            adjusted=value;
            for(digit=0;digit<5;digit=digit+1)
                if(value[16+digit*4 +: 4]>=5)
                    adjusted[16+digit*4 +: 4]=value[16+digit*4 +: 4]+4'd3;
            decimal_step=adjusted<<1;
        end
    endfunction
    function [19:0] decimal_byte_step;
        input [19:0] value;
        reg [19:0] adjusted;
        integer digit;
        begin
            adjusted=value;
            for(digit=0;digit<3;digit=digit+1)
                if(value[8+digit*4 +: 4]>=5)
                    adjusted[8+digit*4 +: 4]=value[8+digit*4 +: 4]+4'd3;
            decimal_byte_step=adjusted<<1;
        end
    endfunction
    wire [19:0] flags_step=decimal_byte_step(flags_shift);
    wire [19:0] candidate_flags_step=decimal_byte_step(candidate_flags_shift);
    wire [35:0] pan_step=decimal_step(pan_shift);
    wire [35:0] tilt_step=decimal_step(tilt_shift);
    wire [35:0] error_step=decimal_step(error_shift);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pan_shift<=0;tilt_shift<=0;error_shift<=0;
            flags_shift<=0;candidate_flags_shift<=0;flags_bcd<=0;candidate_flags_bcd<=0;
            pan_bcd<=0;tilt_bcd<=0;error_bcd<=0;conversion_left<=0;
            pan_limit_q<=0;tilt_limit_q<=0;servo_enabled_q<=0;
            source_alive_q<=0;stale_q<=1;error_negative_q<=0;
        end else if(SHOW_SERVO_DEBUG!=0) begin
            if(in_vs && !previous_vs) begin
                if(SHOW_SERVO_DEBUG==2) begin
                    pan_shift<={23'd0,diag_width};tilt_shift<={23'd0,diag_height};
                    error_shift<={20'd0,diag_pixels[19:4]};
                    flags_shift<={12'd0,diag_flags};candidate_flags_shift<={12'd0,diag_candidate_flags};
                end else begin
                    pan_shift<={20'd0,servo_pan_us};tilt_shift<={20'd0,servo_tilt_us};
                    error_shift<={20'd0,incoming_error_magnitude};
                end
                conversion_left<=16;
                pan_limit_q<=servo_pan_limit;tilt_limit_q<=servo_tilt_limit;
                servo_enabled_q<=servo_enabled;source_alive_q<=servo_source_alive;stale_q<=servo_stale;
                error_negative_q<=target_valid && track_state==2 && center_y<CY;
            end else if(conversion_left!=0) begin
                pan_shift<=pan_step;tilt_shift<=tilt_step;error_shift<=error_step;
                conversion_left<=conversion_left-1'b1;
                if(SHOW_SERVO_DEBUG==2 && conversion_left>8) begin
                    flags_shift<=flags_step;candidate_flags_shift<=candidate_flags_step;
                    if(conversion_left==9) begin
                        flags_bcd<=flags_step[19:8];candidate_flags_bcd<=candidate_flags_step[19:8];
                    end
                end
                if(conversion_left==1) begin
                    pan_bcd<=pan_step[35:16];tilt_bcd<=tilt_step[35:16];error_bcd<=error_step[35:16];
                end
            end
        end
    end
    function [31:0] decimal_ascii;
        input [19:0] digits;
        begin
            // Never silently wrap a five-digit input into a plausible command.
            decimal_ascii=(digits[19:16]!=0) ? "OVFL" :
                {4'h3,digits[15:12],4'h3,digits[11:8],4'h3,digits[7:4],4'h3,digits[3:0]};
        end
    endfunction
    function [39:0] decimal_ascii5;
        input [19:0] digits;
        begin
            decimal_ascii5={4'h3,digits[19:16],4'h3,digits[15:12],4'h3,digits[11:8],4'h3,digits[7:4],4'h3,digits[3:0]};
        end
    endfunction
    function [23:0] decimal_ascii3;
        input [11:0] digits;
        begin
            decimal_ascii3={4'h3,digits[11:8],4'h3,digits[7:4],4'h3,digits[3:0]};
        end
    endfunction
    function [31:0] limit_ascii;
        input [1:0] limit_code;
        begin
            case(limit_code)
                1:limit_ascii="LOW ";2:limit_ascii="HIGH";3:limit_ascii="OFF ";
                default:limit_ascii="    ";
            endcase
        end
    endfunction

    wire tracking=(state_q==2) && valid_q;
    wire target_left=tracking && cx_q<CX-DEAD_ZONE;
    wire target_right=tracking && cx_q>CX+DEAD_ZONE;
    wire target_up=tracking && cy_q<CY-DEAD_ZONE;
    wire target_down=tracking && cy_q>CY+DEAD_ZONE;
    wire centered=tracking && !(target_left || target_right || target_up || target_down);
    wire [23:0] aim_color=!tracking?GRAY:(centered?GREEN:WHITE);
    wire [12:0] dx=(x>=CX)?({1'b0,x}-CX):(CX-{1'b0,x});
    wire [12:0] dy=(y>=CY)?({1'b0,y}-CY):(CY-{1'b0,y});
    wire [12:0] target_dx=(x>=cx_q)?({1'b0,x}-{1'b0,cx_q}):({1'b0,cx_q}-{1'b0,x});
    wire [12:0] target_dy=(y>=cy_q)?({1'b0,y}-{1'b0,cy_q}):({1'b0,cy_q}-{1'b0,y});
    wire box_border=x>=left_q && x<=right_q && y>=top_q && y<=bottom_q &&
        (x==left_q || x==right_q || y==top_q || y==bottom_q);
    wire box_pixel=tracking && box_border;
    wire raw_box_pixel=SHOW_SERVO_DEBUG==2 && raw_valid_q &&
        x>=raw_left_q && x<=raw_right_q && y>=raw_top_q && y<=raw_bottom_q &&
        (x==raw_left_q || x==raw_right_q || y==raw_top_q || y==raw_bottom_q);
    wire hold_box=(state_q==3) && right_q>left_q && bottom_q>top_q && box_border &&
        ((x==left_q || x==right_q) ? !y[2] : !x[2]);
    wire ball_cross=tracking && ((target_dx<=8 && target_dy<=1) || (target_dy<=8 && target_dx<=1));
    wire aim_cross=(dx<=10 && dy<=1) || (dy<=10 && dx<=1);
    wire aim_square=(dx==DEAD_ZONE && dy<=DEAD_ZONE) || (dy==DEAD_ZONE && dx<=DEAD_ZONE);
    // A head is only seven columns/rows deep. Full coordinate guards below
    // establish depth 0..6; modulo-eight subtraction is then exact. Avoid a
    // 32-bit integer subtraction/comparison on the single-cycle pixel path.
    wire [2:0] left_head_depth=x[2:0]-AX0_LOW;
    wire [2:0] right_head_depth=AX1_LOW-x[2:0];
    wire [2:0] up_head_depth=y[2:0]-AY0_LOW;
    wire [2:0] down_head_depth=AY1_LOW-y[2:0];
    wire left_arrow=target_left &&
        ((x>=AX0+6 && x<=AX0+16 && dy<=1) || (x>=AX0 && x<=AX0+6 && dy<={10'b0,left_head_depth}));
    wire right_arrow=target_right &&
        ((x>=AX1-16 && x<=AX1-6 && dy<=1) || (x>=AX1-6 && x<=AX1 && dy<={10'b0,right_head_depth}));
    wire up_arrow=target_up &&
        ((y>=AY0+6 && y<=AY0+16 && dx<=1) || (y>=AY0 && y<=AY0+6 && dx<={10'b0,up_head_depth}));
    wire down_arrow=target_down &&
        ((y>=AY1-16 && y<=AY1-6 && dx<=1) || (y>=AY1-6 && y<=AY1 && dx<={10'b0,down_head_depth}));

    // Two rows of 5x7 letters, scaled 2x in 16x16 cells. Bit-sliced cell
    // addressing avoids dividers/modulo logic in the single-cycle video path.
    function [34:0] glyph;
        input [7:0] c;
        begin
            case(c)
                "A": glyph=35'b01110100011000111111100011000110001;
                "B": glyph=35'b11110100011000111110100011000111110;
                "C": glyph=35'b01111100001000010000100001000001111;
                "D": glyph=35'b11110100011000110001100011000111110;
                "E": glyph=35'b11111100001000011110100001000011111;
                "F": glyph=35'b11111100001000011110100001000010000;
                "G": glyph=35'b01111100001000010111100011000101111;
                "H": glyph=35'b10001100011000111111100011000110001;
                "I": glyph=35'b01110001000010000100001000010001110;
                "L": glyph=35'b10000100001000010000100001000011111;
                "N": glyph=35'b10001110011010110011100011000110001;
                "O": glyph=35'b01110100011000110001100011000101110;
                "P": glyph=35'b11110100011000111110100001000010000;
                "R": glyph=35'b11110100011000111110101001001010001;
                "S": glyph=35'b01111100001000001110000010000111110;
                "T": glyph=35'b11111001000010000100001000010000100;
                "U": glyph=35'b10001100011000110001100011000101110;
                "W": glyph=35'b10001100011000110101101011101110001;
                default: glyph=0;
            endcase
            if(SHOW_SERVO_DEBUG!=0) begin
                case(c)
                "0": glyph=35'b01110100011001110101110011000101110;
                "1": glyph=35'b00100011000010000100001000010001110;
                "2": glyph=35'b01110100010000100010001000100011111;
                "3": glyph=35'b11110000010000101110000010000111110;
                "4": glyph=35'b00010001100101010010111110001000010;
                "5": glyph=35'b11111100001000011110000010000111110;
                "6": glyph=35'b01110100001000011110100011000101110;
                "7": glyph=35'b11111000010001000100010000100001000;
                "8": glyph=35'b01110100011000101110100011000101110;
                "9": glyph=35'b01110100011000101111000010000101110;
                "+": glyph=35'b00000001000010011111001000010000000;
                "-": glyph=35'b00000000000000011111000000000000000;
                "M": glyph=35'b10001110111010110101100011000110001;
                "V": glyph=35'b10001100011000110001100010101000100;
                "X": glyph=35'b10001100010101000100010101000110001;
                "Y": glyph=35'b10001100010101000100001000010000100;
                    default:begin end
                endcase
            end
        end
    endfunction
    // Explicit row/column selection replaces a wide variable index formed
    // from integer multiply/subtract. Invalid glyph coordinates remain blank.
    function glyph_ink;
        input [34:0] bits;
        input [2:0] gx,gy;
        reg [4:0] row_bits;
        begin
            case(gy)
                0:row_bits=bits[34:30];1:row_bits=bits[29:25];
                2:row_bits=bits[24:20];3:row_bits=bits[19:15];
                4:row_bits=bits[14:10];5:row_bits=bits[9:5];
                6:row_bits=bits[4:0];default:row_bits=0;
            endcase
            case(gx)
                0:glyph_ink=row_bits[4];1:glyph_ink=row_bits[3];
                2:glyph_ink=row_bits[2];3:glyph_ink=row_bits[1];
                4:glyph_ink=row_bits[0];default:glyph_ink=0;
            endcase
        end
    endfunction
    reg [7:0] character;
    reg [127:0] text_row;
    wire [11:0] label_x=x-12'd8, label_y=y-12'd8;
    wire [3:0] column=label_x[7:4];
    wire [2:0] glyph_x=label_x[3:1];
    wire [2:0] glyph_y=label_y[3:1];
    wire normal_panel=x>=8 && x<200 && y>=8 && y<40;
    wire debug_panel=SHOW_SERVO_DEBUG!=0 && x>=8 && x<264 && y>=40 && y<104;
    wire panel=normal_panel || debug_panel;
    reg [34:0] bitmap;
    reg [23:0] pixel, base_pixel;
    wire [23:0] font_color=debug_panel ? WHITE :
        ((label_y<16) ? 24'hc8f0ff : ((state_q==3) ? YELLOW : aim_color));
    always @* begin
        text_row="                ";
        if(label_y<16) text_row=(SHOW_SERVO_DEBUG==2)?"RED DIAG        ":"RED BALL        ";
        else if(label_y<32) text_row=status_text_q;
        else if(SHOW_SERVO_DEBUG==2) begin
            case(label_y[6:4])
                2:text_row={"WIDTH ",decimal_ascii(pan_bcd),"      "};
                3:text_row={"HEIGHT",decimal_ascii(tilt_bcd),"      "};
                4:text_row={"P16 ",decimal_ascii5(error_bcd),"       "};
                5:text_row={"G ",decimal_ascii3(flags_bcd)," C ",decimal_ascii3(candidate_flags_bcd),"     "};
                default:text_row="                ";
            endcase
        end else if(SHOW_SERVO_DEBUG!=0) begin
            case(label_y[6:4])
                2:text_row={"PAN ",decimal_ascii(pan_bcd)," ",limit_ascii(pan_limit_q),"   "};
                3:text_row={"TILT",decimal_ascii(tilt_bcd)," ",limit_ascii(tilt_limit_q),"   "};
                4:text_row={"DY ",(error_negative_q ? 8'h2d : 8'h2b),decimal_ascii(error_bcd)," PX     "};
                5:begin
                    if(!servo_enabled_q) text_row="PWM OFF         ";
                    else if(!source_alive_q) text_row="NO FRAME        ";
                    else if(stale_q) text_row="STALE DATA      ";
                    else if(!tracking) text_row="WAIT TARGET     ";
                    else text_row="PWM READY       ";
                end
                default:text_row="                ";
            endcase
        end
        character=" ";
        case(column)
            0:character=text_row[127:120];1:character=text_row[119:112];
            2:character=text_row[111:104];3:character=text_row[103:96];
            4:character=text_row[95:88];5:character=text_row[87:80];
            6:character=text_row[79:72];7:character=text_row[71:64];
            8:character=text_row[63:56];9:character=text_row[55:48];
            10:character=text_row[47:40];11:character=text_row[39:32];
            12:character=text_row[31:24];13:character=text_row[23:16];
            14:character=text_row[15:8];15:character=text_row[7:0];
        endcase
        base_pixel={in_r,in_g,in_b};
        if(raw_box_pixel) base_pixel=24'hff8000;
        if(box_pixel) base_pixel=GREEN;
        if(hold_box) base_pixel=YELLOW;
        if(ball_cross) base_pixel=YELLOW;
        if(aim_cross || aim_square) base_pixel=aim_color;
        if(left_arrow || right_arrow || up_arrow || down_arrow) base_pixel=YELLOW;
        bitmap=glyph(character);
        pixel=base_pixel;
        if(panel) begin
            pixel=24'h081820;
            if(glyph_ink(bitmap,glyph_x,glyph_y)) pixel=font_color;
        end
    end
    generate if(SHOW_SERVO_DEBUG==0) begin : normal_video
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            {out_r,out_g,out_b}<=0; out_de<=0; out_hs<=0; out_vs<=0;
        end else begin
            {out_de,out_hs,out_vs}<={in_de,in_hs,in_vs};
            {out_r,out_g,out_b}<=in_de?pixel:24'h000000;
        end
    end
    end else begin : diagnostic_video
        // Break the former row/character/font/pixel path at character selection.
        // Coordinate comparisons and color selection use separate existing
        // stages. Raw RGB, shape hits, colors, font cells and syncs stay aligned.
        reg [7:0] character_q;
        reg [2:0] glyph_x_q, glyph_y_q;
        reg panel_q, de_q, hs_q, vs_q;
        reg [23:0] raw_rgb_q, color_q, aim_color_q;
        reg [4:0] shapes_q; // arrows, aim, yellow target, green box, raw box
        reg [34:0] bitmap_q;
        reg [23:0] pixel_q;
        always @* begin
            bitmap_q=glyph(character_q);
            pixel_q=raw_rgb_q;
            if(shapes_q[0]) pixel_q=24'hff8000;
            if(shapes_q[1]) pixel_q=GREEN;
            if(shapes_q[2]) pixel_q=YELLOW;
            if(shapes_q[3]) pixel_q=aim_color_q;
            if(shapes_q[4]) pixel_q=YELLOW;
            if(panel_q) begin
                pixel_q=24'h081820;
                if(glyph_ink(bitmap_q,glyph_x_q,glyph_y_q))
                    pixel_q=color_q;
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                character_q<=8'h20;glyph_x_q<=0;glyph_y_q<=0;panel_q<=0;
                raw_rgb_q<=0;shapes_q<=0;aim_color_q<=0;color_q<=0;
                de_q<=0;hs_q<=0;vs_q<=0;
                {out_r,out_g,out_b}<=0;out_de<=0;out_hs<=0;out_vs<=0;
            end else begin
                character_q<=character;glyph_x_q<=glyph_x;glyph_y_q<=glyph_y;
                panel_q<=panel;raw_rgb_q<={in_r,in_g,in_b};color_q<=font_color;
                aim_color_q<=aim_color;
                shapes_q<={left_arrow || right_arrow || up_arrow || down_arrow,
                    aim_cross || aim_square,hold_box || ball_cross,box_pixel,raw_box_pixel};
                {de_q,hs_q,vs_q}<={in_de,in_hs,in_vs};
                {out_de,out_hs,out_vs}<={de_q,hs_q,vs_q};
                {out_r,out_g,out_b}<=de_q ? pixel_q : 24'h000000;
            end
        end
    end endgenerate
endmodule
