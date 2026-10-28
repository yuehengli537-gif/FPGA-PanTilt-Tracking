`timescale 1ns/1ps
// Bounded streaming red-ball candidates. All sufficiently long red runs are
// considered; CANDIDATES limits simultaneous regions, not regions per frame.
// No frame RAM. VS sampled at N publishes a completed measurement at N+12.
// Shape tests describe a 2-D near-circle, not proof of a physical 3-D sphere.
module red_ball_detector #(
    parameter integer WIDTH = 1280,
    parameter integer HEIGHT = 720,
    parameter integer MIN_RUN = 4,
    parameter integer MIN_DIAMETER = 12,
    parameter integer MAX_DIAMETER = 360,
    parameter integer MIN_PIXELS = 100,
    parameter integer RED_MIN = 80,
    parameter integer RED_MARGIN = 32,
    parameter integer JOIN_MARGIN = 8,
    parameter integer CANDIDATES = 4,
    parameter integer MAX_ROW_GAP = 1
)(
    input wire clk, rst_n,
    input wire [15:0] in_rgb565,
    input wire in_de, in_vs,
    input wire search_enable,
    input wire [11:0] search_left, search_top, search_right, search_bottom,
    output reg frame_update, target_valid,
    output reg [11:0] bbox_left, bbox_top, bbox_right, bbox_bottom,
    output reg [11:0] center_x, center_y,
    output reg [19:0] pixel_count,
    // Frame-latched diagnostic outputs; no feedback into target qualification.
    // flags: bit0 no run, 1 overflow, 2 ambiguous association, 3 size/count,
    // 4 aspect, 5 low fill, 6 high fill, 7 taper or projected-contour rejection.
    output reg [7:0] diag_flags,diag_candidate_flags,
    output reg diag_candidate_valid,
    output reg [12:0] diag_width,diag_height,diag_first,diag_last,diag_peak,
    output reg [19:0] diag_pixels,
    output reg [11:0] diag_left,diag_top,diag_right,diag_bottom
);
    // One to four live regions; empty tree leaves retain fixed N+12 latency.
    localparam integer GAP=MAX_ROW_GAP<0?0:(MAX_ROW_GAP>1?1:MAX_ROW_GAP);
    localparam [12:0] GAP_U=GAP,GAP_PLUS_ONE_U=GAP+1;
    localparam integer NC = CANDIDATES < 1 ? 1 : (CANDIDATES > 4 ? 4 : CANDIDATES);
    localparam integer SIDE_BITS = MAX_DIAMETER < 2 ? 1 : $clog2(MAX_DIAMETER+1);
    localparam [11:0] LAST_X = WIDTH-1, LAST_Y = HEIGHT-1;
    // MIN_RUN >= 4: ordinary runs close at least five clocks apart; WIDTH=4
    // continuous rows can close four clocks apart. The next association still
    // occurs after the preceding bank commit. No extra writeback stage is safe.
    localparam [11:0] MIN_RUN_U = MIN_RUN < 4 ? 4 : MIN_RUN;
    localparam [12:0] MIN_DIAMETER_U = MIN_DIAMETER, MAX_DIAMETER_U = MAX_DIAMETER;
    localparam [19:0] MIN_PIXELS_U = MIN_PIXELS;
    localparam [7:0] RED_MIN_U = RED_MIN;
    localparam [8:0] RED_MARGIN_U = RED_MARGIN;
    localparam [12:0] JOIN_MARGIN_U = JOIN_MARGIN;

    // V8: x/y bbox plus x+y, x-y, 2x+y and 2x-y support intervals.
    // Only retained-run endpoints are needed. Internal highlight holes do not
    // change an object's outer support, and interval union is exact when the
    // four candidate slots merge. HEIGHT-1 offsets negative y projections.
    // Narrow to the configured image, not the generic 12-bit pixel counters.
    localparam integer DIAG_BITS = WIDTH+HEIGHT>2 ? $clog2(WIDTH+HEIGHT-1) : 1;
    localparam integer OBLIQUE_BITS = 2*WIDTH+HEIGHT>3 ? $clog2(2*WIDTH+HEIGHT-2) : 1;
    localparam integer PROJECTION_BITS = 4*DIAG_BITS+4*OBLIQUE_BITS;
    localparam [DIAG_BITS-1:0] PROJECTION_Y_BIAS = HEIGHT-1;

    function [PROJECTION_BITS-1:0] run_projection;
        input [11:0] run_l,run_r,run_y;
        reg [DIAG_BITS-1:0] inverse_y,dp_l,dp_r,dm_l,dm_r;
        reg [OBLIQUE_BITS-1:0] op_l,op_r,om_l,om_r;
        begin
            inverse_y=PROJECTION_Y_BIAS-run_y;
            dp_l={1'b0,run_l}+{1'b0,run_y};dp_r={1'b0,run_r}+{1'b0,run_y};
            dm_l=run_l+inverse_y;dm_r=run_r+inverse_y;
            op_l=({2'b0,run_l}<<1)+run_y;op_r=({2'b0,run_r}<<1)+run_y;
            om_l=({2'b0,run_l}<<1)+inverse_y;om_r=({2'b0,run_r}<<1)+inverse_y;
            run_projection={dp_l,dp_r,dm_l,dm_r,op_l,op_r,om_l,om_r};
        end
    endfunction
    function [PROJECTION_BITS-1:0] merge_projection;
        input [PROJECTION_BITS-1:0] a,b;
        reg [DIAG_BITS-1:0] adpl,adpr,adml,admr,bdpl,bdpr,bdml,bdmr;
        reg [OBLIQUE_BITS-1:0] aopl,aopr,aoml,aomr,bopl,bopr,boml,bomr;
        begin
            {adpl,adpr,adml,admr,aopl,aopr,aoml,aomr}=a;
            {bdpl,bdpr,bdml,bdmr,bopl,bopr,boml,bomr}=b;
            merge_projection={adpl<bdpl?adpl:bdpl,adpr>bdpr?adpr:bdpr,
                adml<bdml?adml:bdml,admr>bdmr?admr:bdmr,
                aopl<bopl?aopl:bopl,aopr>bopr?aopr:bopr,
                aoml<boml?aoml:boml,aomr>bomr?aomr:bomr};
        end
    endfunction

    reg [15:0] rgb_q;
    reg de_q, vs_q, mask_red, mask_de, mask_vs;
    wire [7:0] red = {rgb_q[15:11],rgb_q[15:13]};
    wire [7:0] green = {rgb_q[10:5],rgb_q[10:9]};
    wire [7:0] blue = {rgb_q[4:0],rgb_q[4:2]};
    wire is_red = red>=RED_MIN_U &&
        {1'b0,red}>={1'b0,green}+RED_MARGIN_U &&
        {1'b0,red}>={1'b0,blue}+RED_MARGIN_U &&
        {1'b0,red}>={green,1'b0} && {1'b0,red}>={blue,1'b0};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rgb_q<=0;de_q<=0;vs_q<=0;mask_red<=0;mask_de<=0;mask_vs<=0;
        end else begin
            rgb_q<=in_rgb565;de_q<=in_de;vs_q<=in_vs;
            mask_red<=is_red;mask_de<=de_q;mask_vs<=vs_q;
        end
    end

    reg vs_d, frame_started, frame_complete;
    reg [11:0] x,y,run_left,run_length;
    reg roi_q;
    reg [11:0] roi_left,roi_top,roi_right,roi_bottom;
    reg [11:0] roi_left_prev,roi_top_prev;
    reg roi_left_zero,roi_top_zero,col_in_roi,row_in_roi;
    // Membership is computed one pixel/row ahead, using the same DE-driven
    // counters. Run closure sees only registered booleans, not four coordinate
    // comparisons. Bounds remain frozen at VS, including invalid/reversed ROIs.
    wire selected_red = mask_red && (!roi_q || (col_in_roi && row_in_roi));
    wire [11:0] tail_length = run_length+12'd1;
    reg run_ready,row_ready,frame_boundary,publish_frame;
    reg [11:0] desc_left,desc_right,desc_length,desc_y,row_y;
    reg [PROJECTION_BITS-1:0] desc_projection;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_d<=0;frame_started<=0;frame_complete<=0;
            x<=0;y<=0;run_left<=0;run_length<=0;
            roi_q<=0;roi_left<=0;roi_top<=0;roi_right<=0;roi_bottom<=0;
            roi_left_prev<=0;roi_top_prev<=0;roi_left_zero<=0;roi_top_zero<=0;
            col_in_roi<=0;row_in_roi<=0;
            run_ready<=0;row_ready<=0;frame_boundary<=0;publish_frame<=0;
            desc_left<=0;desc_right<=0;desc_length<=0;desc_y<=0;row_y<=0;
            desc_projection<=0;
        end else begin
            vs_d<=mask_vs;
            run_ready<=0;row_ready<=0;frame_boundary<=0;publish_frame<=0;
            if(mask_vs && !vs_d) begin
                roi_q<=search_enable;roi_left<=search_left;roi_top<=search_top;
                roi_right<=search_right;roi_bottom<=search_bottom;
                roi_left_prev<=search_left-12'd1;roi_top_prev<=search_top-12'd1;
                roi_left_zero<=search_left==0;roi_top_zero<=search_top==0;
                col_in_roi<=search_left==0;row_in_roi<=search_top==0;
                frame_boundary<=1;publish_frame<=frame_started && frame_complete;
                frame_started<=1;frame_complete<=0;
                x<=0;y<=0;run_left<=0;run_length<=0;
            end else if(mask_de && frame_started && !frame_complete) begin
                if(x==LAST_X) begin
                    row_ready<=1;row_y<=y;
                    col_in_roi<=roi_left_zero;
                    row_in_roi<=(roi_top_zero || y>=roi_top_prev) && y<roi_bottom;
                    if(selected_red && tail_length>=MIN_RUN_U) begin
                        run_ready<=1;desc_left<=run_length==0?x:run_left;
                        desc_right<=x;desc_length<=tail_length;desc_y<=y;
                        desc_projection<=run_projection(run_left,x,y);
                    end else if(!selected_red && run_length>=MIN_RUN_U) begin
                        run_ready<=1;desc_left<=run_left;
                        desc_right<=x-12'd1;desc_length<=run_length;desc_y<=y;
                        desc_projection<=run_projection(run_left,x-12'd1,y);
                    end
                    x<=0;run_left<=0;run_length<=0;
                    if(y==LAST_Y) frame_complete<=1;else y<=y+12'd1;
                end else begin
                    x<=x+12'd1;
                    col_in_roi<=(roi_left_zero || x>=roi_left_prev) && x<roi_right;
                    if(selected_red) begin
                        if(run_length==0) run_left<=x;
                        run_length<=tail_length;
                    end else begin
                        if(run_length>=MIN_RUN_U) begin
                            run_ready<=1;desc_left<=run_left;
                            desc_right<=x-12'd1;desc_length<=run_length;desc_y<=y;
                            desc_projection<=run_projection(run_left,x-12'd1,y);
                        end
                        run_length<=0;
                    end
                end
            end
        end
    end


    reg [NC-1:0] active,bad,previous_valid;
    reg [11:0] left_edge[0:NC-1],right_edge[0:NC-1],top_edge[0:NC-1],bottom_edge[0:NC-1];
    reg [11:0] row_left[0:NC-1],row_right[0:NC-1];
    reg [11:0] previous_left[0:NC-1],previous_right[0:NC-1],previous_y[0:NC-1];
    reg [11:0] first_left[0:NC-1],first_right[0:NC-1];
    reg [12:0] first_width[0:NC-1],peak_width[0:NC-1];
    reg [19:0] pixels[0:NC-1];
    reg [PROJECTION_BITS-1:0] projection[0:NC-1];
    reg capacity_overflow;
    wire [NC-1:0] candidate_matches;
    genvar g;
    generate for(g=0;g<NC;g=g+1) begin: candidate_association
        wire same_row=bottom_edge[g]==desc_y;
        wire next_row=bottom_edge[g]<desc_y && {1'b0,bottom_edge[g]}+GAP_PLUS_ONE_U>={1'b0,desc_y};
        wire overlaps_current={1'b0,desc_left}<={1'b0,row_right[g]}+JOIN_MARGIN_U &&
            {1'b0,row_left[g]}<={1'b0,desc_right}+JOIN_MARGIN_U;
        wire overlaps_previous={1'b0,desc_left}<={1'b0,previous_right[g]}+JOIN_MARGIN_U &&
            {1'b0,previous_left[g]}<={1'b0,desc_right}+JOIN_MARGIN_U;
        assign candidate_matches[g]=run_ready && active[g] && ((next_row && overlaps_current) ||
            (same_row && previous_valid[g] && overlaps_previous) || (same_row && overlaps_current));
    end endgenerate
    wire any_match=|candidate_matches;
    reg [NC-1:0] allocate;
    reg free_taken;
    integer scan,j,m;
    always @* begin
        allocate=0;free_taken=0;
        for(scan=0;scan<NC;scan=scan+1) begin
            if(!free_taken && (!active[scan] || (row_ready &&
                {1'b0,bottom_edge[scan]}+GAP_U<{1'b0,row_y}))) begin
                allocate[scan]=1;free_taken=1;
            end
        end
    end
    // Merge record: retained pixels; bbox L/R/top; first-row L/R; current-row
    // L/R; latest previous-row L/R/y; peak span; current/previous valid; bad.
    // Each actual red run contributes once. Gaps never count as red pixels.
    function [155:0] merge_records;
        input [155:0] a,b;
        reg [19:0] ap,bp;
        reg [11:0] al,ar,at,afl,afr,acl,acr,apl,apr,apy;
        reg [11:0] bl,br,bt,bfl,bfr,bcl,bcr,bpl,bpr,bpy;
        reg [12:0] ak,bk;
        reg acv,apv,ab,bcv,bpv,bb;
        reg [20:0] total;
        reg [11:0] l,r,t,fl,fr,cl,cr,pl,pr,py;
        reg [12:0] peak;
        begin
            {ap,al,ar,at,afl,afr,acl,acr,apl,apr,apy,ak,acv,apv,ab}=a;
            {bp,bl,br,bt,bfl,bfr,bcl,bcr,bpl,bpr,bpy,bk,bcv,bpv,bb}=b;
            total={1'b0,ap}+{1'b0,bp};
            l=al<bl?al:bl;r=ar>br?ar:br;t=at<bt?at:bt;
            if(at<bt) begin fl=afl;fr=afr;end
            else if(bt<at) begin fl=bfl;fr=bfr;end
            else begin fl=afl<bfl?afl:bfl;fr=afr>bfr?afr:bfr;end
            // Every selected record already contains the current run span.
            // Boundary union is idempotent; pixel counts are still old-only.
            cl=acl<bcl?acl:bcl;cr=acr>bcr?acr:bcr;
            if(!apv || (bpv && bpy>apy)) begin pl=bpl;pr=bpr;py=bpy;end
            else if(!bpv || apy>bpy) begin pl=apl;pr=apr;py=apy;end
            else begin pl=apl<bpl?apl:bpl;pr=apr>bpr?apr:bpr;py=apy;end
            peak=ak>bk?ak:bk;
            // Historical row unions older than the latest previous row are
            // unknown. max(old peaks, previous union, current union at commit)
            // is a lower bound; never substitute bbox width for peak span.
            merge_records={total[20]?20'hfffff:total[19:0],l,r,t,fl,fr,cl,cr,pl,pr,py,
                peak,(acv|bcv),(apv|bpv),(ab|bb)};
        end
    endfunction
    reg a_run,a_row,a_frame,a_publish;
    reg [11:0] a_left,a_right,a_length,a_y,a_row_y;
    reg [NC-1:0] a_matches,a_allocate;
    reg [155:0] a_record[0:3];
    reg [PROJECTION_BITS-1:0] a_projection[0:3];
    reg [PROJECTION_BITS-1:0] a_new_projection;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            a_run<=0;a_row<=0;a_frame<=0;a_publish<=0;
            a_left<=0;a_right<=0;a_length<=0;a_y<=0;a_row_y<=0;
            a_matches<=0;a_allocate<=0;
            a_new_projection<=0;
            for(m=0;m<4;m=m+1) begin a_record[m]<=0;a_projection[m]<=0;end
        end else begin
            a_run<=run_ready;a_row<=row_ready;a_frame<=frame_boundary;a_publish<=publish_frame;
            a_left<=desc_left;a_right<=desc_right;a_length<=desc_length;a_y<=desc_y;a_row_y<=row_y;
            a_matches<=candidate_matches;a_allocate<=allocate;
            a_new_projection<=desc_projection;
            for(m=0;m<NC;m=m+1) begin
                // The current run is included in every selected donor. Union
                // is idempotent; this adds no pixels and does not double-count.
                a_projection[m]<=merge_projection(projection[m],desc_projection);
                a_record[m]<={pixels[m],left_edge[m],right_edge[m],top_edge[m],
                    (top_edge[m]==desc_y && desc_left<first_left[m])?desc_left:first_left[m],
                    (top_edge[m]==desc_y && desc_right>first_right[m])?desc_right:first_right[m],
                    (bottom_edge[m]==desc_y && row_left[m]<desc_left)?row_left[m]:desc_left,
                    (bottom_edge[m]==desc_y && row_right[m]>desc_right)?row_right[m]:desc_right,
                    bottom_edge[m]==desc_y?previous_left[m]:row_left[m],
                    bottom_edge[m]==desc_y?previous_right[m]:row_right[m],
                    bottom_edge[m]==desc_y?previous_y[m]:bottom_edge[m],
                    peak_width[m],1'b1,
                    (bottom_edge[m]!=desc_y || previous_valid[m]),bad[m]};
            end
        end
    end
    reg b_run,b_row,b_frame,b_publish;
    reg [11:0] b_left,b_right,b_length,b_y,b_row_y;
    reg [NC-1:0] b_matches,b_allocate,b_winner;
    reg [1:0] b_valid;
    reg [155:0] b_record[0:1];
    reg [PROJECTION_BITS-1:0] b_projection[0:1];
    wire [3:0] padded_matches={{(4-NC){1'b0}},a_matches};
    wire [3:0] projection_matches=(|padded_matches)?padded_matches:4'b0001;
    reg [1:0] b_projection_valid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            b_run<=0;b_row<=0;b_frame<=0;b_publish<=0;
            b_left<=0;b_right<=0;b_length<=0;b_y<=0;b_row_y<=0;
            b_matches<=0;b_allocate<=0;b_winner<=0;
        end else begin
            b_run<=a_run;b_row<=a_row;b_frame<=a_frame;b_publish<=a_publish;
            b_left<=a_left;b_right<=a_right;b_length<=a_length;b_y<=a_y;b_row_y<=a_row_y;
            b_matches<=a_matches;b_allocate<=a_allocate;
            // Priority is evaluated after match registration. Provisional
            // allocation is used only when no match exists, so its free-slot
            // chain need not depend on the current descriptor's match logic.
            b_winner<=a_matches & (~a_matches+{{(NC-1){1'b0}},1'b1});
        end
    end
    generate for(g=0;g<2;g=g+1) begin: merge_pair
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin b_valid[g]<=0;b_record[g]<=0;b_projection[g]<=0;b_projection_valid[g]<=0;end
            else begin
                b_valid[g]<=|padded_matches[2*g +: 2];
                if(&padded_matches[2*g +: 2]) b_record[g]<=merge_records(a_record[2*g],a_record[2*g+1]);
                else b_record[g]<=padded_matches[2*g]?a_record[2*g]:a_record[2*g+1];
                b_projection_valid[g]<=|projection_matches[2*g +: 2];
                // V8c: choose allocation only after association is registered.
                // Do not fan the live candidate-match path into the wide
                // a_projection mux. The new run reuses leaf zero from here.
                if(g==0 && !(|padded_matches)) b_projection[g]<=a_new_projection;
                else if(&projection_matches[2*g +: 2]) b_projection[g]<=merge_projection(a_projection[2*g],a_projection[2*g+1]);
                else b_projection[g]<=projection_matches[2*g]?a_projection[2*g]:a_projection[2*g+1];
            end
        end
    end endgenerate
    reg commit_run,commit_row,commit_frame,commit_publish;
    reg [11:0] commit_left,commit_right,commit_length,commit_y,commit_row_y;
    reg [NC-1:0] commit_matches,commit_allocate,commit_winner;
    reg [155:0] commit_record;
    reg [PROJECTION_BITS-1:0] commit_projection;
    wire [155:0] merged_root_record=(&b_valid)?merge_records(b_record[0],b_record[1]):
        (b_valid[0]?b_record[0]:b_record[1]);
    wire [PROJECTION_BITS-1:0] merged_root_projection=(&b_projection_valid)?merge_projection(b_projection[0],b_projection[1]):
        (b_projection_valid[0]?b_projection[0]:b_projection[1]);
    reg [12:0] commit_current_width;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            commit_run<=0;commit_row<=0;commit_frame<=0;commit_publish<=0;
            commit_left<=0;commit_right<=0;commit_length<=0;commit_y<=0;commit_row_y<=0;
            commit_matches<=0;commit_allocate<=0;commit_winner<=0;commit_record<=0;commit_current_width<=0;
            commit_projection<=0;
        end else begin
            commit_run<=b_run;commit_row<=b_row;commit_frame<=b_frame;commit_publish<=b_publish;
            commit_left<=b_left;commit_right<=b_right;commit_length<=b_length;commit_y<=b_y;commit_row_y<=b_row_y;
            commit_matches<=b_matches;commit_allocate<=b_allocate;commit_winner<=b_winner;
            commit_record<=merged_root_record;
            commit_projection<=merged_root_projection;
            commit_current_width<={1'b0,merged_root_record[63:52]}-
                {1'b0,merged_root_record[75:64]}+13'd1;
        end
    end
    wire [19:0] merged_pixels;
    wire [11:0] ml,mr,mt,mfl,mfr,mcl,mcr,mpl,mpr,mpy;
    wire [12:0] merged_peak;
    wire mcv,mpv,mbad;
    assign {merged_pixels,ml,mr,mt,mfl,mfr,mcl,mcr,mpl,mpr,mpy,merged_peak,mcv,mpv,mbad}=commit_record;
    wire [11:0] current_left=mcl;
    wire [11:0] current_right=mcr;
    wire [12:0] current_width=commit_current_width;
    wire [11:0] top_left=mfl;
    wire [11:0] top_right=mfr;
    wire [12:0] previous_width={1'b0,mpr}-{1'b0,mpl}+13'd1;
    wire current_is_peak=current_width>=merged_peak && (!mpv || current_width>=previous_width);
    wire previous_is_peak=mpv && previous_width>=merged_peak;
    wire [20:0] current_pixels={1'b0,merged_pixels}+{9'b0,commit_length};
    wire [NC-1:0] actual_retiring;
    generate for(g=0;g<NC;g=g+1) begin: retire_after_merge
        // Evaluate against the latest bank after all earlier run commits, not
        // the stale bank at association time. Donors cannot publish twice.
        assign actual_retiring[g]=commit_row && active[g] &&
            {1'b0,bottom_edge[g]}+GAP_U<{1'b0,commit_row_y} &&
            !(commit_run && commit_matches[g]);
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            active<=0;bad<=0;previous_valid<=0;capacity_overflow<=0;
            for(j=0;j<NC;j=j+1) begin
                left_edge[j]<=0;right_edge[j]<=0;top_edge[j]<=0;bottom_edge[j]<=0;
                row_left[j]<=0;row_right[j]<=0;previous_left[j]<=0;previous_right[j]<=0;previous_y[j]<=0;
                first_left[j]<=0;first_right[j]<=0;first_width[j]<=0;peak_width[j]<=0;pixels[j]<=0;
                projection[j]<=0;
            end
        end else if(commit_frame) begin
            active<=0;bad<=0;previous_valid<=0;capacity_overflow<=0;
        end else begin
            for(j=0;j<NC;j=j+1) begin
                if(actual_retiring[j]) begin active[j]<=0;bad[j]<=0;previous_valid[j]<=0;end
                if(commit_run) begin
                    if(commit_matches[j] && !commit_winner[j]) begin
                        active[j]<=0;bad[j]<=0;previous_valid[j]<=0;
                    end else if(commit_winner[j]) begin
                        active[j]<=1;bad[j]<=mbad;
                        left_edge[j]<=ml<commit_left?ml:commit_left;
                        right_edge[j]<=mr>commit_right?mr:commit_right;
                        top_edge[j]<=mt;bottom_edge[j]<=commit_y;
                        row_left[j]<=current_left;row_right[j]<=current_right;
                        previous_left[j]<=mpl;previous_right[j]<=mpr;previous_y[j]<=mpy;previous_valid[j]<=mpv;
                        first_left[j]<=top_left;first_right[j]<=top_right;
                        first_width[j]<={1'b0,top_right}-{1'b0,top_left}+13'd1;
                        peak_width[j]<=current_is_peak?current_width:(previous_is_peak?previous_width:merged_peak);
                        pixels[j]<=current_pixels[20]?20'hfffff:current_pixels[19:0];
                        projection[j]<=commit_projection;
                    end else if(!( |commit_matches ) && commit_allocate[j]) begin
                        active[j]<=1;bad[j]<=0;previous_valid[j]<=0;
                        left_edge[j]<=commit_left;right_edge[j]<=commit_right;top_edge[j]<=commit_y;bottom_edge[j]<=commit_y;
                        row_left[j]<=commit_left;row_right[j]<=commit_right;previous_left[j]<=0;previous_right[j]<=0;previous_y[j]<=0;
                        first_left[j]<=commit_left;first_right[j]<=commit_right;
                        first_width[j]<={1'b0,commit_length};peak_width[j]<={1'b0,commit_length};pixels[j]<={8'b0,commit_length};
                        projection[j]<=commit_projection;
                    end
                end
            end
            if(commit_run && !(|commit_matches) && !(|commit_allocate)) capacity_overflow<=1;
        end
    end
    // Retiring slots are snapshotted before they are reused. All four can finish
    // in one cycle. A frame-end token takes the exact same pipeline as candidates.
    wire [3:0] diag_leaf_valid;
    wire [7:0] diag_leaf_flags[0:3];
    wire [64:0] diag_leaf_geometry[0:3]; // width,height,first,last,peak
    wire [3:0] leaf_valid;
    wire [11:0] leaf_left[0:3],leaf_top[0:3],leaf_right[0:3],leaf_bottom[0:3];
    wire [11:0] leaf_x[0:3],leaf_y[0:3];
    wire [19:0] leaf_pixels[0:3];
    generate for(g=0;g<4;g=g+1) begin: shape_pipeline
        if(g<NC) begin: used
            reg s0_valid,s1_valid,s2_valid,s3_valid;
            reg diag_v0,diag_v1,diag_v2,diag_v3,diag_bad0;
            reg [7:0] diag_f1,diag_f2,diag_f3;
            reg diag_product_ok1,diag_product_ok2;
            reg [64:0] diag_geometry1,diag_geometry2,diag_geometry3;
            reg [12:0] s0_width,s0_height,s0_first,s0_last,s0_peak;
            wire [DIAG_BITS-1:0] dp_min,dp_max,dm_min,dm_max;
            wire [OBLIQUE_BITS-1:0] op_min,op_max,om_min,om_max;
            assign {dp_min,dp_max,dm_min,dm_max,op_min,op_max,om_min,om_max}=projection[g];
            // Inclusive support spans have an extra bit for a full interval.
            reg [DIAG_BITS:0] s0_dp,s0_dm,s1_dp,s1_dm,s2_dp,s2_dm;
            reg [OBLIQUE_BITS:0] s0_op,s0_om,s1_op,s1_om,s2_op,s2_om;
            reg [DIAG_BITS:0] diag_den1;
            reg [OBLIQUE_BITS:0] oblique_den1;
            reg [DIAG_BITS+4:0] diag_min2,diag_max2;
            reg [OBLIQUE_BITS+4:0] oblique_min2,oblique_max2;
            wire [18:0] diag_den0={6'b0,s0_width}+{6'b0,s0_height};
            wire [18:0] oblique_den0=({6'b0,s0_width}<<1)+{6'b0,s0_height};
            wire [18:0] dp_wide2=s2_dp,dm_wide2=s2_dm,op_wide2=s2_op,om_wide2=s2_om;
            // For a circle: D/(W+H)~=0.707; O/(2W+H)~=0.745.
            // Bounds D: 5/8..13/16; O: 11/16..13/16, with one
            // projection-pixel tolerance at each boundary. The 2x directions
            // remove the rotated-square blind spot of diagonal-only checks.
            // This is a sampled near-circle test, not a 3-D sphere classifier;
            // tiny/strongly rounded polygons can still resemble a circle.
            wire contour_ok2=((dp_wide2+19'd1)<<3)>=diag_min2 &&
                ((dm_wide2+19'd1)<<3)>=diag_min2 &&
                (dp_wide2<<4)<=diag_max2 && (dm_wide2<<4)<=diag_max2 &&
                ((op_wide2+19'd1)<<4)>=oblique_min2 &&
                ((om_wide2+19'd1)<<4)>=oblique_min2 &&
                (op_wide2<<4)<=oblique_max2 && (om_wide2<<4)<=oblique_max2;
            reg [11:0] l0,t0,r0,b0,l1,t1,r1,b1,l2,t2,r2,b2,l3,t3,r3,b3;
            reg [19:0] p0,p1,p2,p3;
            reg [12:0] x1,y1,x2,y2;
            reg [11:0] x3,y3;
            reg [2*SIDE_BITS-1:0] area1;
            reg guard1,guard2;
            reg [23:0] minimum2,maximum2,retained2;
            wire [23:0] area1_wide=area1;
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    s0_valid<=0;s1_valid<=0;s2_valid<=0;s3_valid<=0;
                    s0_width<=0;s0_height<=0;s0_first<=0;s0_last<=0;s0_peak<=0;
                    s0_dp<=0;s0_dm<=0;s0_op<=0;s0_om<=0;s1_dp<=0;s1_dm<=0;s1_op<=0;s1_om<=0;
                    s2_dp<=0;s2_dm<=0;s2_op<=0;s2_om<=0;diag_den1<=0;oblique_den1<=0;
                    diag_min2<=0;diag_max2<=0;oblique_min2<=0;oblique_max2<=0;
                    l0<=0;t0<=0;r0<=0;b0<=0;l1<=0;t1<=0;r1<=0;b1<=0;
                    l2<=0;t2<=0;r2<=0;b2<=0;l3<=0;t3<=0;r3<=0;b3<=0;
                    p0<=0;p1<=0;p2<=0;p3<=0;x1<=0;y1<=0;x2<=0;y2<=0;x3<=0;y3<=0;
                    area1<=0;guard1<=0;guard2<=0;minimum2<=0;maximum2<=0;retained2<=0;
                end else begin
                    // N+6: complete candidate geometry is independent of all
                    // later changes to the live association bank.
                    s0_valid<=active[g] && !bad[g] && (actual_retiring[g] || commit_frame);
                    s0_width<={1'b0,right_edge[g]}-{1'b0,left_edge[g]}+13'd1;
                    s0_height<={1'b0,bottom_edge[g]}-{1'b0,top_edge[g]}+13'd1;
                    s0_first<=first_width[g];s0_last<={1'b0,row_right[g]}-{1'b0,row_left[g]}+13'd1;
                    s0_peak<=peak_width[g];p0<=pixels[g];
                    s0_dp<={1'b0,dp_max}-{1'b0,dp_min}+1'b1;
                    s0_dm<={1'b0,dm_max}-{1'b0,dm_min}+1'b1;
                    s0_op<={1'b0,op_max}-{1'b0,op_min}+1'b1;
                    s0_om<={1'b0,om_max}-{1'b0,om_min}+1'b1;
                    l0<=left_edge[g];t0<=top_edge[g];r0<=right_edge[g];b0<=bottom_edge[g];
                    // N+7: bounded product and independent size/taper guards.
                    // Full-width guards prevent oversized objects aliasing into
                    // the narrowed multiplier. Holes never add retained pixels.
                    s1_valid<=s0_valid;
                    s1_dp<=s0_dp;s1_dm<=s0_dm;s1_op<=s0_op;s1_om<=s0_om;
                    // V8b separates denominator addition, constant scaling,
                    // and comparisons into the existing N+7/N+8/N+9 stages.
                    diag_den1<=diag_den0;oblique_den1<=oblique_den0;
                    area1<=s0_width[SIDE_BITS-1:0]*s0_height[SIDE_BITS-1:0];
                    guard1<=s0_width>=MIN_DIAMETER_U && s0_height>=MIN_DIAMETER_U &&
                        s0_width<=MAX_DIAMETER_U && s0_height<=MAX_DIAMETER_U && p0>=MIN_PIXELS_U &&
                        ({2'b0,s0_width}<<2)>=(({2'b0,s0_height}<<1)+s0_height) &&
                        ({2'b0,s0_height}<<2)>=(({2'b0,s0_width}<<1)+s0_width) &&
                        ({2'b0,s0_first}<<2)<=(({2'b0,s0_peak}<<1)+s0_peak) &&
                        ({2'b0,s0_last}<<2)<=(({2'b0,s0_peak}<<1)+s0_peak);
                    l1<=l0;t1<=t0;r1<=r0;b1<=b0;p1<=p0;
                    x1<={1'b0,l0}+{1'b0,r0};y1<={1'b0,t0}+{1'b0,b0};
                    // N+8: 9/16 <= retained-red area / bounding area <= 14/16.
                    s2_valid<=s1_valid;guard2<=guard1;
                    s2_dp<=s1_dp;s2_dm<=s1_dm;s2_op<=s1_op;s2_om<=s1_om;
                    diag_min2<=(diag_den1<<2)+diag_den1;
                    diag_max2<=(diag_den1<<3)+(diag_den1<<2)+diag_den1+5'd16;
                    oblique_min2<=(oblique_den1<<3)+(oblique_den1<<1)+oblique_den1;
                    oblique_max2<=(oblique_den1<<3)+(oblique_den1<<2)+oblique_den1+5'd16;
                    minimum2<=(area1_wide<<3)+area1_wide;
                    maximum2<=(area1_wide<<3)+(area1_wide<<2)+(area1_wide<<1);
                    retained2<={p1,4'b0};
                    l2<=l1;t2<=t1;r2<=r1;b2<=b1;p2<=p1;x2<=x1;y2<=y1;
                    // N+9: binary shape qualification; choose largest qualified
                    // region later, never simply the largest red region.
                    s3_valid<=s2_valid && guard2 && contour_ok2 && retained2>=minimum2 && retained2<=maximum2;
                    l3<=l2;t3<=t2;r3<=r2;b3<=b2;p3<=p2;x3<=x2[12:1];y3<=y2[12:1];
                end
            end
            // Parallel observability pipeline uses the exact geometry already
            // snapshotted for qualification. Bad candidates remain observable.
            // Oversize boxes suppress fill flags because area1 is narrowed.
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    diag_v0<=0;diag_v1<=0;diag_v2<=0;diag_v3<=0;diag_bad0<=0;
                    diag_f1<=0;diag_f2<=0;diag_f3<=0;
                    diag_product_ok1<=0;diag_product_ok2<=0;
                    diag_geometry1<=0;diag_geometry2<=0;diag_geometry3<=0;
                end else begin
                    diag_v0<=active[g] && (actual_retiring[g] || commit_frame);
                    diag_bad0<=bad[g];
                    diag_v1<=diag_v0;
                    diag_f1<=0;
                    diag_f1[2]<=diag_bad0;
                    diag_f1[3]<=!(s0_width>=MIN_DIAMETER_U && s0_height>=MIN_DIAMETER_U &&
                        s0_width<=MAX_DIAMETER_U && s0_height<=MAX_DIAMETER_U && p0>=MIN_PIXELS_U);
                    diag_f1[4]<=!(({2'b0,s0_width}<<2)>=(({2'b0,s0_height}<<1)+s0_height) &&
                        ({2'b0,s0_height}<<2)>=(({2'b0,s0_width}<<1)+s0_width));
                    diag_f1[7]<=!(({2'b0,s0_first}<<2)<=(({2'b0,s0_peak}<<1)+s0_peak) &&
                        ({2'b0,s0_last}<<2)<=(({2'b0,s0_peak}<<1)+s0_peak));
                    diag_product_ok1<=s0_width<=MAX_DIAMETER_U && s0_height<=MAX_DIAMETER_U;
                    diag_geometry1<={s0_width,s0_height,s0_first,s0_last,s0_peak};
                    diag_v2<=diag_v1;diag_f2<=diag_f1;diag_product_ok2<=diag_product_ok1;
                    diag_geometry2<=diag_geometry1;
                    diag_v3<=diag_v2;diag_f3<=diag_f2;
                    diag_f3[7]<=diag_f2[7] || !contour_ok2;
                    diag_f3[5]<=diag_product_ok2 && retained2<minimum2;
                    diag_f3[6]<=diag_product_ok2 && retained2>maximum2;
                    diag_geometry3<=diag_geometry2;
                end
            end
            assign diag_leaf_valid[g]=diag_v3;
            assign diag_leaf_flags[g]=diag_f3;
            assign diag_leaf_geometry[g]=diag_geometry3;
            assign leaf_valid[g]=s3_valid;
            assign leaf_left[g]=l3;assign leaf_top[g]=t3;assign leaf_right[g]=r3;assign leaf_bottom[g]=b3;
            assign leaf_pixels[g]=p3;assign leaf_x[g]=x3;assign leaf_y[g]=y3;
        end else begin: unused
            assign diag_leaf_valid[g]=0;
            assign diag_leaf_flags[g]=0;
            assign diag_leaf_geometry[g]=0;
            assign leaf_valid[g]=0;
            assign leaf_left[g]=0;assign leaf_top[g]=0;assign leaf_right[g]=0;assign leaf_bottom[g]=0;
            assign leaf_pixels[g]=0;assign leaf_x[g]=0;assign leaf_y[g]=0;
        end
    end endgenerate

    // Two registered comparison levels, rather than a four-object priority
    // chain behind multiplication. Coordinate sums have already been pipelined.
    reg [1:0] pair_valid;
    reg [11:0] pair_left[0:1],pair_top[0:1],pair_right[0:1],pair_bottom[0:1],pair_x[0:1],pair_y[0:1];
    reg [19:0] pair_pixels[0:1];
    generate for(g=0;g<2;g=g+1) begin: select_pair
        wire pick_left=leaf_valid[2*g] && (!leaf_valid[2*g+1] || leaf_pixels[2*g]>=leaf_pixels[2*g+1]);
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                pair_valid[g]<=0;pair_left[g]<=0;pair_top[g]<=0;pair_right[g]<=0;pair_bottom[g]<=0;
                pair_pixels[g]<=0;pair_x[g]<=0;pair_y[g]<=0;
            end else begin
                pair_valid[g]<=leaf_valid[2*g] || leaf_valid[2*g+1];
                pair_left[g]<=pick_left?leaf_left[2*g]:leaf_left[2*g+1];
                pair_top[g]<=pick_left?leaf_top[2*g]:leaf_top[2*g+1];
                pair_right[g]<=pick_left?leaf_right[2*g]:leaf_right[2*g+1];
                pair_bottom[g]<=pick_left?leaf_bottom[2*g]:leaf_bottom[2*g+1];
                pair_pixels[g]<=pick_left?leaf_pixels[2*g]:leaf_pixels[2*g+1];
                pair_x[g]<=pick_left?leaf_x[2*g]:leaf_x[2*g+1];
                pair_y[g]<=pick_left?leaf_y[2*g]:leaf_y[2*g+1];
            end
        end
    end endgenerate
    wire pick_pair0=pair_valid[0] && (!pair_valid[1] || pair_pixels[0]>=pair_pixels[1]);
    reg root_valid;
    reg [11:0] root_left,root_top,root_right,root_bottom,root_x,root_y;
    reg [19:0] root_pixels;
    reg [5:0] end_pipe,publish_pipe,overflow_pipe;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            root_valid<=0;root_left<=0;root_top<=0;root_right<=0;root_bottom<=0;
            root_pixels<=0;root_x<=0;root_y<=0;end_pipe<=0;publish_pipe<=0;overflow_pipe<=0;
        end else begin
            root_valid<=|pair_valid;
            root_left<=pick_pair0?pair_left[0]:pair_left[1];root_top<=pick_pair0?pair_top[0]:pair_top[1];
            root_right<=pick_pair0?pair_right[0]:pair_right[1];root_bottom<=pick_pair0?pair_bottom[0]:pair_bottom[1];
            root_pixels<=pick_pair0?pair_pixels[0]:pair_pixels[1];
            root_x<=pick_pair0?pair_x[0]:pair_x[1];root_y<=pick_pair0?pair_y[0]:pair_y[1];
            end_pipe<={end_pipe[4:0],commit_frame};
            publish_pipe<={publish_pipe[4:0],commit_publish};
            overflow_pipe<={overflow_pipe[4:0],capacity_overflow};
        end
    end
    reg best_valid;
    reg [11:0] best_left,best_top,best_right,best_bottom,best_x,best_y;
    reg [19:0] best_pixels;
    wire root_wins=root_valid && (!best_valid || root_pixels>best_pixels);
    wire chosen_valid=root_valid || best_valid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            best_valid<=0;best_left<=0;best_top<=0;best_right<=0;best_bottom<=0;
            best_pixels<=0;best_x<=0;best_y<=0;
            frame_update<=0;target_valid<=0;
            bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;
            center_x<=0;center_y<=0;pixel_count<=0;
        end else begin
            frame_update<=0;
            if(end_pipe[5]) begin
                // N+12: the frame-end candidate and all earlier retired regions
                // are now complete. Clear history only after this publication.
                if(publish_pipe[5]) begin
                    frame_update<=1;target_valid<=chosen_valid && !overflow_pipe[5];
                    if(chosen_valid && !overflow_pipe[5]) begin
                        bbox_left<=root_wins?root_left:best_left;bbox_top<=root_wins?root_top:best_top;
                        bbox_right<=root_wins?root_right:best_right;bbox_bottom<=root_wins?root_bottom:best_bottom;
                        center_x<=root_wins?root_x:best_x;center_y<=root_wins?root_y:best_y;
                        pixel_count<=root_wins?root_pixels:best_pixels;
                    end else begin
                        bbox_left<=0;bbox_top<=0;bbox_right<=0;bbox_bottom<=0;
                        center_x<=0;center_y<=0;pixel_count<=0;
                    end
                end
                best_valid<=0;best_pixels<=0;
            end else if(root_wins) begin
                best_valid<=1;best_left<=root_left;best_top<=root_top;best_right<=root_right;best_bottom<=root_bottom;
                best_pixels<=root_pixels;best_x<=root_x;best_y<=root_y;
            end
        end
    end
    // Largest RAW candidate and OR-of-all-rejections share the original frame
    // token but cannot alter qualification, target metadata, or servo control.
    wire [140:0] diag_leaf_record[0:3];
    reg [1:0] diag_pair_valid;
    reg [140:0] diag_pair_record[0:1];
    reg [7:0] diag_pair_or[0:1];
    generate for(g=0;g<4;g=g+1) begin: diagnostic_leaf_record
        assign diag_leaf_record[g]={diag_leaf_flags[g],diag_leaf_geometry[g],leaf_pixels[g],
            leaf_left[g],leaf_top[g],leaf_right[g],leaf_bottom[g]};
    end endgenerate
    generate for(g=0;g<2;g=g+1) begin: diagnostic_select_pair
        wire choose_left=diag_leaf_valid[2*g] && (!diag_leaf_valid[2*g+1] ||
            leaf_pixels[2*g]>=leaf_pixels[2*g+1]);
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin diag_pair_valid[g]<=0;diag_pair_record[g]<=0;diag_pair_or[g]<=0;end
            else begin
                diag_pair_valid[g]<=diag_leaf_valid[2*g] || diag_leaf_valid[2*g+1];
                diag_pair_record[g]<=choose_left?diag_leaf_record[2*g]:diag_leaf_record[2*g+1];
                diag_pair_or[g]<=(diag_leaf_valid[2*g]?diag_leaf_flags[2*g]:8'd0) |
                    (diag_leaf_valid[2*g+1]?diag_leaf_flags[2*g+1]:8'd0);
            end
        end
    end endgenerate
    wire diag_choose_pair0=diag_pair_valid[0] && (!diag_pair_valid[1] ||
        diag_pair_record[0][67:48]>=diag_pair_record[1][67:48]);
    reg diag_root_valid,diag_best_valid;
    reg [140:0] diag_root_record,diag_best_record;
    reg [7:0] diag_root_or,diag_best_or;
    reg diag_run_seen;
    reg [5:0] diag_run_pipe;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_root_valid<=0;diag_root_record<=0;diag_root_or<=0;
            diag_run_seen<=0;diag_run_pipe<=0;
        end else begin
            diag_root_valid<=|diag_pair_valid;
            diag_root_record<=diag_choose_pair0?diag_pair_record[0]:diag_pair_record[1];
            diag_root_or<=diag_pair_or[0]|diag_pair_or[1];
            if(commit_frame) diag_run_seen<=0;
            else if(commit_run) diag_run_seen<=1;
            diag_run_pipe<={diag_run_pipe[4:0],diag_run_seen};
        end
    end
    wire diag_root_wins=diag_root_valid && (!diag_best_valid ||
        diag_root_record[67:48]>diag_best_record[67:48]);
    wire diag_any_candidate=diag_root_valid || diag_best_valid;
    wire [140:0] diag_chosen_record=diag_root_wins?diag_root_record:diag_best_record;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            diag_best_valid<=0;diag_best_record<=0;diag_best_or<=0;
            diag_flags<=0;diag_candidate_flags<=0;diag_candidate_valid<=0;
            diag_width<=0;diag_height<=0;diag_first<=0;diag_last<=0;diag_peak<=0;
            diag_pixels<=0;diag_left<=0;diag_top<=0;diag_right<=0;diag_bottom<=0;
        end else if(end_pipe[5]) begin
            if(publish_pipe[5]) begin
                diag_flags<=diag_best_or|diag_root_or|{6'b0,overflow_pipe[5],!diag_run_pipe[5]};
                diag_candidate_valid<=diag_any_candidate;
                {diag_candidate_flags,diag_width,diag_height,diag_first,diag_last,diag_peak,
                    diag_pixels,diag_left,diag_top,diag_right,diag_bottom}<=
                    diag_any_candidate?diag_chosen_record:141'd0;
            end
            diag_best_valid<=0;diag_best_record<=0;diag_best_or<=0;
        end else begin
            diag_best_or<=diag_best_or|diag_root_or;
            if(diag_root_wins) begin diag_best_valid<=1;diag_best_record<=diag_root_record;end
        end
    end

endmodule
