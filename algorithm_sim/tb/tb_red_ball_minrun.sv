`timescale 1ns/1ps
module tb_red_ball_minrun;
 localparam integer W=4,H=20,LAT=12;
 reg clk=0,rst_n=0,de=0,vs=0;
 always #5 clk=~clk;
 reg [15:0] rgb=0;
 wire [1:0] update,valid,dc;
 wire [7:0] flags[0:1],cflags[0:1];
 wire [12:0] dw[0:1],dh[0:1],df[0:1],dl[0:1],dk[0:1];
 wire [19:0] dp[0:1],pc[0:1];
 wire [11:0] l[0:1],t[0:1],r[0:1],b[0:1],cx[0:1],cy[0:1];
 wire [195:0] result[0:1];
 genvar g;
 generate for(g=0;g<2;g=g+1)begin: cases
  red_ball_detector #(.WIDTH(W),.HEIGHT(H),.MIN_RUN(g==0?1:4)) dut(
   .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_vs(vs),
   .search_enable(1'b0),.search_left(12'd0),.search_top(12'd0),.search_right(12'd0),.search_bottom(12'd0),
   .frame_update(update[g]),.target_valid(valid[g]),.pixel_count(pc[g]),
   .bbox_left(l[g]),.bbox_top(t[g]),.bbox_right(r[g]),.bbox_bottom(b[g]),.center_x(cx[g]),.center_y(cy[g]),
   .diag_flags(flags[g]),.diag_candidate_flags(cflags[g]),.diag_candidate_valid(dc[g]),
   .diag_width(dw[g]),.diag_height(dh[g]),.diag_first(df[g]),.diag_last(dl[g]),.diag_peak(dk[g]),.diag_pixels(dp[g]));
  assign result[g]={update[g],valid[g],dc[g],flags[g],cflags[g],dw[g],dh[g],df[g],dl[g],dk[g],dp[g],pc[g],l[g],t[g],r[g],b[g],cx[g],cy[g]};
 end endgenerate
 integer x,y,n,frames=0;
 task tick(input [15:0] p,input bit pd,pv);
  begin
   @(negedge clk);rgb=p;de=pd;vs=pv;@(posedge clk);#1;
   if(result[0]!==result[1])$fatal(1,"MIN_RUN=1 does not clamp to4");
  end
 endtask
 task boundary(input bit expected);
  begin
   for(n=0;n<=LAT;n=n+1)begin
    tick(0,0,n<2);
    if(update!==((n==LAT && expected)?2'b11:2'b00))$fatal(1,"WIDTH4 exact N+12 publication failed at%0d",n);
   end
   tick(0,0,0);if(update)$fatal(1,"extra publication");
  end
 endtask
 task draw(input bit short_runs);
  begin
   for(y=0;y<H;y=y+1)for(x=0;x<W;x=x+1)begin
    tick(short_runs && x==W-1 ? 16'h8410:16'hf800,1,0);
    if(update)$fatal(1,"active-frame publication");
   end
   // Last red input pixel is immediately followed by raw VS in full-run mode.
   boundary(1);frames=frames+1;
   if(valid || pc[0]!=0)$fatal(1,"WIDTH4 object cannot be a ball");
   if(short_runs)begin
    if(dc || dp[0]!=0 || !flags[0][0])$fatal(1,"three-pixel fragments survived MIN_RUN4");
   end else begin
    if(dc!==2'b11 || dp[0]!=W*H || dw[0]!=W || dh[0]!=H || df[0]!=W || dl[0]!=W || dk[0]!=W)
     $fatal(1,"WIDTH4 pending commits lost/doubled data: pixels%0d width%0d height%0d first%0d last%0d peak%0d",dp[0],dw[0],dh[0],df[0],dl[0],dk[0]);
   end
  end
 endtask
 initial begin
  repeat(3)tick(0,0,0);rst_n=1;boundary(0);
  draw(0);draw(1);draw(0);
  $display("PASS red_ball_minrun: WIDTH4 continuous rows, final red pixel adjacent VS, exact N+12, 80 pixels once, MIN_RUN1 clamped4, three-pixel runs rejected; frames=%0d",frames);
  $finish;
 end
 initial begin #1000000;$fatal(1,"minrun timeout");end
endmodule
