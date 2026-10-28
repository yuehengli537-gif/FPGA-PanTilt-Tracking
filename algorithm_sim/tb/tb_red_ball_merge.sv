`timescale 1ns/1ps
// Independent raster fixtures for union of initially separate red lobes.
// A white notch reaches the sphere's top, unlike a hole inside an already
// connected component. Exact area is counted once per retained red pixel.
module tb_red_ball_merge;
 parameter integer WIDTH=240,HEIGHT=180,SMOKE=0,CANDIDATES=4;
 localparam integer CX=WIDTH/2,CY=HEIGHT/2,LATENCY=12;
 reg clk=0,rst_n=0,de=0,vs=0;
 always #5 clk=~clk;
 reg [15:0] rgb=0;
 wire update,valid;
 wire [11:0] left,top,right,bottom,cx,cy;
 wire [19:0] count;
 red_ball_detector #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MAX_DIAMETER(220),.CANDIDATES(CANDIDATES)) dut(
  .clk(clk),.rst_n(rst_n),.in_rgb565(rgb),.in_de(de),.in_vs(vs),
  .search_enable(1'b0),.search_left(12'd0),.search_top(12'd0),
  .search_right(12'd0),.search_bottom(12'd0),.frame_update(update),
  .target_valid(valid),.bbox_left(left),.bbox_top(top),.bbox_right(right),
  .bbox_bottom(bottom),.center_x(cx),.center_y(cy),.pixel_count(count));
 wire [92:0] published={valid,left,top,right,bottom,cx,cy,count};
 reg [92:0] frozen;
 integer trace,checks=0,frames=0,current_scene=-1,x,y,k;
 integer total,l,t,r,b,run_len,run_start,end_x;
 reg [15:0] p;
 function integer disk(input integer px,py,bx,by,rad);
  disk=(px-bx)*(px-bx)+(py-by)*(py-by)<=rad*rad;
 endfunction
 function integer rect(input integer px,py,x0,y0,x1,y1);
  rect=px>=x0 && px<=x1 && py>=y0 && py<=y1;
 endfunction
 function [15:0] pixel(input integer scene,px,py);
  begin
   pixel=16'h8410;
   case(scene)
    0,1,3,4,5,6,10,11,13: if(disk(px,py,CX,CY,40)) pixel=16'hf800;
    2: if(disk(px,py,CX,CY,50))pixel=16'hf800;
    7: if(rect(px,py,CX-40,CY-40,CX+39,CY+39))pixel=16'hf800;
    8: if(disk(px,py,CX,CY,40) && !disk(px,py,CX,CY,32))pixel=16'hf800;
    9: if(rect(px,py,CX-80,CY-10,CX+79,CY+9))pixel=16'hf800;
    12: if(disk(px,py,CX-50,CY,25) || disk(px,py,CX+50,CY,25) ||
           rect(px,py,CX-25,CY,CX+25,CY+1))pixel=16'hf800;
    14: if(disk(px,py,WIDTH-41,HEIGHT-41,40))pixel=16'hf800;
   endcase
   if((scene==1 || scene==6 || scene==7 || scene==8) && px>=CX-4 && px<=CX+5 && py<=CY-14)
    pixel=16'hffff;
   if(scene==2 && px>=CX-4 && px<=CX+5 && py<=CY-17)pixel=16'hffff;
   if(scene==3 && ((px>=CX-14 && px<=CX-5)||(px>=CX+6 && px<=CX+15)) && py<=CY-14)
    pixel=16'hffff;
   if((scene==4 || scene==6) && py==CY)pixel=16'h8410;
   if(scene==5 && (py==CY || py==CY+1))pixel=16'h8410;
   if(scene==9 && px>=CX-4 && px<=CX+5 && py<=CY)pixel=16'hffff;
   if(scene==10 && rect(px,py,CX+37,CY-4,CX+44,CY+4))pixel=16'hf800;
   if(scene==11 && rect(px,py,CX-4,CY-85,CX+3,CY-20))pixel=16'hf800;
   if(scene==13 && px<CX)pixel=16'h8410;
  end
 endfunction
 task tick(input [15:0] value,input bit active,sync);
  begin @(negedge clk);rgb=value;de=active;vs=sync;@(posedge clk);#1;end
 endtask
 task boundary(input bit expected);
  integer stage;
  begin
   for(stage=0;stage<=LATENCY;stage=stage+1)begin
    tick(0,0,stage<2);
    if(update!==((stage==LATENCY)?expected:1'b0))
     $fatal(1,"merge scene%0d publication stage%0d expected N+%0d",current_scene,stage,LATENCY);
   end
   tick(0,0,0);if(update)$fatal(1,"donor caused extra publication pulse");
   checks=checks+1;frozen=published;
  end
 endtask
 task draw(input integer scene,input bit expected);
  begin
   current_scene=scene;total=0;l=WIDTH;t=HEIGHT;r=0;b=0;
   for(y=0;y<HEIGHT;y=y+1)begin
    run_len=0;run_start=0;
    for(x=0;x<WIDTH;x=x+1)begin
     p=pixel(scene,x,y);
     // The reference works on the complete bitmap, without candidate slots,
     // matches, merge priorities, geometry scores, or RTL bank state.
     if(p==16'hf800)begin if(run_len==0)run_start=x;run_len=run_len+1;end
     if(p!=16'hf800 || x==WIDTH-1)begin
      end_x=(p==16'hf800)?x:x-1;
      if(run_len>=4)begin
       total=total+run_len;
       if(run_start<l)l=run_start;if(end_x>r)r=end_x;
       if(y<t)t=y;if(y>b)b=y;
      end
      run_len=0;
     end
     tick(p,1,0);
     if(update || published!==frozen)$fatal(1,"donor or active-frame metadata publication scene%0d",scene);
     // No row blanking: covers pending run commits and first-pixel next rows.
    end
   end
   boundary(1);frames=frames+1;
   if(expected)begin
    if(!valid || left!=l || right!=r || top!=t || bottom!=b ||
       cx!=(l+r)/2 || cy!=(t+b)/2 || count!=total)
     $fatal(1,"merge scene%0d expected bbox%0d,%0d..%0d,%0d area%0d got valid%0d bbox%0d,%0d..%0d,%0d center%0d,%0d area%0d",scene,l,t,r,b,total,valid,left,top,right,bottom,cx,cy,count);
   end else if(published!==0)$fatal(1,"merge scene%0d non-ball must be invalid got%h",scene,published);
   checks=checks+1;
   $fdisplay(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",scene,expected,valid,left,top,right,bottom,count,total,frames);
   $fflush(trace);
   $display("SCENE merge%0d pass expected%0d area%0d",scene,expected,count);
  end
 endtask
 initial begin
  trace=$fopen("merge_trace.csv","w");if(!trace)$fatal(1,"cannot open merge trace");
  $fdisplay(trace,"scene,expected,valid,left,top,right,bottom,area,reference_area,frame");
  repeat(3)tick(0,0,0);rst_n=1;boundary(0);
  if(SMOKE)begin draw(1,1);draw(4,1);draw(14,1);end
  else begin
   draw(0,1);draw(1,1);draw(2,1);draw(3,1);draw(4,1);
   draw(5,0);draw(6,1);draw(7,0);draw(8,0);draw(9,0);
   draw(10,1);draw(11,0);draw(12,0);draw(13,0);draw(14,1);
  end
  $fclose(trace);
  $display("PASS red_ball_merge %0dx%0d: %0d checks %0d frames, %0d candidate slots; top-notch two/three-branch union, one-row gap, exact once-only pixel area, negative connected non-balls, N+12",WIDTH,HEIGHT,checks,frames,CANDIDATES);
  $finish;
 end
 initial begin #1000000000;$fatal(1,"merge timeout");end
endmodule
