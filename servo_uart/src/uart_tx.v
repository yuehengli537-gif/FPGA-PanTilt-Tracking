// valid && ready accepts a byte; ready returns after the full stop bit.
module uart_tx #(
    parameter integer CLK_HZ=27000000,
    parameter integer BAUD=115200
)(input wire clk,rst_n,
  input wire [7:0] data,input wire valid,
  output wire ready,output reg tx);
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer TIMER_BITS=(BIT_CYCLES>1)?$clog2(BIT_CYCLES):1;
    reg busy;
    reg [TIMER_BITS-1:0] timer;
    reg [3:0] bit_index;
    reg [7:0] shift;
    assign ready=!busy;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin busy<=0;timer<=0;bit_index<=0;shift<=0;tx<=1;end
        else if(!busy)begin
            tx<=1;
            if(valid)begin busy<=1;shift<=data;bit_index<=0;timer<=BIT_CYCLES-1;tx<=0;end
        end else if(timer!=0)timer<=timer-1'b1;
        else begin
            timer<=BIT_CYCLES-1;
            if(bit_index<8)begin tx<=shift[bit_index];bit_index<=bit_index+1'b1;end
            else if(bit_index==8)begin tx<=1;bit_index<=9;end
            else begin busy<=0;tx<=1;end
        end
    end
endmodule
