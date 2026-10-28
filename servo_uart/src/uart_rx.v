// 8N1 synchronized receiver with mid-bit sampling and stop-bit validation.
module uart_rx #(
    parameter integer CLK_HZ=27000000,
    parameter integer BAUD=115200
)(input wire clk,rst_n,rx,
  output reg [7:0] data,
  output reg valid,framing_error);
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer TIMER_BITS=(BIT_CYCLES>1)?$clog2(BIT_CYCLES):1;
    localparam [1:0] IDLE=0,START=1,BITS=2,STOP=3;
    reg [1:0] state;
    reg rx_meta,rx_sync;
    reg [TIMER_BITS-1:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rx_meta<=1;rx_sync<=1;state<=IDLE;timer<=0;bit_index<=0;shift<=0;
            data<=0;valid<=0;framing_error<=0;
        end else begin
            rx_meta<=rx;rx_sync<=rx_meta;valid<=0;framing_error<=0;
            case(state)
                IDLE:if(!rx_sync)begin timer<=BIT_CYCLES/2-1;state<=START;end
                START:if(timer!=0)timer<=timer-1'b1;
                    else if(rx_sync)state<=IDLE;
                    else begin timer<=BIT_CYCLES-1;bit_index<=0;state<=BITS;end
                BITS:if(timer!=0)timer<=timer-1'b1;
                    else begin
                        shift[bit_index]<=rx_sync;timer<=BIT_CYCLES-1;
                        if(bit_index==7)state<=STOP;else bit_index<=bit_index+1'b1;
                    end
                STOP:if(timer!=0)timer<=timer-1'b1;
                    else begin
                        state<=IDLE;
                        if(rx_sync)begin data<=shift;valid<=1;end
                        else framing_error<=1;
                    end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
