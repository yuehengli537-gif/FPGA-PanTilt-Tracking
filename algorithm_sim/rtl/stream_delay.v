`timescale 1ns/1ps
// Clock delay (not valid-pixel delay), synchronous read/write circular RAM.
// An input sampled at n appears at n+DELAY. No reset of RAM contents.
module stream_delay #(
    parameter integer BITS=8, DELAY=1
) (
    input wire clk,rst_n,
    input wire [BITS-1:0] in_data,
    output reg [BITS-1:0] out_data
);
    localparam integer ADDR_BITS=(DELAY<2)?1:$clog2(DELAY);
    reg [BITS-1:0] memory[0:DELAY-1];
    reg [ADDR_BITS-1:0] pointer;
    reg [ADDR_BITS:0] filled;
    reg [BITS-1:0] read_data;
    always @(posedge clk) begin
        if(rst_n) begin
            memory[pointer]<=in_data;
            read_data<=memory[pointer];
        end
    end
    // Memory output is itself the delay's last register. Reset only guards
    // access; output masking below prevents stale samples after a reset.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin pointer<=0; filled<=0; end
        else begin
            pointer <= (pointer==DELAY-1) ? 0 : pointer+1'b1;
            if(filled<DELAY+1) filled<=filled+1'b1;
        end
    end
    always @* out_data=(rst_n && filled==DELAY+1)?read_data:{BITS{1'b0}};
endmodule
