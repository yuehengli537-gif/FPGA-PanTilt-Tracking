// Coherent latest-value mailbox between unrelated clocks.
// The publisher keeps held_data stable until an acknowledged delivery. A
// one-entry pending slot coalesces bursts to the newest complete measurement.
// arst_n MUST reset both domains together. Each domain releases reset on its
// own clock; neither a stopped clock nor reset may create a phantom update.
module servo_measurement_cdc #(
    parameter integer WIDTH=29
)(
    input wire arst_n,
    input wire src_clk,src_update,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk,
    output reg dst_update,
    output reg [WIDTH-1:0] dst_data
);
    (* syn_preserve=1, async_reg="true" *) reg [1:0] src_reset_sync=2'b00;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] dst_reset_sync=2'b00;
    always @(posedge src_clk or negedge arst_n)
        if(!arst_n)src_reset_sync<=0;
        else src_reset_sync<={src_reset_sync[0],1'b1};
    always @(posedge dst_clk or negedge arst_n)
        if(!arst_n)dst_reset_sync<=0;
        else dst_reset_sync<={dst_reset_sync[0],1'b1};
    wire src_run_n=arst_n && src_reset_sync[1];
    wire dst_run_n=arst_n && dst_reset_sync[1];

    reg request_toggle,acknowledge_toggle;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] acknowledge_sync;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] request_sync;
    reg [WIDTH-1:0] held_data,pending_data;
    reg pending_valid;
    wire source_busy=request_toggle!=acknowledge_sync[1];

    always @(posedge src_clk or negedge src_run_n)begin
        if(!src_run_n)begin
            acknowledge_sync<=0;request_toggle<=0;
            held_data<=0;pending_data<=0;pending_valid<=0;
        end else begin
            acknowledge_sync<={acknowledge_sync[0],acknowledge_toggle};
            if(!source_busy)begin
                // A concurrent new sample supersedes any older pending one.
                if(src_update)begin
                    held_data<=src_data;request_toggle<=~request_toggle;
                    pending_valid<=0;
                end else if(pending_valid)begin
                    held_data<=pending_data;request_toggle<=~request_toggle;
                    pending_valid<=0;
                end
            end else if(src_update)begin
                pending_data<=src_data;pending_valid<=1;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_run_n)begin
        if(!dst_run_n)begin
            request_sync<=0;acknowledge_toggle<=0;
            dst_data<=0;dst_update<=0;
        end else begin
            request_sync<={request_sync[0],request_toggle};
            dst_update<=0;
            if(request_sync[1]!=acknowledge_toggle)begin
                // The data bus has remained stable throughout the two-stage
                // request synchronization and remains held until the ack
                // travels back. Never synchronize each payload bit separately.
                dst_data<=held_data;dst_update<=1;
                acknowledge_toggle<=request_sync[1];
            end
        end
    end
endmodule
