`timescale 1ns/1ps
module tb_servo_measurement_cdc;
    reg arst_n=0,src_clk=0,dst_clk=0;
    reg src_running=1,dst_running=1;
    reg src_update=0;
    reg [63:0] src_data=0;
    wire dst_update;
    wire [63:0] dst_data;
    integer highest_sent=0,last_received=0,received=0;
    integer saved_received=0,i;
    reg previously_busy;
    reg [63:0] previously_held;
    always #3.5 if(src_running)src_clk=~src_clk;
    always #11 if(dst_running)dst_clk=~dst_clk;
    servo_measurement_cdc #(.WIDTH(64)) dut(
        .arst_n(arst_n),.src_clk(src_clk),.src_update(src_update),.src_data(src_data),
        .dst_clk(dst_clk),.dst_update(dst_update),.dst_data(dst_data));
    function [63:0] pattern(input integer sequence_number);
        pattern={32'(sequence_number),~32'(sequence_number)};
    endfunction
    task send_one(input integer sequence_number);
        begin
            @(negedge src_clk);highest_sent=sequence_number;
            src_data=pattern(sequence_number);src_update=1;
            @(negedge src_clk);src_update=0;
        end
    endtask
    task await_latest(input integer expected);
        integer cycles;
        begin
            cycles=0;
            while(last_received!=expected && cycles<1000)begin
                @(posedge dst_clk);#2;cycles=cycles+1;
            end
            if(last_received!=expected)
                $fatal(1,"latest snapshot lost: expected %0d got %0d",expected,last_received);
        end
    endtask
    // Both words must belong to one publisher sample. Samples may be skipped,
    // but never fabricated, duplicated, reordered or torn across clock domains.
    always @(posedge dst_clk or negedge arst_n)begin
        if(!arst_n)begin last_received=0;received=0;end
        else begin
            #1;
            if(dst_update)begin
                if(dst_data[31:0]!==~dst_data[63:32])$fatal(1,"torn payload %h",dst_data);
                if(dst_data[63:32]<=last_received || dst_data[63:32]>highest_sent)
                    $fatal(1,"phantom/reordered snapshot: previous %0d got %0d sent %0d",
                           last_received,dst_data[63:32],highest_sent);
                last_received=dst_data[63:32];received=received+1;
            end
        end
    end
    always @(posedge src_clk)begin
        previously_busy=dut.source_busy;previously_held=dut.held_data;
        #1;
        if(arst_n && dut.src_run_n && previously_busy && dut.held_data!==previously_held)
            $fatal(1,"publisher changed held payload before acknowledgement");
    end
    initial begin
        #80;arst_n=1;
        repeat(6)@(negedge src_clk);
        send_one(1);await_latest(1);
        saved_received=received;
        repeat(30)@(posedge dst_clk);
        if(received!=saved_received)$fatal(1,"idle publisher repeated delivery");
        for(i=2;i<=120;i=i+1)begin
            @(negedge src_clk);highest_sent=i;src_data=pattern(i);src_update=1;
        end
        @(negedge src_clk);src_update=0;
        await_latest(120);
        if(received>=120)$fatal(1,"burst did not exercise pending coalescing");
        $display("PASS CDC burst coherence and newest snapshot: %0d deliveries for 120 updates",received);

        // With receiver stopped, one in-flight sample stays held and the
        // pending slot keeps the newest of all subsequent camera updates.
        @(negedge dst_clk);dst_running=0;
        for(i=121;i<=180;i=i+1)begin
            @(negedge src_clk);highest_sent=i;src_data=pattern(i);src_update=1;
        end
        @(negedge src_clk);src_update=0;
        #200;dst_running=1;
        await_latest(180);
        $display("PASS CDC destination stall preserves newest pending snapshot");

        // Stopping the source after launch permits at most the outstanding
        // delivery; it must not synthesize a new measurement every dst cycle.
        send_one(181);
        @(negedge src_clk);src_running=0;
        await_latest(181);saved_received=received;
        repeat(50)@(posedge dst_clk);
        if(received!=saved_received)$fatal(1,"stalled source generated phantom updates");
        $display("PASS CDC stopped camera produces no false freshness pulses");

        // Assert reset while clocks are both stopped, then release while the
        // camera is still stopped. Control side must see no stale pre-reset
        // event, and a later camera restart must deliver fresh data normally.
        @(negedge dst_clk);dst_running=0;
        #3;arst_n=0;src_update=0;highest_sent=0;
        #3;
        if(dst_update!==0 || dst_data!==0)$fatal(1,"asynchronous reset failed with stopped clocks");
        #30;arst_n=1;dst_running=1;
        repeat(15)@(posedge dst_clk);
        if(dst_update!==0 || received!=0)$fatal(1,"reset release replayed old measurement");
        src_running=1;repeat(6)@(negedge src_clk);
        send_one(1);await_latest(1);

        // Reset with an occupied pending slot also discards that old slot.
        @(negedge dst_clk);dst_running=0;
        send_one(2);send_one(3);send_one(4);
        #2;arst_n=0;src_update=0;highest_sent=0;
        #40;arst_n=1;dst_running=1;
        repeat(30)@(posedge dst_clk);
        if(received!=0)$fatal(1,"reset replayed a pending pre-reset snapshot");
        send_one(1);await_latest(1);
        $display("PASS CDC reset with stopped clocks and occupied mailbox");
        $display("PASS tb_servo_measurement_cdc: atomic/latest/reset/stalls");
        $finish;
    end
    initial begin #1000000;$fatal(1,"CDC test timeout");end
endmodule
