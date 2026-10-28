`timescale 1ns/1ps
// Independent line-level 8N1 stimulus/decoder; no RTL uart_rx/tx reused here.
module uart_host_model #(parameter real BIT_NS=9000.0)(
    output reg to_dut=1,input wire from_dut);
    reg [7:0] frame[0:15];
    reg [7:0] received[0:8191];
    integer received_count=0;
    reg [7:0] incoming;
    integer bit_no;
    function [15:0] crc_byte(input [15:0] seed,input [7:0] value);
        reg [15:0] c;integer n;
        begin
            c=seed^(value<<8);
            for(n=0;n<8;n=n+1) c=c[15]?((c<<1)^16'h1021):(c<<1);
            crc_byte=c;
        end
    endfunction
    task make_frame(input [7:0] command,seq_id, input [63:0] payload,
                    input bad_crc,input [7:0] version,flags);
        integer n;reg [15:0] c;
        begin
            frame[0]=8'ha5;frame[1]=8'h5a;frame[2]=version;frame[3]=command;
            frame[4]=seq_id;frame[5]=flags;
            for(n=0;n<8;n=n+1)frame[6+n]=payload>>(8*n);
            c=16'hffff;for(n=2;n<14;n=n+1)c=crc_byte(c,frame[n]);
            if(bad_crc)c=c^16'h0001;
            frame[14]=c[15:8];frame[15]=c[7:0];
        end
    endtask
    task send_byte(input [7:0] value);
        integer n;begin
            to_dut=0;#(BIT_NS);
            for(n=0;n<8;n=n+1)begin to_dut=value[n];#(BIT_NS);end
            to_dut=1;#(BIT_NS);
        end
    endtask
    task send_frame;
        integer n;begin for(n=0;n<16;n=n+1)send_byte(frame[n]);end
    endtask
    task send_packet(input [7:0] command,seq_id,input [63:0] payload,
                     input bad_crc,input [7:0] version,flags);
        begin make_frame(command,seq_id,payload,bad_crc,version,flags);send_frame();end
    endtask
    task await_reply(input integer offset,input [7:0] seq_id,reply_type);
        integer guard,n;reg [15:0] c;
        begin
            guard=0;
            while(received_count<offset+16 && guard<4000)begin #(BIT_NS);guard=guard+1;end
            if(received_count!=offset+16)$fatal(1,"reply length/timeout offset=%0d count=%0d",offset,received_count);
            if(received[offset]!=8'ha5||received[offset+1]!=8'h5a||received[offset+2]!=1)
                $fatal(1,"bad reply header");
            if(received[offset+3]!=reply_type||received[offset+4]!=seq_id)
                $fatal(1,"reply type/seq got%h/%h expected%h/%h",received[offset+3],received[offset+4],reply_type,seq_id);
            c=16'hffff;for(n=2;n<14;n=n+1)c=crc_byte(c,received[offset+n]);
            if({received[offset+14],received[offset+15]}!=c)$fatal(1,"reply CRC mismatch");
            if(received[offset+5][7:3]!=0)$fatal(1,"unexpected mode/reserved flags");
            #(BIT_NS*3); // Allow the transmitter to finish the final stop bit.
        end
    endtask
    function [15:0] word_at(input integer offset,input integer n);
        word_at={received[offset+7+n*2],received[offset+6+n*2]};
    endfunction
    initial forever begin
        @(negedge from_dut);
        #(BIT_NS*1.5);
        for(bit_no=0;bit_no<8;bit_no=bit_no+1)begin incoming[bit_no]=from_dut;#(BIT_NS);end
        if(from_dut!==1)$fatal(1,"UART reply framing error");
        received[received_count]=incoming;received_count=received_count+1;
    end
endmodule
