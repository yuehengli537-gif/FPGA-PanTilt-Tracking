// RGB565 capture in the camera PCLK domain.
// hblank/de_o are one-cycle pixel enables, never clocks.
module cmos_8_16bit(
    input rst,
    input pclk,
    input [7:0] pdata_i,
    input de_i,
    output reg [15:0] pdata_o,
    output reg hblank,
    output wire de_o
);
    reg [7:0] first_byte;
    reg second_byte;
    assign de_o = hblank;

    always @(posedge pclk or posedge rst) begin
        if (rst) begin
            first_byte <= 8'd0;
            second_byte <= 1'b0;
            pdata_o <= 16'd0;
            hblank <= 1'b0;
        end else begin
            hblank <= 1'b0;
            if (!de_i) begin
                second_byte <= 1'b0;
            end else if (!second_byte) begin
                first_byte <= pdata_i;
                second_byte <= 1'b1;
            end else begin
                pdata_o <= {first_byte, pdata_i};
                hblank <= 1'b1;
                second_byte <= 1'b0;
            end
        end
    end
endmodule
