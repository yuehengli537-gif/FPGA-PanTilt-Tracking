`timescale 1ns/1ps

// A signed constant brightness offset with saturation; not equalization.
module brightness_adjust #(
    parameter integer OFFSET = 24
) (
    input  wire [7:0] in_gray,
    output reg  [7:0] out_bright
);
    // One extra bit also covers extreme signed 32-bit parameter values.
    localparam signed [32:0] SIGNED_OFFSET = OFFSET;
    reg signed [32:0] adjusted;

    always @* begin
        adjusted = $signed({1'b0, in_gray}) + SIGNED_OFFSET;
        if (adjusted < 0)
            out_bright = 8'd0;
        else if (adjusted > 255)
            out_bright = 8'd255;
        else
            out_bright = adjusted[7:0];
    end
endmodule
