`ifndef FLOP_SV
`define FLOP_SV

module flop #(
    parameter WIDTH=1
) (
    input logic Clk,
    input logic En,
    input logic [WIDTH-1:0] D,
    output logic [WIDTH-1:0] Q
);
    always_ff @ (posedge Clk) begin
        if (En) Q <= D;
    end
endmodule

`endif // FLOP_SV