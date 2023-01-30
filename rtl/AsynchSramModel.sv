`ifndef ASYNCH_SRAM_SV
`define ASYNCH_SRAM_SV

module AsynchSRAMModel #(
    parameter DEPTH = 256,
    parameter WIDTH = 8,
    parameter ADDR_BITS=$clog2(DEPTH)
)(
    input logic Rst,
    input logic [ADDR_BITS-1:0] Addr,
    input logic [WIDTH-1:0] WrData,
    input logic RdEn,
    input logic WrEn,
    output logic [WIDTH-1:0] RdData
);
    logic [DEPTH-1:0][WIDTH-1:0] DataCells;
    logic [DEPTH-1:0] WordLines;
    logic [WIDTH-1:0] DataLines;
    logic Enable;

    assign Enable = RdEn | WrEn;

    assert (RdEn != WrEn) else $error("AsynchSRAM RdEn and WrEn simultaneously asserted");

    for (genvar i=0; i<DEPTH; i++) begin
        assign WordLines[i] = (Addr == i) ? Enable : 1'b0;
    end

    always_comb begin
        DataLines = 'Z;
        if (WrEn) DataLines = WrData;
        for (int i=0; i<DEPTH; i++) begin
            if (WordLines[i] & RdEn) DataLines = DataCells[i]; 
            if (WordLines[i] & WrEn) DataCells[i] = DataLines;
        end
        if (Rst) DataCells = '0;
    end

    assign RdData = DataLines;

endmodule

`endif // ASYNCH_SRAM_SV