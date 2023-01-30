import cache_tb_pkg::*;
import uvm_pkg::*;

module top;

    logic Clk;
    logic Rst; 
    CacheAddr_t Addr;
    logic AddrRdy;
    logic RdEn;
    logic WrEn;
    logic RdFillEn;
    logic [LINE_BITS-1:0] WrData;
    logic WrDataRdy;
    logic Hit;
    logic HitVal;
    logic [LINE_BITS-1:0] RdData;
    logic [ADDR_BITS-1:0] WrBackAddr;
    logic WrBackAddrVal;
    logic WrBackDataVal;
    logic WrBackAddrRdy;
    
    SetAssociativeCache dut(.*);
    cache_if if (.*);
    
    always #5 Clk = ~Clk;
    initial begin
        fork
            begin
                Clk = 1'b0;
                Rst = 1'b1;
                #100 Rst = 1'b0;
            end
            run_test("cache_base_test");
        join_any
    end
    
endmodule