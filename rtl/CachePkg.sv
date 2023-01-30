`ifndef CACHE_PKG_SV
`define CACHE_PKG_SV

package CachePkg;
    localparam SET_SIZE=4;
    localparam SET_PTR_BITS=$clog2(SET_SIZE);
    localparam LINE_BYTES = 16;
    localparam ADDR_BITS = 32;
    localparam LINE_BITS = LINE_BYTES*8;
    localparam OFFS_BITS = $clog2(LINE_BYTES);
    localparam IDX_BITS = $clog2(SET_SIZE);
    localparam CTAG_BITS = ADDR_BITS - OFFS_BITS - IDX_BITS;
    localparam MEM_BYTES = 2**ADDR_BITS;
    localparam NUM_CACHE_SETS = MEM_BYTES / (LINE_BYTES * SET_SIZE);
    localparam NUM_LINES = NUM_CACHE_SETS * SET_SIZE;

    typedef struct packed {
        logic [CTAG_BITS-1:0] Tag;
        logic [IDX_BITS-1:0] Idx;
        logic [OFFS_BITS-1:0] Offs;
    } CacheAddr_t;

    typedef struct packed {
        logic Valid;
        logic Dirty;
        logic [SET_PTR_BITS-1:0] Age;
        logic [CTAG_BITS-1:0] Tag;
    } CacheLineInfo_t;
    
    typedef bit [4:0] enum {
        IDLE        = 5'b00000,
        READ        = 5'b00001,
        WRITE       = 5'b00010,
        READ_WB     = 5'b00100,
        WRITE_WB    = 5'b01000,
        READ_FILL   = 5'b10000
    } CacheState_t;

endpackage : CachePkg

`endif // CACHE_PKG_SV