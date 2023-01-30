
package cache_tb_pkg;
    import uvm_pkg::*;
    typedef struct packed {
        bit [CTAG_BITS-1:0] tag;
        bit [IDX_BITS-1:0] idx;
        bit [OFFS_BITS-1:0] offs;
    } cache_addr_tb_t; 

    typedef bit [ADDR_BITS-1:0] mem_addr_t;
    typedef bit [LINE_BITS-1:0] line_data_t;

    `include "uvm_macros.svh"
    `include "cache_tb_components.sv"
endpackage