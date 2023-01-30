`ifndef CACHE_TB_COMPONENTS_SV
`define CACHE_TB_COMPONENTS_SV

//===================================================================

class mem_model extends uvm_component;

    `uvm_component_utils_begin(mem_model)
    `uvm_component_utils_end

    line_data_t [LINE_BITS-1:0] mem [mem_addr_t];

    function bit exists(mem_addr_t addr);
        return mem.exists(addr);
    endfunction

    function void randomize_data(mem_addr_t addr);
        mem[addr] = $urandom();
    endfunction

    function line_data_t read(mem_addr_t addr);
        if (!mem.exists(addr)) randomize_data(addr);
        read = mem[addr];
    endfunction

    function void write(mem_addr_t addr, line_data_t data);
        mem[addr] = data;
    endfunction

endclass mem_model

//===================================================================

class cache_tx extends uvm_sequence_item;

    rand cache_addr_tb_t addr;
    rand bit [LINE_BITS-1:0] wr_data;
    rand bit rd_val;
    rand bit wr_val;
    bit addr_rdy;
    bit data_rdy;
    bit [LINE_BITS-1:0] rd_data;
    bit rdfill_val;
    bit rdfill_rdy;
    bit hit;
    bit miss;
    
    cache_addr_tb_t wrback_addr;
    bit wrback_val;
    bit wrback_rdy;

    rand int unsigned init_tx_delay;
    rand int unsigned rdfill_val_delay;
    rand int unsigned wrback_rdy_delay;

    constraint c_rd_wr_val {
        rd_val != wr_val;
    }
    constraint c_delay {
        init_tx_delay <= 20;
        rdfill_val_delay <= 20;
        wrback_rdy_delay <= 20; 
    }

    `uvm_object_utils_begin(cache_tx)
        `uvm_field_int(addr, UVM_ALL_ON)
        `uvm_field_int(wr_data, UVM_ALL_ON)
        `uvm_field_int(rd_val, UVM_ALL_ON)
        `uvm_field_int(wr_val, UVM_ALL_ON)
        `uvm_field_int(addr_rdy, UVM_ALL_ON)
        `uvm_field_int(data_rdy, UVM_ALL_ON)
        `uvm_field_int(rd_data, UVM_ALL_ON)
        `uvm_field_int(rdfill_val, UVM_ALL_ON)
        `uvm_field_int(rdfill_rdy, UVM_ALL_ON)
        `uvm_field_int(hit, UVM_ALL_ON)
        `uvm_field_int(miss, UVM_ALL_ON)
        `uvm_field_int(wrback_addr, UVM_ALL_ON)
        `uvm_field_int(wrback_val, UVM_ALL_ON)
        `uvm_field_int(wrback_rdy, UVM_ALL_ON)
        `uvm_field_int(init_tx_delay, UVM_ALL_ON)
        `uvm_field_int(rdfill_val_delay, UVM_ALL_ON)
        `uvm_field_int(wrback_rdy_delay, UVM_ALL_ON)
    `uvm_object_utils_end

endclass : cache_tx

//===================================================================

interface cache_if(
    input logic Clk,
    input logic Rst,
    input CacheAddr_t Addr,
    input logic AddrRdy,
    input logic RdEn,
    input logic WrEn,
    input logic RdFillEn,
    input logic [LINE_BITS-1:0] WrData,
    input logic WrDataRdy,
    input logic Hit,
    input logic HitVal,
    input logic [LINE_BITS-1:0] RdData,
    input logic RdDataVal,
    input logic [ADDR_BITS-1:0] WrBackAddr,
    input logic WrBackAddrVal,
    input logic WrBackDataVal,
    input logic WrBackAddrRdy
);
    clocking mcb @(posedge Clk);
        input Addr;
        input AddrRdy;
        input RdEn;
        input WrEn;
        input RdFillEn;
        input WrData;
        input WrDataRdy;
        input Hit;
        input HitVal;
        input RdData;
        input RdDataVal;
        input WrBackAddr;
        input WrBackAddrVal;
        input WrBackDataVal;
        input WrBackAddrRdy;
    endclocking

    clocking dcb @(posedge Clk);
        output Addr;
        output RdEn;
        output WrEn;
        output RdFillEn;
        output WrData;
        output WrBackAddrRdy;
    endclocking
endinterface

//===================================================================

class cache_monitor extends uvm_monitor;

    cache_tx tx;
    virtual cache_if vif;
    uvm_analysis_port #(cache_tx) mon_ap;

    `uvm_component_utils_begin(cache_monitor)
    `uvm_component_utils_end

    function new (string name="", uvm_component parent);
        super.new(name, parent);
        mon_ap = new("mon_ap", this);
    endfunction


    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(virtual cache_if)::get(this, "", "vif", vif);
    endfunction

    task run_phase(uvm_phase phase);
        forever begin
            @vif.mcb;
        end
    endtask
endclass : cache_monitor

//===================================================================

class cache_driver extends uvm_driver#(cache_tx);

    virtual cache_if vif;
    uvm_analysis_port #(cache_tx) drv_ap;
    mem_model mm;
    int num_tx = 100;

    `uvm_component_utils_begin(cache_driver)
    `uvm_component_utils_end

    function new(string name="", uvm_component parent);
        super.new(name,parent);
        drv_ap = new("drv_ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(virtual cache_if)::get(this, "", "vif", vif);
    endfunction

    function void drive_signals(cache_tx tx);
        vif.dcb.Addr <= tx.addr;
        vif.dcb.WrData <= tx.wr_data;
        vif.dcb.RdEn <= tx.rd_en;
        vif.dcb.WrEn <= tx.wr_en;
        vif.dcb.RdFillEn <= tx.rd_fill_en;
        vif.dcb.WrData <= tx.wr_data;
        vif.dcb.WrBackAddrRdy <= tx.wrback_rdy;
    endfunction

    task run_phase(uvm_phase phase);
        cache_tx tx;
        int tx_cnt;
        while(tx_cnt < num_tx) begin
            @vif.mcb;
            if(vif.mcb.Rst) begin
                vif.dcb.Addr <= '0;
                vif.dcb.WrData <= '0;
                vif.dcb.RdEn <= '0;
                vif.dcb.WrEn <= 0;
                vif.dcb.RdFillEn <= '0;
                vif.dcb.WrData <= '0;
                vif.dcb.WrBackAddrRdy <= '0;
                continue;
            end
            tx = cache_tx::type_id::create("tx");
            tx.randomize();
            tx_cnt++;
            if (tx.rd_en) begin
                tx.rd_data = mm.read(tx.addr);
            end
            else if (tx.wr_en) begin
                mm.write(tx.addr, tx.wr_data);
            end
            drv_ap.write(tx);

            repeat(tx.init_tx_delay) @vif.mcb;
            drive_signals(tx);

            while(!vif.mcb.HitVal) @vif.mcb;
            if (vif.mcb.Hit) continue;
            if (tx.rd_en) begin
                if (vif.mcb.WrBackAddrVal) begin
                    repeat(tx.wrback_rdy_delay) @vif.mcb;
                    vif.dcb.WrBackAddrRdy <= 1'b1;
                    @vif.mcb;
                    vif.dcb.WrBackAddrRdy <= 1'b0;
                end
                repeat(tx.rdfill_val_delay) @vif.mcb;
                vif.dcb.RdFillEn <= 1'b1;
                @vif.mcb;
                vif.dcb.RdFillEn <= 1'b0;
            end
            else if (tx.wr_en) begin
                if (vif.mcb.WrBackAddrVal) begin
                    repeat(tx.wrback_rdy_delay) @vif.mcb;
                    vif.dcb.WrBackAddrRdy <= 1'b1;
                    @vif.mcb;
                    vif.dcb.WrBackAddrRdy <= 1'b0;
                end
            end
        end
    endtask

endclass : cache_driver

//===================================================================

class cache_agent extends uvm_agent;

    `uvm_component_utils_begin(cache_agent)
    `uvm_component_utils_end

    cache_monitor mon;
    cache_driver drv;

    function new(string name="", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        drv = cache_driver::type_id::create("drv", this);
        mon = cache_driver::type_id::create("mon", this);
    endfunction : build_phase

endclass : cache_agent

//===================================================================

class cache_env extends uvm_environment;
    `uvm_component_utils_begin(cache_env)
    `uvm_component_utils_end

    mem_model mm;
    cache_agent agent;

    function void build_phase(uvm_phase phase);
        agent = cache_agent::type_id::create("agent", this);
        mm = mem_model::type_id::create("mm", this);
        agent.drv.mm = mm;
    endfunction : build_phase
endclass

//===================================================================

class cache_base_test extends uvm_test;
    `uvm_component_utils_begin(cache_base_test)
    `uvm_component_utils_end
    cache_env env;
    
    function void build_phase(uvm_phase phase);
        agent = cache_agent::type_id::create("agent", this);
    endfunction : build_phase
endclass

//===================================================================

`endif // CACHE_TB_COMPONENTS_SV