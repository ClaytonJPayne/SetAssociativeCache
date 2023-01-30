`ifndef SET_ASSOCIATIVE_CACHE_SV
`define SET_ASSOCIATIVE_CACHE_SV
 
import CachePkg::*;

/*
    Parmeterized set-associative cache
    LRU eviction policy
    Write-back policy
*/

module SetAssociativeCache
(
    input logic Clk,
    input logic Rst,    // FIXME Clayton not currently used
    input CacheAddr_t Addr,
    output logic AddrRdy,
    input logic RdEn,
    input logic WrEn,
    input logic RdFillEn,
    input logic [LINE_BITS-1:0] WrData,
    output logic WrDataRdy,
    output logic Hit,
    output logic HitVal,
    output logic [LINE_BITS-1:0] RdData,
    output logic [ADDR_BITS-1:0] WrBackAddr,
    output logic WrBackAddrVal,
    output logic WrBackDataVal,
    input logic WrBackAddrRdy
);

    // Declarations

    // Cache state
    CacheAddr_t Addr_1;
    CacheState_t CacheState, CacheStateD;
    logic LdInputAddr, LdInputData;
    
    logic HitValD;

    // Line state
    logic [SET_SIZE-1:0] LineValid, LineValidD, LineValid_1;
    logic [SET_SIZE-1:0] LineDirty, LineDirtyD, LineDirty_1;
    logic [SET_SIZE-1:0][SET_PTR_BITS-1:0] LineAge, LineAgeD, LineAge_1;
    logic [SET_SIZE-1:0][TAG_BITS-1:0] LineTag, LineTagD, LineTag_1;
    CacheLineInfo_t [SET_SIZE-1:0] CacheLineInfo_1;
    CacheLineInfo_t HitLineInfo_1, ReplLineInfo_1;

    // Line state control
    logic LineStateRdEn;
    logic [SET_PTR_BITS-1:0] HitIdx_1, ReplIdx_1, MRUIdx_1; // MRUIdx will be selected from HitIdx or ReplIdx
    logic [SET_SIZE-1:0] HitIdxOneHot_1, LRUIdxOneHot_1, ReplIdxOneHot_1;
    logic Hit_1;
    logic InvalidateMRU, ValidateMRU, UpdateValid;
    logic MakeDirtyMRU, MakeCleanMRU, UpdateDirty;
    logic UpdateAge;
    logic UpdateTag;
    logic [SET_PTR_BITS-1:0] AgeThreshold;

    // Line data reads/writes
    logic [LINE_BITS-1:0] WrData_1;
    logic [IDX_BITS+SET_PTR_BITS-1:0] CacheLineAddr_1;
    logic [LINE_BITS-1:0] CacheLineWrData;
    logic [LINE_BITS-1:0] CacheLineRdData;
    logic CacheLineRdEn, CacheLineWrEn;

    // Cache structure

    // SRAMs holding cache line state metadata

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE))
    CacheLineValid
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .WrData(LineValidD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateValid),
        .RdData(LineValid)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE))
    CacheLineDirty
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .WrData(LineDirtyD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateDirty),
        .RdData(LineDirty)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE*SET_PTR_BITS))
    CacheLineAge
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .WrData(LineAgeD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateAge),
        .RdData(LineAge)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE*SET_PTR_BITS))
    CacheLineTag
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .WrData(LineTagD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateValid),
        .RdData(LineTag)
    ); 

    // SRAM holding cache line data

    AsynchSRAMModel #(.DEPTH(NUM_LINES), .WIDTH(LINE_BITS))
    CacheLineData
    (
        .Rst(1'b0),
        .Addr(CacheLineAddr_1)
        .WrData(CacheLineWrData),
        .RdEn(CacheLineRdEn),
        .WrEn(CacheLineWrEn),
        .RdData(CacheLineRdData)
    ); 

    // Cache state machine / control

    assert ((RdEn != WrEn) && (RdEn != ReplEn) && (WrEn != ReplEn)) else $error("Invalid RdEn/WrEn/ReplEn combo");

    always_comb begin
        CacheStateD = IDLE;    

        LdInputAddr = 1'b0;
        LdInputData = 1'b0;   

        InvalidateMRU = 1'b0;
        ValidateMRU = 1'b0;
        MakeCleanMRU = 1'b0;
        MakeDirtyMRU = 1'b0;
        UpdateAge = 1'b0;
        UpdateTag = 1'b0;
        LineStateRdEn = 1'b0;
        AgeThreshold = '1;

        CacheLineRdEn = 1'b0;
        CacheLineWrEn = 1'b0;
        CacheLineWrData = WrData_1;

        AddrRdy = 1'b0;
        WrDataRdy = 1'b0;

        WrBackAddrVal = 1'b0;
        WrBackDataVal = 1'b0;
        HitValD = HitVal;

        case (CacheState)
            IDLE: begin
                AddrRdy = 1'b1;
                WrDataRdy = 1'b1;
                if (RdEn) begin
                    CacheStateD = READ;
                    LdInputAddr = 1'b1;
                    LineStateRdEn = 1'b1;
                    HitValD = 1'b0;
                    ReadDataValD = 1'b0;
                end
                if (WrEn) begin
                    CacheStateD = WRITE;
                    LdInputAddr = 1'b1;
                    LdInputData = 1'b1;
                    LineStateRdEn = 1'b1;
                    HitValD = 1'b0;
                    ReadDataValD = 1'b0;
                end
            end
            READ: begin
                UpdateAge = 1'b1;
                HitValD = 1'b1;
                if (Hit_1) begin
                    CacheStateD = IDLE;
                    CacheLineRdEn = 1'b1;
                    AgeThreshold = HitLineInfo_1.Age;
                    ReadDataValD = 1'b1;
                end
                else begin
                    InvalidateMRU = 1'b1;
                    if (ReplLineInfo_1.Valid) begin
                        AgeThreshold = ReplLineInfo_1.Age;
                        if (ReplLineInfo_1.Dirty) begin
                            CacheStateD = READ_WB;
                        end
                        else begin
                            CacheStateD = READ_FILL;
                        end
                    end
                    else begin
                        AgeThreshold = '0;
                        CacheStateD = READ_FILL;
                    end
                end
            end
            WRITE: begin
                UpdateAge = 1'b1;
                HitValD = 1'b1;
                if (Hit_1) begin
                    CacheStateD = IDLE;
                    CacheLineWrEn = 1'b1;
                    MakeDirtyMRU = 1'b1;
                    AgeThreshold = HitLineInfo_1.Age;
                end
                else begin
                    if (ReplLineInfo_1.Valid) begin
                        AgeThreshold = ReplLineInfo_1.Age;
                        if (ReplLineInfo_1.Dirty) begin
                            CacheStateD = WRITE_WB;
                            InvalidateMRU = 1'b1;
                        end
                        else begin
                            CacheStateD = IDLE;
                            CacheLineWrEn = 1'b1;
                            MakeDirtyMRU = 1'b1;
                            CacheLineWrEn = 1'b1;
                            UpdateTag = 1'b1;
                        end
                    end
                    else begin
                        CacheStateD = IDLE;
                        AgeThreshold = '0;
                        WrCacheEn = 1'b1;
                        MakeDirtyMRU = 1'b1;
                        ValidateMRU = 1'b1;
                        UpdateTag = 1'b1;
                    end
                end
            end
            READ_WB: begin
                WrBackAddrVal = 1'b1;
                WrBackDataVal = 1'b1;
                HitValD = 1'b0;
                if (WrBackRdy) begin
                    CacheStateD = READ_FILL;
                end
                else begin
                    CacheStateD = READ_WB;
                end
            end
            WRITE_WB: begin
                WrBackAddrVal = 1'b1;
                WrBackDataVal = 1'b1;
                HitValD = 1'b0;
                if (WrBackRdy) begin
                    CacheStateD = IDLE;
                    WrCacheEn = 1'b1;
                    MakeDirtyMRU = 1'b1;
                    ValidateMRU = 1'b1;
                    UpdateTag = 1'b1;
                end
                else begin
                    CacheStateD = WRITE_WB;
                end
            end
            READ_FILL: begin
                HitValD = 1'b0;
                WrDataRdy = 1'b1;
                if (RdFillEn) begin
                    CacheStateD = IDLE;
                    CacheLineWrEn = 1'b1;
                    MakeCleanMRU = 1'b1;
                    ValidateMRU = 1'b1;
                    UpdateTag = 1'b1;
                    CacheLineWrData = WrData;
                end
                else begin
                    CacheStateD = READ_FILL;
                end
            end
        endcase
    end

    // Input and line state read flops

    assign UpdateValid = InvalidateMRU | ValidateMRU;
    assign UpdateDirty = MakeCleanMRU | MakeDirtyMRU;
    flop #(4)         I_CacheState      (.Q(CacheState),    .D(CacheStateD), .Clk(Clk), .En(1'b1));
    flop #(ADDR_BITS) I_Addr_1          (.Q(Addr_1),        .D(Addr),        .Clk(Clk), .En(LdInputAddr));
    flop #(LINE_BITS) I_WrData_1        (.Q(WrData_1),      .D(WrData),      .Clk(Clk), .En(LdInputData));
    flop #(SET_SIZE)  I_LineValid_1     (.Q(LineValid_1),   .D(LineValid),   .Clk(Clk), .En(LineStateRdEn));
    flop #(SET_SIZE)  I_LineDirty_1     (.Q(LineDirty_1),   .D(LineDirty),   .Clk(Clk), .En(LineStateRdEn));
    flop #(SET_SIZE)  I_LineAge_1       (.Q(LineAge_1),     .D(LineAge),     .Clk(Clk), .En(LineStateRdEn));
    flop #(SET_SIZE)  I_LineTag_1       (.Q(LineTag_1),     .D(LineTag),     .Clk(Clk), .En(LineStateRdEn));

    // Determine whether we have a cache hit or miss; determine which line to allocate/replace if it turns out to be a miss

    for (genvar i=0; i<SET_SIZE; i++) begin

        // Reshape set state metadatar into array of individual lines' state metadata
        assign CacheLineInfo_1[i].Valid = LineValid_1[i];
        assign CacheLineInfo_1[i].Dirty = LineDirty_1[i];
        assign CacheLineInfo_1[i].Age = LineAge_1[i];
        assign CacheLineInfo_1[i].Tag = LineTag_1[i];

        // Decode cache hit and find LRU entry in case we need to evict
        assign HitIdxOneHot_1[i] = (Addr_1.Tag == LineTag_1[i]) & (LineValid_1[i]);
        assign LRUIdxOneHot_1[i] = ~(|LineAge_1[i]) & (LineValid_1[i]);
    end

    // We can evict any invalid entry, or the LRU entry
    assign ReplIdxOneHot_1 = ~(CacheSetInfo.Valid) | LRUIdxOneHot_1;

    assign Hit_1 = |HitIdxOneHot_1;
    assign MRUIdx_1 = Hit_1 ? HitIdx_1 : ReplIdx_1;
    assign FullCacheAddr_1 = {Addr_1.Idx, MRUIdx_1};

    always_comb begin
        HitIdx_1 = '0;
        ReplIdx_1 = '0;
        HitLineInfo_1 = '0;
        ReplLineInfo_1 = '0;
        for (int i=SET_SIZE-1; i>=0; i--) begin

            // Normal mux to select line you hit
            HitIdx_1 |= HitIdxOneHot_1[i] ? SET_PTR_BITS'(i) : '0;
            HitLineInfo_1 |= HitIdxOneHot_1[i] ? CacheLineInfo_1[i] : '0;

            // Priority mux to select a line to evict
           if (ReplIdxOneHot_1[i]) ReplIdx_1 = SET_PTR_BITS'(i);
           if (ReplIdxOneHot_1[i]) ReplLineInfo_1 = CacheLineInfo_1[i];
        end
    end   

    // Calculate next MRU state values

    for (genvar i=0; i<SET_SIZE; i++) begin
        assign LineAgeD[i]   =    (i == MRUIdx_1)               ? {SET_PTR_BITS{1'b1}} : '0 
                                | (LineAge_1[i] < AgeThreshold) ? LineAge_1[i]         : '0 
                                | (LineAge_1[i] > AgeThreshold) ? LineAge_1[i] - 1     : '0;
        assign LineValidD[i] =    (i != MRUIdx_1) & LineValid_1[i]
                                | (i == MRUIdx_1) & ValidateMRU
                                | !InvalidateMRU  & LineValid_1[i];
        assign LineDirtyD[i] =    (i != MRUIdx_1) & LineDirty_1[i]
                                | (i == MRUIdx_1) & MakeDirtyMRU
                                | !MakeCleanMRU   & LineDirty_1[i];
        assign LineTagD[i]   =    (i == MRUIdx_1) ? Addr_1.Tag  : LineTag_1[i];
    end

    // Output flops

    CacheAddr_t WrBackAddrD;

    assign WrBackAddrD.Tag = ReplLineInfo_1.Tag;
    assign WrBackAddrD.Idx = MRUIdx_1;
    assign WrBackAddrD.Offs = OFFS_BITS'(0);

    flop #(ADDR_BITS)   I_WrBackAddr    (.Q(WrBackAddr),    .D(WrBackAddrD),     .Clk(Clk), .En(1'b1));
    flop #(LINE_BITS)   I_RdData        (.Q(RdData),        .D(CacheLineRdData), .Clk(Clk), .En(1'b1));
    flop #(1)           I_Hit           (.Q(Hit),           .D(Hit_1),           .Clk(Clk), .En(1'b1));
    flop #(1)           I_HitVal        (.Q(HitVal),        .D(HitValD),         .Clk(Clk), .En(1'b1));

endmodule

`endif // SET_ASSOCIATIVE_CACHE_SV
