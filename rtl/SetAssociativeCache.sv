`ifndef SET_ASSOCIATIVE_CACHE_SV
`define SET_ASSOCIATIVE_CACHE_SV
 
import CachePkg::*;

// Parmeterized set-associative cache with LRU eviction and write-back policy

module SetAssociativeCache
(
    input logic Clk,
    input logic Rst,
    input CacheAddr_t Addr,                     // Read or write address
    output logic AddrRdy,                       // Ready to receive read or write address
    input logic RdEn,                           // Read address is valid; begin read
    input logic WrEn,                           // Write address/data is valid; begin write
    input logic RdFillEn,                       // Read fill data is valid; begin read fill
    input logic [LINE_BITS-1:0] DataIn,         // Write data or read fill data
    output logic DataInRdy,                     // Ready to receive write data or read fill data
    output logic Hit,                           // Was it a hit or a miss?
    output logic HitVal,                        // Hit/miss result is valid
    output logic [LINE_BITS-1:0] DataOut,       // Read-hit data or write-back data
    output logic RdHitDataVal,                  // Read-hit data is valid   
    output logic [ADDR_BITS-1:0] WrBackAddr,    // Address of dirty line we're writing back
    output logic WrBackVal,                     // WriteBack address and WriteBack data are valid
    input logic WrBackRdy                       // WriteBack is finished
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
    logic [SET_PTR_BITS-1:0] HitIdx_1, ReplIdx_1, MRUIdx_1;
    logic [SET_SIZE-1:0] HitIdxOneHot_1, LRUIdxOneHot_1, ReplIdxOneHot_1;
    logic Hit_1;
    logic InvalidateMRU, ValidateMRU, UpdateValid;
    logic MakeDirtyMRU, MakeCleanMRU, UpdateDirty;
    logic UpdateAge;
    logic UpdateTag;
    logic [SET_PTR_BITS-1:0] AgeThreshold;

    // Line data reads/writes
    logic [LINE_BITS-1:0] DataIn_1;
    logic [IDX_BITS+SET_PTR_BITS-1:0] CacheLineAddr_1;
    logic [LINE_BITS-1:0] DataOutD;
    logic [LINE_BITS-1:0] CacheLineDataIn;
    logic [LINE_BITS-1:0] CacheLineDataOut;
    logic CacheLineRdEn, CacheLineWrEn;

    // Cache structure
    /*
        Separate SRAMs storing state data for cache lines
        Logically, these are 2D arrays with shape [NUM_CACHE_SETS, SET_SIZE].
        Not sure if asynchronous SRAMs are actually what you'd do here, but the 
        memory needs to be fast, since in my design you read the state of the
        set you're indexing, then flop in the set's state along with your input 
        signals in the same cycle.
    */

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE))
    CacheLineValid
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .DataIn(LineValidD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateValid),
        .DataOut(LineValid)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE))
    CacheLineDirty
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .DataIn(LineDirtyD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateDirty),
        .DataOut(LineDirty)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE*SET_PTR_BITS))
    CacheLineAge
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .DataIn(LineAgeD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateAge),
        .DataOut(LineAge)
    ); 

    AsynchSRAMModel #(.DEPTH(NUM_CACHE_SETS), .WIDTH(SET_SIZE*TAG_BITS))
    CacheLineTag
    (
        .Rst(Rst),
        .Addr(Addr.Idx),
        .DataIn(LineTagD),
        .RdEn(LineStateRdEn),
        .WrEn(UpdateValid),
        .DataOut(LineTag)
    ); 

    // SRAM storing cache line date
    // Logically, it's a flattened array of length NUM_CACHE_SETS*SET_SIZE

    AsynchSRAMModel #(.DEPTH(NUM_LINES), .WIDTH(LINE_BITS))
    CacheLineData
    (
        .Rst(1'b0),
        .Addr(CacheLineAddr_1)
        .DataIn(CacheLineDataIn),
        .RdEn(CacheLineRdEn),
        .WrEn(CacheLineWrEn),
        .DataOut(CacheLineDataOut)
    ); 

    // Cache state machine / controller
    /*
        The cache has 6 states:
        1) Idle - Ready to receive new read or write transaction
        2) Read - Reading cache line and/or calculating hit or miss
        3) Write - Writing cache line and/or calculating hit or miss
        4) ReadWriteback - A dirty line has been evicted from the cache on a read miss
                         - This is a separate state from WriteWriteback, since we
                           we also must go into the ReadFill state from here, since
                           it was a miss
        5) WriteWriteback - A dirty line has been evicted from the cache on a write miss
        6) ReadFill - We're waiting to fill the line we've allocated on a read miss
                      with data from main memory
    */

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
        CacheLineDataIn = DataIn_1;
        DataOutD = CacheLineDataOut;

        AddrRdy = 1'b0;
        DataInRdy = 1'b0;

        WrBackVal = 1'b0;
        HitValD = HitVal;
        RdHitDataValD = RdHitDataVal;

        case (CacheState)
            IDLE: begin
                AddrRdy = 1'b1;
                DataInRdy = 1'b1;
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
                WrBackVal = 1'b1;
                HitValD = 1'b0;
                if (WrBackRdy) begin
                    CacheStateD = READ_FILL;
                end
                else begin
                    CacheStateD = READ_WB;
                end
            end
            WRITE_WB: begin
                WrBackVal = 1'b1;
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
                DataInRdy = 1'b1;
                if (RdFillEn) begin
                    CacheStateD = IDLE;
                    CacheLineWrEn = 1'b1;
                    MakeCleanMRU = 1'b1;
                    ValidateMRU = 1'b1;
                    UpdateTag = 1'b1;
                    CacheLineDataIn = DataIn;
                    DataOutD = DataIn;
                    ReadDataValD = 1'b1;
                end
                else begin
                    CacheStateD = READ_FILL;
                end
            end
        endcase
    end

    // Input and line state read flip-flops

    assign UpdateValid = InvalidateMRU | ValidateMRU;
    assign UpdateDirty = MakeCleanMRU | MakeDirtyMRU;
    dff #(4)                        I_CacheState      (.Q(CacheState),    .D(CacheStateD), .Clk(Clk), .En(1'b1));
    dff #(ADDR_BITS)                I_Addr_1          (.Q(Addr_1),        .D(Addr),        .Clk(Clk), .En(LdInputAddr));
    dff #(LINE_BITS)                I_DataIn_1        (.Q(DataIn_1),      .D(DataIn),      .Clk(Clk), .En(LdInputData));
    dff #(SET_SIZE)                 I_LineValid_1     (.Q(LineValid_1),   .D(LineValid),   .Clk(Clk), .En(LineStateRdEn));
    dff #(SET_SIZE)                 I_LineDirty_1     (.Q(LineDirty_1),   .D(LineDirty),   .Clk(Clk), .En(LineStateRdEn));
    dff #(SET_SIZE*SET_PTR_BITS)    I_LineAge_1       (.Q(LineAge_1),     .D(LineAge),     .Clk(Clk), .En(LineStateRdEn));
    dff #(SET_SIZE*TAG_BITS)        I_LineTag_1       (.Q(LineTag_1),     .D(LineTag),     .Clk(Clk), .En(LineStateRdEn));

    // Determine whether we have a cache hit or miss; determine which line to 
    // allocate/replace if it turns out to be a miss

    for (genvar i=0; i<SET_SIZE; i++) begin

        // Reshape set state metadata into array of packed structs
        assign CacheLineInfo_1[i].Valid = LineValid_1[i];
        assign CacheLineInfo_1[i].Dirty = LineDirty_1[i];
        assign CacheLineInfo_1[i].Age = LineAge_1[i];
        assign CacheLineInfo_1[i].Tag = LineTag_1[i];

        // Calculate cache hit, and find LRU entry in case we need to evict
        assign HitIdxOneHot_1[i] = (Addr_1.Tag == LineTag_1[i]) & (LineValid_1[i]);
        assign LRUIdxOneHot_1[i] = ~(|LineAge_1[i]) & (LineValid_1[i]);
    end

    logic InvalidEntryExists;
    logic [SET_SIZE-1:0] LineInvalid_1;

    assign LineInvalid_1 = ~LineValid_1;
    assign InvalidEntryExists = |LineInvalid_1;

    // Prioritize allocating any invalid entry
    // If there isn't one; evict the LRU entry
    assign ReplIdxOneHot_1 = InvalidEntryExists ? LineInvalid_1 : LRUIdxOneHot_1;

    always_comb begin
        HitIdx_1 = '0;
        ReplIdx_1 = '0;
        HitLineInfo_1 = '0;
        ReplLineInfo_1 = '0;
        for (int i=0; i<SET_SIZE; i++) begin

            // Select line you hit you hit
            HitIdx_1 |= HitIdxOneHot_1[i] ? SET_PTR_BITS'(i) : '0;
            HitLineInfo_1 |= HitIdxOneHot_1[i] ? CacheLineInfo_1[i] : '0;

            // Select line you want to allocate
            ReplIdx_1 |= ReplIdxOneHot_1[i] ? SET_PTR_BITS'(i) : '0;
            ReplLineInfo_1 |= ReplIdxOneHot_1[i] CacheLineInfo_1[i] : '0;
        end
    end   

    assign Hit_1 = |HitIdxOneHot_1;
    assign MRUIdx_1 = Hit_1 ? HitIdx_1 : ReplIdx_1;

    // Concatenate flat cache read address
    // Note this will read the correct cache line
    // whether it was a hit or miss
    assign CacheLineAddr = {Addr_1.Idx, MRUIdx_1};

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

    // Output flip-flops

    CacheAddr_t WrBackAddrD;

    assign WrBackAddrD.Tag = ReplLineInfo_1.Tag;
    assign WrBackAddrD.Idx = MRUIdx_1;
    assign WrBackAddrD.Offs = OFFS_BITS'(0);

    dff #(ADDR_BITS)   I_WrBackAddr    (.Q(WrBackAddr),    .D(WrBackAddrD),     .Clk(Clk), .En(1'b1));
    dff #(LINE_BITS)   I_WrBackData    (.Q(WrBackData),    .D(WrBackDataD),     .Clk(Clk), .En(1'b1));
    dff #(LINE_BITS)   I_DataOut       (.Q(DataOut),       .D(DataOutD),        .Clk(Clk), .En(1'b1));
    dff #(1)           I_RdHitDataVal  (.Q(RdHitDataVal),  .D(RdHitDataValD),   .Clk(Clk), .En(1'b1));
    dff #(1)           I_Hit           (.Q(Hit),           .D(Hit_1),           .Clk(Clk), .En(1'b1));
    dff #(1)           I_HitVal        (.Q(HitVal),        .D(HitValD),         .Clk(Clk), .En(1'b1));

endmodule

`endif // SET_ASSOCIATIVE_CACHE_SV
