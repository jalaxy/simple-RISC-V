module stats(
    input  logic        clk,
    input  logic        rst,
    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    output logic        icache_flsh,
    input  logic        icache_done,
    input  logic [31:0] icache_data,
    output logic [`lgCQSZ:0] dcache_rqst,
    output logic             dcache_wena,
    output logic      [63:0] dcache_addr,
    output logic       [2:0] dcache_bits,
    input  logic [`lgCQSZ:0] dcache_done,
    input  logic      [63:0] dcache_rdat,
    output logic      [63:0] dcache_wdat,
    // stats
    output logic [63:0] cmtpc[1:0],
    output logic  [6:0] cmtaddr[1:0],
    output logic [63:0] cmtdata[1:0],
    output logic [63:0] arregs[63:0]
);
    pipeline pipeline_inst(clk, rst,
        icache_rqst, icache_addr, icache_flsh, icache_done, icache_data,
        dcache_rqst, dcache_wena, dcache_addr, dcache_bits, dcache_done,
        dcache_rdat, dcache_wdat);
    always_comb {cmtpc[0], cmtaddr[0], cmtdata[0]} =
        pipeline_inst.wb_stage_inst.cqpop1 ?
            {pipeline_inst.wb_stage_inst.cqpc,
             pipeline_inst.wb_stage_inst.cqrda,
             pipeline_inst.wb_stage_inst.cqfrontval[63:0]} : 0;
    always_comb {cmtpc[1], cmtaddr[1], cmtdata[1]} =
        pipeline_inst.wb_stage_inst.cqpop2 ?
            {pipeline_inst.wb_stage_inst.cqpcp1,
             pipeline_inst.wb_stage_inst.cqrdap1,
             pipeline_inst.wb_stage_inst.cqfrontp1val[63:0]} : 0;
    /*verilator tracing_off*/ logic [63:0] dupregs[1:0][63:0]; /*verilator tracing_on*/
    for (genvar i = 0; i < 2; i++) for (genvar j = 0; j < 64; j++)
        assign dupregs[i][j] = pipeline_inst.wb_stage_inst.regs_inst.dupregs[i].regs[j];
    always_comb for (int i = 0; i < 64; i++)
        arregs[i] = dupregs[pipeline_inst.wb_stage_inst.regs_inst.sel[i]][i];
endmodule