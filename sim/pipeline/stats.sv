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
    output logic [63:0] arregs[63:0],
    output logic [31:0] cqocc,
    output logic [31:0] ptocc,
    output logic [31:0] lsqocc,
    output logic ifetch
);
    // instantiate
    pipeline pipeline_inst(clk, rst,
        icache_rqst, icache_addr, icache_flsh, icache_done, icache_data,
        dcache_rqst, dcache_wena, dcache_addr, dcache_bits, dcache_done,
        dcache_rdat, dcache_wdat);

    // monitor registers change
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

    // extract architectural registers from instance
    /*verilator tracing_off*/ logic [63:0] dupregs[1:0][63:0]; /*verilator tracing_on*/
    for (genvar i = 0; i < 2; i++) for (genvar j = 0; j < 64; j++)
        assign dupregs[i][j] = pipeline_inst.wb_stage_inst.regs_inst.dupregs[i].regs[j];
    always_comb for (int i = 0; i < 64; i++)
        arregs[i] = dupregs[pipeline_inst.wb_stage_inst.regs_inst.sel[i]][i];

    // record occupancy of buffers
    always_comb begin
        cqocc = {{32-`lgCQSZ{1'b0}}, pipeline_inst.wb_stage_inst.cqrear} +
            `CQSZ - {{32-`lgCQSZ{1'b0}}, pipeline_inst.wb_stage_inst.cqfront};
        if (cqocc > `CQSZ | pipeline_inst.wb_stage_inst.cqempty) cqocc -= `CQSZ;
        lsqocc = {{32-`lgLSQSZ{1'b0}}, pipeline_inst.lsu_inst.rear }+
            `LSQSZ - {{32-`lgLSQSZ{1'b0}}, pipeline_inst.lsu_inst.front};
        if (lsqocc > `LSQSZ | pipeline_inst.lsu_inst.empty) lsqocc -= `LSQSZ;
        ptocc = 0;
        for (int i = 0; i < `PTSZ; i++)
            if (pipeline_inst.pending_table_inst.id[i][`lgCQSZ]) ptocc++;
    end

    always_comb ifetch = pipeline_inst.id_stage_inst.get_if &
        pipeline_inst.data_if_id.valid;
endmodule

module mul(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic [`lgCQSZ:0] rqst, input logic [4:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
`define mullatency 10
    logic [`mullatency-1:0][63:0] r_q;
    logic [`mullatency-1:0][`lgCQSZ:0] valid;
    logic [127:0] res, aext, bext;
    logic [63:0] rext;
    // op: 0 -> MUL  1 -> MULH  2 -> MULHSU  3 -> MULHU  4 -> MULW
    always_comb begin
        aext = {128{op[0] | op[1] | op[2] | op[4]}} & {{64{a[63]}}, a} |
               {128{op[3]}} & {64'd0, a};
        bext = {128{op[0] | op[1] | op[4]}} & {{64{b[63]}}, b} |
               {128{op[2] | op[3]}} & {64'd0, b};
        res = aext * bext;
        rext = {64{op[0]}} & res[63:0] | {64{op[4]}} & {{32{res[31]}}, res[31:0]} |
               {64{op[1] | op[2] | op[3]}} & res[127:64];
    end
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`mullatency'd0}};
        else if (ena | ~valid[0][`lgCQSZ]) begin
            valid <= {rqst, valid[`mullatency-1:1]};
            r_q <= {rext, r_q[`mullatency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
    always_comb get = ena | ~valid[0][`lgCQSZ];
endmodule

module div(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic [`lgCQSZ:0] rqst, input logic [7:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);

// should be un-pipelined

`define divlatency 20
    logic [`divlatency-1:0][63:0] r_q;
    logic [`divlatency-1:0][`lgCQSZ:0] valid;
    logic [63:0] res, aabs, babs, rabs, qabs;
    logic [31:0] res32;
    // op: 0 -> DIV   1 -> DIVU   2 -> REM   3 -> REMU
    //     4 -> DIVW  5 -> DIVUW  6 -> REMW  7 -> REMUW
    always_comb res32 = {32{op[4]}} & $signed($signed(a[31:0]) / $signed(b[31:0])) |
                        {32{op[5]}} & (a[31:0] / b[31:0]) |
                        {32{op[6]}} & $signed($signed(a[31:0]) % $signed(b[31:0])) |
                        {32{op[7]}} & (a[31:0] % b[31:0]);
    always_comb res = {64{op[0]}} & $signed($signed(a) / $signed(b)) |
                      {64{op[1]}} & (a / b) |
                      {64{op[2]}} & $signed($signed(a) % $signed(b)) |
                      {64{op[3]}} & (a % b) |
                      {64{|op[7:4]}} & {{32{res32[31]}}, res32};
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`divlatency'd0}};
        else if (ena | ~valid[0][`lgCQSZ]) begin
            valid <= {rqst, valid[`divlatency-1:1]};
            r_q <= {res, r_q[`divlatency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
    always_comb get = ena | ~valid[0][`lgCQSZ];
endmodule

module regfile #(parameter dwidth = 64,
    parameter rports = 2, parameter wports = 1,
    parameter awidth = 6, parameter depth = 64) (
    input  logic clk, input logic rst,
    input  logic [rports-1:0][awidth-1:0] raddr,
    output logic [rports-1:0][dwidth-1:0] rvalue,
    input  logic [wports-1:0][awidth-1:0] waddr,
    input  logic [wports-1:0][dwidth-1:0] wvalue,
    input  logic [wports-1:0] wena
);
    int sel[depth-1:0];
    always_ff @(posedge clk)
        for (int i = 0; i < wports; i++) if (wena[i]) sel[waddr[i]] <= i;
    for (genvar i = 0; i < wports; i++) begin : dupregs
        logic [dwidth-1:0] regs[depth-1:0];
        always_ff @(posedge clk)
            if (wena[i]) regs[waddr[i]] <= wvalue[i];
    end
    for (genvar i = 0; i < rports; i++) begin
        logic [dwidth-1:0] dupval[wports-1:0];
        for (genvar j = 0; j < wports; j++)
            always_comb dupval[j] = dupregs[j].regs[raddr[i]];
        always_comb rvalue[i] = dupval[sel[raddr[i]]];
    end
endmodule
