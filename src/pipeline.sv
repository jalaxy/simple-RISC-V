`define RST_PC 64'h400000 // reset pc
`define PTSZ 8 // pending table size
`define lgPTSZ 3
`define PTLEN 460
`define CQSZ 16 // commit queue size
`define lgCQSZ 4
`define LSQSZ 8 // store queue size
`define lgLSQSZ 3
`define LATENUM 5 // number of late components
`define LOAD      5'b00000 // opcode map
`define LOAD_FP   5'b00001
`define MISC_MEM  5'b00011
`define OP_IMM    5'b00100
`define AUIPC     5'b00101
`define OP_IMM_32 5'b00110
`define STORE     5'b01000
`define STORE_FP  5'b01001
`define AMO       5'b01011
`define OP        5'b01100
`define LUI       5'b01101
`define OP_32     5'b01110
`define MADD      5'b10000
`define MSUB      5'b10001
`define NMSUB     5'b10010
`define NMADD     5'b10011
`define OP_FP     5'b10100
`define BRANCH    5'b11000
`define JALR      5'b11001
`define JAL       5'b11011
`define SYSTEM    5'b11100
`define EX_ADD    6'd0 // execution stage operations
`define EX_SUB    6'd1
`define EX_SLL    6'd2
`define EX_SLT    6'd3
`define EX_SLTU   6'd4
`define EX_XOR    6'd5
`define EX_SRL    6'd6
`define EX_SRA    6'd7
`define EX_OR     6'd8
`define EX_AND    6'd9
`define EX_MIN    6'd10
`define EX_MAX    6'd11
`define EX_MUL    6'd12
`define EX_MULH   6'd13
`define EX_MULHSU 6'd14
`define EX_MULHU  6'd15
`define EX_DIV    6'd16
`define EX_DIVU   6'd17
`define EX_REM    6'd18
`define EX_REMU   6'd19
`define EX_FADD   6'd20
`define EX_FSUB   6'd21
`define EX_FMUL   6'd22
`define EX_FNMUL  6'd23
`define EX_FDIV   6'd24
`define EX_FSQRT  6'd25
`define EX_FSGNJ  6'd26
`define EX_FSGNJN 6'd27
`define EX_FSGNJX 6'd28
`define EX_FMIN   6'd29
`define EX_FMAX   6'd30
`define EX_FEQ    6'd31
`define EX_FLT    6'd32
`define EX_FLE    6'd33
`define EX_FMVXF  6'd34
`define EX_FCLASS 6'd35
`define EX_FMVFX  6'd36
`define EX_FCVTIF 6'd37
`define EX_FCVTFI 6'd38
`define EX_FCVTSD 6'd39
`define EX_FCVTDS 6'd40
`define EX_LOAD   6'd41
`define EX_STORE  6'd42
`define EX_FENCE  6'd43
`define EX_FENCEI 6'd44
`define EX_CSR    6'd45
`define EX_ECALL  6'd46
`define EX_EBREAK 6'd47
`define EX_END    6'd48

typedef struct packed { logic valid, b; logic [63:0] pc, bpc; } pc_if_t;
typedef struct packed { logic valid, c; } if_pc_t;
typedef struct packed { logic valid; logic [63:0] pc, npc; } ex_pc_t;
typedef struct packed {
    logic valid, b, c;
    logic [63:0] pc, bpc;
    logic [31:0] ir;
} if_id_t;
typedef struct packed {
    logic valid, branch, c;
    logic [63:0] pc, bpc;
    logic [`EX_END-1:0] exop;
    logic iword, isign;
    logic [2:0] funct3, bmask;
    logic fdouble, bneg, j;
    logic [64:0] base;
    logic [63:0] offset;
    logic [1:0] rsrv, aqrl;
    logic [64:0] a, b;
    logic [6:0] rda;
} id_ex_t;
typedef struct packed {
    logic valid;
    logic mw;
    logic [63:0] pc;
    logic [64:0] rd;
    logic [6:0] rda;
} ex_wb_t;

module pipeline(
    input  logic        clk,
    input  logic        rst,

    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    output logic        icache_flsh,
    input  logic        icache_done,
    input  logic [31:0] icache_data,

    output logic [`lgCQSZ:0] dcache_rqst,
    output logic       [1:0] dcache_rsrv,
    output logic             dcache_wena,
    output logic      [63:0] dcache_addr,
    output logic       [2:0] dcache_bits,
    input  logic [`lgCQSZ:0] dcache_done,
    input  logic      [63:0] dcache_rdat,
    output logic      [63:0] dcache_wdat,
    output logic             dcache_flsh
);
    if_pc_t data_if_pc; logic get_if_pc;
    pc_if_t data_pc_if; logic get_pc_if;
    if_id_t data_if_id; logic get_if_id;
    id_ex_t data_id_ex; logic get_id_ex;
    ex_wb_t data_ex_wb; logic get_ex_wb;
    ex_pc_t data_ex_pc; logic get_ex_pc;
    id_ex_t data_ex_pt; logic get_ex_pt;
    id_ex_t data_pt_ex; logic get_pt_ex;
    logic [1:0][6:0] raddr; logic [1:0][64:0] rvalue;
    logic [`lgCQSZ:0] cqid_new, cqid_old;
    logic [`lgCQSZ:0] late_done; logic [64:0] late_val; logic late_exc;
    logic [`lgCQSZ:0] pt_done; logic [64:0] pt_data; logic pt_exc, pt_ena;
    logic [`lgCQSZ:0] addr_done; logic [63:0] addr_val;
    logic [`lgCQSZ:0] lsu_rqst, lsu_done; logic lsu_exc, lsu_ena, lsu_free;
    logic lsu_wena, lsu_csr; logic [2:0] lsu_bits;
    logic [11:0] lsu_fence; logic [1:0] lsu_rsrv; logic [1:0] lsu_aqrl;
    logic [64:0] lsu_addr; logic [64:0] lsu_rdat, lsu_wdat;
    logic [`lgCQSZ:0] mul_rqst, mul_done; logic mul_exc, mul_ena, mul_free;
    logic [4:0] mul_op; logic [63:0] mul_a, mul_b; logic [64:0] mul_r;
    logic [`lgCQSZ:0] div_rqst, div_done; logic div_exc, div_ena, div_free;
    logic [7:0] div_op; logic [63:0] div_a, div_b; logic [64:0] div_r;
    logic [`lgCQSZ:0] fpu_rqst, fpu_done; logic fpu_exc, fpu_ena, fpu_free;
    logic [20:0] fpu_op; logic [63:0] fpu_a, fpu_b;
    logic [2:0] fpu_rm; logic fpu_double; logic [64:0] fpu_r;
    logic [11:0] csr_addr; logic csr_wena; logic [2:0] csr_func;
    logic [63:0] csr_rval, csr_wval; logic csr_excp;

    pc_stage pc_stage_inst(.clk(clk), .rst(rst), .flush(data_ex_pc.valid),
        .in_if(data_if_pc), .get_if(get_if_pc),
        .in_ex(data_ex_pc), .get_ex(get_ex_pc),
        .out_if(data_pc_if), .ena_if(get_pc_if));
    if_stage if_stage_inst(.clk(clk), .rst(rst), .flush(data_ex_pc.valid),
        .in_pc(data_pc_if), .get_pc(get_pc_if),
        .out_pc(data_if_pc), .ena_pc(get_if_pc),
        .out_id(data_if_id), .ena_id(get_if_id),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_flsh(icache_flsh),
        .icache_done(icache_done), .icache_data(icache_data));
    id_stage id_stage_inst(.clk(clk), .rst(rst), .flush(data_ex_pc.valid),
        .in_if(data_if_id), .get_if(get_if_id),
        .out_ex(data_id_ex), .ena_ex(get_id_ex & ~|wb_stage_inst.cqexc),
        .raddr(raddr));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst),
        .in_id(data_id_ex), .get_id(get_id_ex),
        .in_pt(data_pt_ex), .get_pt(get_pt_ex),
        .out_wb(data_ex_wb), .ena_wb(get_ex_wb),
        .out_pc(data_ex_pc), .ena_pc(get_ex_pc),
        .out_pt(data_ex_pt), .ena_pt(get_ex_pt),
        .rvalue(rvalue), .cqid_id(cqid_new), .cqid_pt(cqid_old),
        .mul_free(mul_free), .mul_rqst(mul_rqst),
        .mul_op(mul_op), .mul_a(mul_a), .mul_b(mul_b),
        .div_free(div_free), .div_rqst(div_rqst),
        .div_op(div_op), .div_a(div_a), .div_b(div_b),
        .fpu_free(fpu_free), .fpu_rqst(fpu_rqst),
        .fpu_op(fpu_op), .fpu_a(fpu_a), .fpu_b(fpu_b),
        .fpu_rm(fpu_rm), .fpu_double(fpu_double),
        .lsu_fence(lsu_fence), .lsu_empty(lsu_inst.empty),
        .lsu_rsrv(lsu_rsrv), .lsu_aqrl(lsu_aqrl), .lsu_csr(lsu_csr),
        .lsu_free(lsu_free), .lsu_rqst(lsu_rqst), .lsu_wena(lsu_wena),
        .lsu_addr(lsu_addr), .lsu_bits(lsu_bits), .lsu_wdat(lsu_wdat),
        .late_done(late_done), .late_val(late_val),
        .pt_done(pt_done), .pt_data(pt_data), .pt_exc(pt_exc), .ena_arb(pt_ena),
        .addr_done(addr_done), .addr_val(addr_val), .tvec(csr_inst.tvec));
    wb_stage wb_stage_inst(.clk(clk), .rst(rst),
        .in_ex(data_ex_wb), .get_ex(get_ex_wb), .ena_ex(get_id_ex),
        .raddr(raddr), .rvalue(rvalue), .cqid(cqid_new),
        .late_done(late_done), .late_val(late_val), .late_exc(late_exc));
    pending_table pending_table_inst(.clk(clk), .rst(rst),
        .flush(wb_stage_inst.recover),
        .in_ex(data_ex_pt), .get_ex(get_ex_pt),
        .out_ex(data_pt_ex), .ena_ex(get_pt_ex),
        .cqid_in(cqid_new), .cqid_out(cqid_old),
        .late_done(late_done), .late_val(late_val),
        .rda_id(data_id_ex.valid ? data_id_ex.rda : 0),
        .rda_ex(data_ex_wb.valid ? data_ex_wb.rda : 0));
    lsu lsu_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover),
        .ena(lsu_ena), .get(lsu_free),
        .cmt(wb_stage_inst.lsu_cmt), .cmtp1(wb_stage_inst.lsu_cmtp1),
        .fence(lsu_fence), .rsrv(lsu_rsrv), .aqrl(lsu_aqrl), .csr(lsu_csr),
        .rqst(lsu_rqst), .wena(lsu_wena), .addr(lsu_addr), .bits(lsu_bits),
        .done(lsu_done), .excp(lsu_exc), .rdata(lsu_rdat), .wdata(lsu_wdat),
        .late_done(late_done), .late_val(late_val),
        .addr_done(addr_done), .addr_val(addr_val),
        .csr_addr(csr_addr), .csr_wena(csr_wena), .csr_func(csr_func),
        .csr_rval(csr_rval), .csr_wval(csr_wval), .csr_excp(csr_excp),
        .dcache_rqst(dcache_rqst), .dcache_rsrv(dcache_rsrv), .dcache_wena(dcache_wena),
        .dcache_addr(dcache_addr), .dcache_bits(dcache_bits), .dcache_done(dcache_done),
        .dcache_rdat(dcache_rdat), .dcache_wdat(dcache_wdat), .dcache_flsh(dcache_flsh)
    );
    mul mul_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover),
        .ena(mul_ena), .get(mul_free),
        .rqst(mul_rqst), .op(mul_op), .a(mul_a), .b(mul_b),
        .done(mul_done), .r(mul_r), .e(mul_exc));
    div div_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover),
        .ena(div_ena), .get(div_free),
        .rqst(div_rqst), .op(div_op), .a(div_a), .b(div_b),
        .done(div_done), .r(div_r), .e(div_exc));
    fpu fpu_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover),
        .ena(fpu_ena), .get(fpu_free),
        .rqst(fpu_rqst), .op(fpu_op), .a(fpu_a), .b(fpu_b),
        .rm(fpu_rm), .double(fpu_double),
        .done(fpu_done), .r(fpu_r), .e(fpu_exc));
    csr csr_inst(.clk(clk), .rst(rst), .addr(csr_addr), .wena(csr_wena),
        .rval(csr_rval), .wval(csr_wval), .func(csr_func), .excp(csr_excp),
        .nret(wb_stage_inst.cqpop2 ? 2 : (wb_stage_inst.cqpop1 ? 1 : 0)));
    arbiter arbiter_inst( // PT should have lowest priority to avoid deadlock,
                          // or use dynamic priority?
        .done_in({pt_done, fpu_done, div_done, mul_done, lsu_done}),
        .val_in({pt_data, fpu_r, div_r, mul_r, lsu_rdat}),
        .exc_in({pt_exc, fpu_exc, div_exc, mul_exc, lsu_exc}),
        .get({pt_ena, fpu_ena, div_ena, mul_ena, lsu_ena}),
        .done_out(late_done), .val_out(late_val), .exc_out(late_exc));
endmodule

module pc_stage(input logic clk, input logic rst, input logic flush,
    input  if_pc_t in_if, output  logic get_if,
    input  ex_pc_t in_ex, output  logic get_ex,
    output pc_if_t out_if, input  logic ena_if
);
    logic [63:0] pc, pc2, pc4, bpc;
    logic b;
    always_comb out_if.valid = ~flush;
    always_comb out_if.pc = pc;
    always_comb out_if.b = b;
    always_comb out_if.bpc = bpc;
    always_comb pc = in_if.valid & in_if.c ? pc2 : pc4;
    always_ff @(posedge clk)
        if (rst) {pc2, pc4} <= {`RST_PC, `RST_PC};
        else if (in_ex.valid) {pc2, pc4} <= {in_ex.npc, in_ex.npc};
        else if (ena_if) {pc2, pc4} <= b ? {bpc, bpc} : {pc + 64'd2, pc + 64'd4};
    always_comb get_if = 1'b1;
    always_comb get_ex = 1'b1;
    // branch predictor
    always_comb b = 0;
    always_comb bpc = 0;
endmodule

module if_stage(input logic clk, input logic rst, input logic flush,
    input  pc_if_t in_pc,  output logic get_pc,
    output if_pc_t out_pc, input  logic ena_pc, // always enabled
    output if_id_t out_id, input  logic ena_id,
    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    output logic        icache_flsh,
    input  logic        icache_done,
    input  logic [31:0] icache_data
);
    pc_if_t in_pc_r;
    logic sent, hold_data;
    always_comb get_pc = ~in_pc.valid | ena_id & ~(sent & ~icache_done);
    always_comb icache_rqst = ~rst & get_pc & in_pc.valid;
    always_ff @(posedge clk)
        sent <= rst | flush ? 1'b0 : icache_rqst | sent & ~icache_done;
    always_ff @(posedge clk)
        hold_data <= rst | flush ? 1'b0 : (ena_id ? 1'b0 : icache_done | hold_data);
    always_comb icache_addr = in_pc.pc;
    always_comb icache_flsh = flush;
    always_comb out_id.valid = ~flush & (sent & icache_done | hold_data);
    always_comb out_id.b = in_pc_r.b;
    always_comb out_id.c = icache_data[1:0] != 2'b11;
    always_comb out_id.pc = in_pc_r.pc;
    always_comb out_id.bpc = in_pc_r.bpc;
    always_comb out_id.ir = icache_data;
    always_comb out_pc.valid = ~flush & (sent & icache_done | hold_data);
    always_comb out_pc.c = icache_data[1:0] != 2'b11;
    always_ff @(posedge clk) in_pc_r <= icache_rqst ? in_pc : in_pc_r;
endmodule

module id_stage(input logic clk, input logic rst, input logic flush,
    input  if_id_t in_if, output logic get_if,
    output id_ex_t out_ex, input logic ena_ex,
    output logic [1:0][6:0] raddr
);
    logic [31:0] ir, op;
    logic [63:0] imm;
    id_ex_t [2:0] out_ex_q;
    logic [`EX_END-1:0] exop0, exop1, exop2;
    logic [64:0] a0, b0;
    ci2i ci2i_inst(.ci(in_if.ir), .i(ir));
    always_comb for (int i = 0; i < 32; i++)
        op[i] = ir[6:2] == i[4:0];
    always_comb imm =
        {{53{ir[31]}}, ir[30:20]} & {64{
            op[`LOAD] | op[`LOAD_FP] | op[`MISC_MEM] | op[`OP_IMM] |
            op[`OP_IMM_32] | op[`JALR] | op[`SYSTEM]}} | // I type
        {{32{ir[31]}}, ir[31:12], 12'd0} & {64{
            op[`AUIPC] | op[`LUI]}} | // U type
        {{53{ir[31]}}, ir[30:25], ir[11:7]} & {64{
            op[`STORE] | op[`STORE_FP]}} | // S type
        {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} & {64{op[`BRANCH]}} | // B type
        {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} & {64{op[`JAL]}}; // J type
    always_comb begin
        exop0 = 0;
        exop0[`EX_ADD] =
            op[`JALR] | op[`AUIPC] | op[`LUI] | op[`JAL] |
            (op[`OP_IMM] | op[`OP_IMM_32]) & ir[14:12] == 3'b000 |
            (op[`OP] | op[`OP_32]) & ir[14:12] == 3'b000 & ir[31:25] == 7'd0;
        exop0[`EX_SUB] = op[`BRANCH] |
            (op[`OP] | op[`OP_32]) & ir[14:12] == 3'b000 & ir[31:25] == 7'b0100000;
        exop0[`EX_SLL] =
            op[`OP_IMM] & ir[14:12] == 3'b001 & ir[31:26] == 6'd0 |
            (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                ir[14:12] == 3'b001 & ir[31:25] == 7'd0;
        exop0[`EX_SLT] =
            op[`OP_IMM] & ir[14:12] == 3'b010 |
            op[`OP] & ir[14:12] == 3'b010 & ir[31:25] == 7'd0;
        exop0[`EX_SLTU] =
            op[`OP_IMM] & ir[14:12] == 3'b011 |
            op[`OP] & ir[14:12] == 3'b011 & ir[31:25] == 7'd0;
        exop0[`EX_XOR] =
            op[`OP_IMM] & ir[14:12] == 3'b100 |
            op[`OP] & ir[14:12] == 3'b100 & ir[31:25] == 7'd0;
        exop0[`EX_SRL] =
            op[`OP_IMM] & ir[14:12] == 3'b101 & ir[31:26] == 6'd0 |
            (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                ir[14:12] == 3'b101 & ir[31:25] == 7'd0;
        exop0[`EX_SRA] =
            op[`OP_IMM] & ir[14:12] == 3'b101 & ir[31:26] == 6'b010000 |
            (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                ir[14:12] == 3'b101 & ir[31:25] == 7'b0100000;
        exop0[`EX_OR] =
            op[`OP_IMM] & ir[14:12] == 3'b110 |
            op[`OP] & ir[14:12] == 3'b110 & ir[31:25] == 7'd0;
        exop0[`EX_AND] =
            op[`OP_IMM] & ir[14:12] == 3'b111 |
            op[`OP] & ir[14:12] == 3'b111 & ir[31:25] == 7'd0;
        exop0[`EX_MUL] = (op[`OP] | op[`OP_32]) &
            ir[14:12] == 3'b000 & ir[31:25] == 7'b1;
        exop0[`EX_MULH] = op[`OP] & ir[14:12] == 3'b001 & ir[31:25] == 7'b1;
        exop0[`EX_MULHSU] = op[`OP] & ir[14:12] == 3'b010 & ir[31:25] == 7'b1;
        exop0[`EX_MULHU] = op[`OP] & ir[14:12] == 3'b011 & ir[31:25] == 7'b1;
        exop0[`EX_DIV] = (op[`OP] | op[`OP_32]) &
            ir[14:12] == 3'b100 & ir[31:25] == 7'b1;
        exop0[`EX_DIVU] = (op[`OP] | op[`OP_32]) &
            ir[14:12] == 3'b101 & ir[31:25] == 7'b1;
        exop0[`EX_REM] = (op[`OP] | op[`OP_32]) &
            ir[14:12] == 3'b110 & ir[31:25] == 7'b1;
        exop0[`EX_REMU] = (op[`OP] | op[`OP_32]) &
            ir[14:12] == 3'b111 & ir[31:25] == 7'b1;
        exop0[`EX_FADD] = op[`OP_FP] & ir[31:26] == 6'b000000;
        exop0[`EX_FSUB] = op[`OP_FP] & ir[31:26] == 6'b000010;
        exop0[`EX_FMUL] = op[`OP_FP] & ir[31:26] == 6'b000100 | op[`MADD] | op[`MSUB];
        exop0[`EX_FNMUL] = op[`NMADD] | op[`NMSUB];
        exop0[`EX_FDIV] = op[`OP_FP] & ir[31:26] == 6'b000110;
        exop0[`EX_FSQRT] = op[`OP_FP] & ir[31:26] == 6'b010110 & ir[24:20] == 5'd0;
        exop0[`EX_FSGNJ] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b000;
        exop0[`EX_FSGNJN] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b001;
        exop0[`EX_FSGNJX] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b010;
        exop0[`EX_FMIN] = op[`OP_FP] & ir[31:26] == 6'b001010 & ir[14:12] == 3'b000;
        exop0[`EX_FMAX] = op[`OP_FP] & ir[31:26] == 6'b001010 & ir[14:12] == 3'b001;
        exop0[`EX_FEQ] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b010;
        exop0[`EX_FLT] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b001;
        exop0[`EX_FLE] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b000;
        exop0[`EX_FMVXF] = op[`OP_FP] & ir[31:26] == 6'b111000 &
            ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
        exop0[`EX_FCLASS] = op[`OP_FP] & ir[31:26] == 6'b111000 &
            ir[14:12] == 3'b001 & ir[24:20] == 5'd0;
        exop0[`EX_FMVFX] = op[`OP_FP] & ir[31:26] == 6'b111100 &
            ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
        exop0[`EX_FCVTIF] = op[`OP_FP] & ir[31:26] == 6'b110000;
        exop0[`EX_FCVTFI] = op[`OP_FP] & ir[31:26] == 6'b110100;
        exop0[`EX_FCVTSD] = op[`OP_FP] & ir[31:25] == 7'b0100000 & ir[24:20] == 5'd1;
        exop0[`EX_FCVTDS] = op[`OP_FP] & ir[31:25] == 7'b0100001 & ir[24:20] == 5'd0;
        exop0[`EX_LOAD] = op[`LOAD] | op[`LOAD_FP] |
            op[`AMO] & (ir[31:27] == 5'b00010 | |exop1);
        exop0[`EX_STORE] = op[`STORE] | op[`STORE_FP] |
            op[`AMO] & ir[31:27] == 5'b00011;
        exop0[`EX_FENCE] = op[`MISC_MEM] & ir[14:12] == 3'b000;
        exop0[`EX_FENCEI] = op[`MISC_MEM] & ir[14:12] == 3'b001;
        exop0[`EX_CSR] = op[`SYSTEM] & |ir[13:12];
        exop0[`EX_ECALL] = ir == 32'h00000073;
        exop0[`EX_EBREAK] = ir == 32'h00100073;
        if (ir[1:0] != 2'b11) exop0 = 0;
    end
    always_comb exop1 =
        (1 << `EX_ADD)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00001}} |
        (1 << `EX_ADD)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00000}} |
        (1 << `EX_XOR)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00100}} |
        (1 << `EX_AND)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b01100}} |
        (1 << `EX_OR)   & {`EX_END{op[`AMO] & ir[31:27] == 5'b01000}} |
        (1 << `EX_MIN)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b10000}} |
        (1 << `EX_MIN)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b11000}} |
        (1 << `EX_MAX)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b10100}} |
        (1 << `EX_MAX)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b11100}} |
        (1 << `EX_FADD) & {`EX_END{op[`MADD] | op[`NMSUB]}} |
        (1 << `EX_FSUB) & {`EX_END{op[`MSUB] | op[`NMADD]}};
    always_comb exop2 = (1 << `EX_STORE) & {`EX_END{op[`AMO] & |exop1}};
    always_comb if (exop0[`EX_FCVTFI] | exop0[`EX_FMVFX])
            a0 = {1'd1, 59'd0, ir[19:15]};
        else a0 = {~(op[`SYSTEM] & ir[14]), 59'd0, ir[19:15]} & {65{
                      op[`LOAD]   | op[`LOAD_FP]  | op[`OP_IMM] | op[`OP_IMM_32] |
                      op[`STORE]  | op[`STORE_FP] | op[`OP]     | op[`OP_32]     |
                      op[`BRANCH] | op[`AMO]      | op[`SYSTEM]}} |
                  {1'd1, 59'd1, ir[19:15]} & {{65{
                      op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                      op[`OP_FP]}}} |
                  {1'd0, in_if.pc} & {65{op[`JALR] | op[`JAL] | op[`AUIPC]}};
    always_comb if (exop0[`EX_FCVTIF] | exop0[`EX_FCVTFI])
            b0 = {60'd0, ir[24:20]};
        else if (exop0[`EX_FSQRT] | exop0[`EX_FCVTDS] | exop0[`EX_FCVTSD] |
            exop0[`EX_FMVFX] | exop0[`EX_FMVXF])
            b0 = 65'd0;
        else b0 = {1'd1, 59'd0, ir[24:20]} & {65{
                      op[`OP] | op[`OP_32] | op[`OP_FP] | op[`BRANCH]}} |
                  {1'd1, 59'd1, ir[24:20]} & {{65{
                      op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                      op[`OP_FP]}}} |
                  {1'd0, imm} & {65{              op[`SYSTEM] | op[`MISC_MEM]  |
                      op[`LOAD]  | op[`LOAD_FP] | op[`OP_IMM] | op[`OP_IMM_32] |
                      op[`AUIPC] | op[`LUI]     | op[`STORE]  | op[`STORE_FP]}} |
                  (in_if.c ? 65'd2 : 65'd4) & {65{op[`JAL] | op[`JALR]}};
    always_comb get_if = ena_ex & ~out_ex_q[1].valid | ~in_if.valid;
    always_comb begin out_ex = out_ex_q[0]; out_ex.valid = out_ex.valid & ~flush; end
    always_ff @(posedge clk)
        if (rst | flush)
            {out_ex_q[0].valid, out_ex_q[1].valid, out_ex_q[2].valid} <= 0;
        else if (get_if & in_if.valid) begin
            out_ex_q[0].valid <= 1'b1;
            out_ex_q[0].branch <= in_if.b;
            out_ex_q[0].c <= in_if.c;
            out_ex_q[0].pc <= in_if.pc;
            out_ex_q[0].bpc <= in_if.bpc;
            out_ex_q[0].a <= a0;
            out_ex_q[0].b <= b0;
            out_ex_q[0].exop <= exop0;
            out_ex_q[0].bmask <= {3{op[`BRANCH]}} &
                {ir[14:13] == 2'b00, ir[14:13] == 2'b10, ir[14:13] == 2'b11};
            out_ex_q[0].bneg <= ir[12];
            out_ex_q[0].j <= op[`JAL] | op[`JALR];
            out_ex_q[0].base <= {1'b0, in_if.pc} & {65{op[`JAL] | op[`BRANCH]}}|
                                {1'b1, 59'd0, ir[19:15]} & {65{op[`JALR]}};
            out_ex_q[0].offset <= imm & {64{op[`JAL] | op[`JALR] | op[`BRANCH]}};
            out_ex_q[0].iword <= op[`OP_32] | op[`OP_IMM_32];
            out_ex_q[0].isign <= (op[`OP_32] | op[`OP_IMM_32]) & ir[30];
            out_ex_q[0].funct3 <= ir[14:12];
            out_ex_q[0].fdouble <= ir[25];
            out_ex_q[0].rsrv <= {op[`AMO] & |exop1, op[`AMO] & ~|exop1};
            out_ex_q[0].aqrl <= {op[`AMO] & ir[26], op[`AMO] & ir[25]};
            if (exop0[`EX_FEQ] | exop0[`EX_FLT] | exop0[`EX_FLE] |
                exop0[`EX_FMVXF] | exop0[`EX_FCLASS] | exop0[`EX_FCVTIF])
                out_ex_q[0].rda <= {2'd0, ir[11:7]};
            else out_ex_q[0].rda <=
                {2'd0, ir[11:7]} & {7{
                    op[`LOAD] | op[`OP_IMM] | op[`AUIPC] | op[`OP_IMM_32] |
                    op[`OP]   | op[`LUI]    | op[`OP_32] | op[`JALR]      |
                    op[`JAL]  | op[`SYSTEM] | op[`AMO] & ~|exop1}} |
                {2'd1, ir[11:7]} & {7{op[`LOAD_FP] | op[`OP_FP]}} |
                {2'd2, 5'd0} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                                  op[`AMO] & |exop1}};

            out_ex_q[1].valid <= |exop1 & (
                op[`AMO] | op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]);
            out_ex_q[1].branch <= in_if.b;
            out_ex_q[1].c <= in_if.c;
            out_ex_q[1].pc <= in_if.pc;
            out_ex_q[1].bpc <= in_if.bpc;
            out_ex_q[1].a <= {1'd1, 59'd2, 5'd0} & {65{
                ~(op[`AMO] & ir[31:27] == 5'b00001)}}; // AMOSWAP
            out_ex_q[1].b <=
                {1'd1, 59'd0, ir[24:20]} & {65{op[`AMO]}} |
                {1'd1, 59'd1, ir[31:27]} & {65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};
            out_ex_q[1].exop <= exop1;
            out_ex_q[1].iword <= op[`AMO] & ir[14:12] == 3'b010;
            out_ex_q[1].isign <= op[`AMO] & ~ir[30];
            out_ex_q[1].rsrv <= 0;
            out_ex_q[1].bmask <= 0;
            out_ex_q[1].j <= 0;
            out_ex_q[1].funct3 <= ir[14:12];
            out_ex_q[1].fdouble <= ir[25];
            out_ex_q[1].rda <=
                {2'd2, 5'd0} & {7{op[`AMO]}} |
                {2'd1, ir[11:7]} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};

            out_ex_q[2].valid <= |exop2 & op[`AMO];
            out_ex_q[2].branch <= in_if.b;
            out_ex_q[2].c <= in_if.c;
            out_ex_q[2].pc <= in_if.pc;
            out_ex_q[2].bpc <= in_if.bpc;
            out_ex_q[2].a <= {1'd1, 59'd0, ir[19:15]} & {65{op[`AMO]}};
            out_ex_q[2].b <= {1'd0, imm} & {65{op[`AMO]}};
            out_ex_q[2].exop <= exop2;
            out_ex_q[2].rsrv <= {op[`AMO] & |exop1, 1'b0};
            out_ex_q[2].bmask <= 0;
            out_ex_q[2].j <= 0;
            out_ex_q[2].funct3 <= ir[14:12];
            out_ex_q[2].rda <= {2'd0, ir[11:7]};
        end else if (ena_ex) begin
            out_ex_q[1:0] <= {out_ex_q[2], out_ex_q[1]};
            out_ex_q[2].valid <= 0;
        end
    always_comb if (out_ex_q[1].valid & out_ex_q[1].a[64])
            raddr[0] = out_ex_q[1].a[6:0];
        else if (a0[64]) raddr[0] = a0[6:0];
        else if (op[`JALR]) raddr[0] = {2'd0, ir[19:15]};
        else raddr[0] = 0;
    always_comb if (out_ex_q[1].valid & out_ex_q[1].b[64])
            raddr[1] = out_ex_q[1].b[6:0];
        else if (out_ex_q[1].valid & out_ex_q[1].exop[`EX_STORE])
            raddr[1] = {2'd2, 5'd0};
        else if (b0[64]) raddr[1] = b0[6:0];
        else if (op[`STORE]) raddr[1] = {2'b0, ir[24:20]};
        else if (op[`STORE_FP]) raddr[1] = {2'b1, ir[24:20]};
        else if (op[`AMO] & ir[31:27] == 5'b00011) raddr[1] = {2'b0, ir[24:20]};
        else raddr[1] = 0;
endmodule

module ex_stage(input logic clk, input logic rst,
    input id_ex_t in_id, output logic get_id,
    input id_ex_t in_pt, output logic get_pt,
    output ex_wb_t out_wb, input logic ena_wb,
    output ex_pc_t out_pc, input logic ena_pc, // always enabled
    output id_ex_t out_pt, input logic ena_pt,
    input logic [1:0][64:0] rvalue,
    input logic [`lgCQSZ:0] cqid_id, input logic [`lgCQSZ:0] cqid_pt,
    input logic mul_free, output logic [`lgCQSZ:0] mul_rqst, output logic [4:0] mul_op,
    output logic [63:0] mul_a, output logic [63:0] mul_b,
    input logic div_free, output logic [`lgCQSZ:0] div_rqst, output logic [7:0] div_op,
    output logic [63:0] div_a, output logic [63:0] div_b,
    input logic fpu_free, output logic [`lgCQSZ:0] fpu_rqst, output logic [20:0] fpu_op,
    output logic [63:0] fpu_a, output logic [63:0] fpu_b,
    output logic [2:0] fpu_rm, output logic fpu_double,
    input logic lsu_free, output logic [`lgCQSZ:0] lsu_rqst,
    output logic [11:0] lsu_fence, input logic lsu_empty,
    output logic [1:0] lsu_rsrv, output logic [1:0] lsu_aqrl, output logic lsu_csr,
    output logic lsu_wena, output logic [64:0] lsu_addr,
    output logic [2:0] lsu_bits, output logic [64:0] lsu_wdat,
    input logic [`lgCQSZ:0] late_done, input logic [64:0] late_val,
    output logic [`lgCQSZ:0] pt_done, output logic [64:0] pt_data,
    output logic pt_exc, input logic ena_arb,
    output logic [`lgCQSZ:0] addr_done, output logic [63:0] addr_val,
    input logic [63:0] tvec
);
    id_ex_t in;
    logic frompt;
    always_comb frompt = in_pt.valid &
        (ena_arb | in_pt.exop[`EX_LOAD] | in_pt.exop[`EX_STORE]);
    always_comb in = frompt ? in_pt : in_id;
    logic [`EX_END-1:0] op;
    logic [63:0] a, b;
    logic jump, excp;
    logic [63:0] jpc;
    logic [64:0] sub, res;
    logic [63:0] add, sll, srl, sra;
    logic lsu, fencei, fencei_r;
    logic [2:0] bflag;
    logic ready, mul_valid, div_valid, fpu_valid, lsu_valid;
    logic [`lgCQSZ:0] cqid;
    always_comb op = in.valid ? in.exop : 0;
    always_comb begin
        if (frompt & in_pt.j) a = in.pc; // JALR in PT
        else a = in.a[64] ? rvalue[0][63:0] : in.a[63:0];
        b = in.b[64] ? rvalue[1][63:0] : in.b[63:0];
        if (in.iword) {a, b} = {32'd0, a[31:0], 32'd0, b[31:0]};
        if (in.isign) {a, b} = {{32{a[31]}}, a[31:0], {32{b[31]}}, b[31:0]};
    end
    always_comb sub = {1'b0, a} - {1'b0, b};
    always_comb add = a + b;
    always_comb sll = a << b[5:0];
    always_comb srl = a >> b[5:0];
    always_comb sra = $signed($signed(a) >>> b[5:0]);
    always_comb bflag = {~|sub, sub[63], sub[64]}; // zero, negative, carry
    always_comb begin
        out_pt = in;
        if (op[`EX_CSR]) out_pt.valid = 0;
        else if (lsu) out_pt.valid = in.valid & rvalue[0][64];
        else out_pt.valid = in.valid & (rvalue[0][64] | rvalue[1][64]);
        if (frompt) out_pt.valid = 0;
        out_pt.a = in.a[64] | in.base[64] ? rvalue[0] : in.a;
        out_pt.b = in.b[64] ? rvalue[1] : in.b;
    end
    always_comb get_pt = ~in_pt.valid | frompt &
        ~(|mul_op & ~mul_free) & ~(|div_op & ~div_free) & ~(|fpu_op & ~fpu_free);
    always_comb get_id = ~in_id.valid | cqid_id[`lgCQSZ] & ~frompt &
        (out_pt.valid & ena_pt | ~out_pt.valid & ena_wb) &
        ~(|mul_op & ~mul_free) & ~(|div_op & ~div_free) & ~(|fpu_op & ~fpu_free) &
        ~(lsu & ~lsu_free & lsu_rqst[`lgCQSZ]);
    always_comb ready = (get_id & in_id.valid | frompt) & ~out_pt.valid;
    always_comb cqid = frompt ? cqid_pt : cqid_id;
    always_comb mul_valid = ~rst & ready & |mul_op;
    always_comb mul_rqst = {`lgCQSZ+1{mul_valid}} & cqid;
    always_comb mul_op = {op[`EX_MUL] & in.iword,
        op[`EX_MULHU], op[`EX_MULHSU], op[`EX_MULH], op[`EX_MUL] & ~in.iword};
    always_comb {mul_a, mul_b} = {a, b};
    always_comb div_valid = ~rst & ready & |div_op;
    always_comb div_rqst = {`lgCQSZ+1{div_valid}} & cqid;
    always_comb div_op = {op[`EX_REMU] & in.iword, op[`EX_REM] & in.iword,
                          op[`EX_DIVU] & in.iword, op[`EX_DIV] & in.iword,
                          op[`EX_REMU] & ~in.iword,  op[`EX_REM] & ~in.iword,
                          op[`EX_DIVU] & ~in.iword,  op[`EX_DIV] & ~in.iword};
    always_comb {div_a, div_b} = {a, b};
    always_comb fpu_valid = ~rst & ready & |fpu_op;
    always_comb fpu_rqst = {`lgCQSZ+1{fpu_valid}} & cqid;
    always_comb fpu_op = {
        op[`EX_FCVTDS], op[`EX_FCVTSD], op[`EX_FCVTFI], op[`EX_FCVTIF],
        op[`EX_FMVFX],  op[`EX_FCLASS], op[`EX_FMVXF],  op[`EX_FLE],
        op[`EX_FLT],    op[`EX_FEQ],    op[`EX_FMAX],   op[`EX_FMIN],
        op[`EX_FSGNJX], op[`EX_FSGNJN], op[`EX_FSGNJ],  op[`EX_FSQRT],
        op[`EX_FDIV],   op[`EX_FNMUL],  op[`EX_FMUL],   op[`EX_FSUB],
        op[`EX_FADD]};
    always_comb {fpu_a, fpu_b} = {a, b};
    always_comb {fpu_rm, fpu_double} = {in.funct3, in.fdouble};
    always_comb lsu = op[`EX_LOAD] | op[`EX_STORE] | op[`EX_FENCE] | op[`EX_CSR];
    always_comb lsu_valid = ~rst & (get_id & in_id.valid) & lsu;
    always_ff @(posedge clk) begin
        if (lsu_wdat[64] & late_done == lsu_wdat[`lgCQSZ:0])
            lsu_wdat <= late_val;
        if (lsu_free | ~lsu_rqst[`lgCQSZ] & ~|lsu_fence)
            if (lsu_valid)
                if (op[`EX_FENCE])
                    {lsu_rqst, lsu_fence} <= {{`lgCQSZ+1{1'b0}}, in.b[11:0]};
                else begin
                    {lsu_rqst, lsu_fence} <= {cqid, 12'd0};
                    lsu_wena <= op[`EX_STORE] | op[`EX_CSR];
                    // maybe using LSQ id instead of CQ id as index is better
                    lsu_addr <= out_pt.valid ? res : {1'b0, add};
                    lsu_bits <= in.funct3;
                    lsu_wdat <= rvalue[1];
                    lsu_rsrv <= in.rsrv;
                    lsu_aqrl <= in.aqrl;
                    lsu_csr <= op[`EX_CSR];
                    if (op[`EX_CSR]) lsu_addr <= {1'b0, b};
                    if (op[`EX_CSR]) lsu_wdat <= in.a[64] ? rvalue[0] : in.a;
                end
            else {lsu_rqst, lsu_fence} <= 0;
    end
    always_comb if (out_pt.valid) res = {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
        else if (in.valid & lsu & ~op[`EX_FENCE])
            res = {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
        else begin
            res =
                {65{op[`EX_ADD]}}  & {1'b0, add} |
                {65{op[`EX_SUB]}}  & {1'b0, sub[63:0]} |
                {65{op[`EX_SLL]}}  & {1'b0, sll} |
                {65{op[`EX_SRL]}}  & {1'b0, srl} |
                {65{op[`EX_SRA]}}  & {1'b0, sra} |
                {65{op[`EX_SLT]}}  & {64'd0, sub[63]} |
                {65{op[`EX_SLTU]}} & {64'd0, sub[64]} |
                {65{op[`EX_XOR]}}  & {1'b0, a ^ b} |
                {65{op[`EX_OR]}}   & {1'b0, a | b} |
                {65{op[`EX_AND]}}  & {1'b0, a & b} |
                {65{op[`EX_MIN]}}  & {1'b0, sub[63] ? a : b} |
                {65{op[`EX_MAX]}}  & {1'b0, sub[63] ? b : a} |
                {65{|mul_op}}      & {1'b1, {63-`lgCQSZ{1'd0}}, cqid} |
                {65{|div_op}}      & {1'b1, {63-`lgCQSZ{1'd0}}, cqid} |
                {65{|fpu_op}}      & {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
            if (in.iword) res[63:0] = {{32{res[31]}}, res[31:0]};
        end
    always_ff @(posedge clk) if (rst) pt_done <= 0;
        else if (frompt & ~lsu)
            if (res[64]) pt_done <= 0;
            else {pt_done, pt_data, pt_exc} <= {cqid_pt, res, ready & excp};
        else if (ena_arb) pt_done <= 0;
    always_ff @(posedge clk) if (rst) addr_done <= 0;
        else if (frompt & lsu) {addr_done, addr_val} <= {cqid_pt, add[63:0]};
        else addr_done <= 0;
    always_ff @(posedge clk)
        if (rst) out_wb.valid <= 0;
        else if (get_id & in_id.valid) begin
            out_wb.valid <= 1'b1;
            out_wb.rda <= in.rda;
            out_wb.rd <= res;
            out_wb.mw <= op[`EX_STORE] | op[`EX_CSR];
            out_wb.pc <= in.pc;
        end else if (ena_wb) out_wb.valid <= 0;
    always_comb if (op[`EX_ECALL] | op[`EX_EBREAK]) jpc = tvec;
        else if (frompt & in_pt.j) jpc = in.a[63:0] + in.offset; // JALR
        else if (in.base[64]) jpc = rvalue[0][63:0] + in.offset;
        else jpc = in.base[63:0] + in.offset;
    always_comb jump = op[`EX_ECALL] | op[`EX_EBREAK] |
        in.j | |in.bmask & in.bneg != |(in.bmask & bflag);
    always_comb excp = jump & ~(in.branch & in.bpc == jpc) | ~jump & in.branch;
    always_comb fencei = (fencei_r | op[`EX_FENCEI]) & ~lsu_empty;
    always_ff @(posedge clk) if (rst | ready & excp | lsu_empty) fencei_r <= 0;
        else if (fencei) fencei_r <= 1;
    always_ff @(posedge clk)
        if (rst) out_pc.valid <= 1'b0;
        else if (ready & excp | fencei) begin
            out_pc.valid <= 1'b1;
            if (ready & excp | in_id.valid) begin
                out_pc.pc <= in.pc;
                out_pc.npc <= jump ? jpc : in.pc + (in.c ? 64'd2 : 64'd4);
            end
        end else out_pc.valid <= 1'b0;
endmodule

module wb_stage(input logic clk, input logic rst,
    input ex_wb_t in_ex, output logic get_ex,
    input logic ena_ex, // for register read sync
    input logic [1:0][6:0] raddr, output logic [1:0][64:0] rvalue,
    output logic [`lgCQSZ:0] cqid,
    input logic [`lgCQSZ:0] late_done, input logic [64:0] late_val, input logic late_exc
);
    // register number: 00_xxxxx -> integer, 01_xxxxx -> float, 10_00000 -> tmp
    logic [64:0][`lgCQSZ:0] regscqid;
    logic [1:0][63:0] regsval;
    // commit queue
    logic [`lgCQSZ-1:0] cqfront, cqfrontp1, cqfrontp2, cqrear, cqrearp1;
    logic cqempty, cqfull, cqpush, cqpop1, cqpop2;
    logic [`CQSZ-1:0] cqexc, cqmw;
    logic [63:0] cqpc, cqpcp1;
    logic [6:0] cqrda, cqrdap1;
    logic [64:0] cqfrontval, cqfrontp1val;
    logic [1:0][`lgCQSZ-1:0] cqraddr;
    logic [1:0][64:0] cqrvalue;
    logic recover, lsu_cmt, lsu_cmtp1;
    regfile #(.dwidth(64), .rports(2), .wports(2), .awidth(7), .depth(65))
        regs_inst(.clk(clk), .rst(rst), .raddr(raddr), .rvalue(regsval),
            .waddr({cqrdap1, cqrda}), .wvalue({cqfrontp1val[63:0], cqfrontval[63:0]}),
            .wena({cqpop2 & |cqrdap1, cqpop1 & |cqrda}));
    regfile #(.dwidth(64), .rports(2), .wports(1), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqpc_inst(.clk(clk), .rst(rst),
            .raddr({cqfrontp1, cqfront}), .rvalue({cqpcp1, cqpc}),
            .waddr(cqrear), .wvalue(in_ex.pc), .wena(cqpush));
    regfile #(.dwidth(7), .rports(2), .wports(1), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqrda_inst(.clk(clk), .rst(rst),
            .raddr({cqfrontp1, cqfront}), .rvalue({cqrdap1, cqrda}),
            .waddr(cqrear), .wvalue(in_ex.rda), .wena(cqpush));
    regfile #(.dwidth(65), .rports(4), .wports(2), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqval_inst(.clk(clk), .rst(rst),
            .raddr({cqfrontp1, cqfront, cqraddr}),
            .rvalue({cqfrontp1val, cqfrontval, cqrvalue}),
            .waddr({late_done[`lgCQSZ-1:0], cqrear}), .wvalue({late_val, in_ex.rd}),
            .wena({late_done[`lgCQSZ], cqpush}));
    always_comb get_ex = ~in_ex.valid | cqpush;
    always_comb cqfrontp1 = cqfront + 1;
    always_comb cqfrontp2 = cqfront + 2;
    always_comb cqrearp1 = cqrear + 1;
    always_comb if (cqfull) cqid = 0;
        else if (cqpush & ~cqpop1 & cqfront == cqrearp1) cqid = 0;
        else cqid = {1'b1, cqpush ? cqrearp1 : cqrear};
    always_comb cqpush = in_ex.valid & (~cqfull | cqpop1);
    always_comb cqpop1 = ~cqfrontval[64] & ~cqempty;
    always_comb cqpop2 = ~cqfrontp1val[64] & ~cqexc[cqfrontp1] &
        cqfrontp1 != cqrear & ~cqexc[cqfront] & cqpop1 & ~cqmw[cqfront];
    always_comb recover = ~cqempty & cqexc[cqfront];
    always_comb lsu_cmt = cqmw[cqfront] | cqmw[cqfrontp1];
    always_comb lsu_cmtp1 = cqmw[cqfront] & cqmw[cqfrontp1];
    always_ff @(posedge clk)
        if (rst | recover) {cqexc, cqfront, cqrear, cqfull, cqempty} <= 1;
        else begin
            if (cqpop1 & ~cqpush & cqrear == cqfrontp1 |
                cqpop2 & ~cqpush & cqrear == cqfrontp2)
                cqempty <= 1;
            if (cqempty & cqpush & ~cqpop1) cqempty <= 0;
            if (cqpush & ~cqpop1 & cqfront == cqrearp1) cqfull <= 1;
            if (cqpop1 & ~cqpush | cqpop2) cqfull <= 0;
            cqfront <= cqpop2 ? cqfrontp2 : (cqpop1 ? cqfrontp1 : cqfront);
            cqrear <= cqpush ? cqrearp1 : cqrear;
            if (cqpush) cqexc[cqrear] <= 0;
            if (late_done[`lgCQSZ]) cqexc[late_done[`lgCQSZ-1:0]] <= late_exc;
            if (cqpush) cqmw[cqrear] <= in_ex.mw;
            if (lsu_cmt) cqmw[cqfront] <= 0;
            if (lsu_cmtp1) cqmw[cqfrontp1] <= 0;
            assert((cqempty | cqfull) == (cqfront == cqrear));
            if (cqpush & in_ex.rd[64]) assert(in_ex.rd[`lgCQSZ-1:0] == cqrear);
        end
    always_ff @(posedge clk)
        if (rst | recover) for (int i = 0; i < 65; i++) regscqid[i][`lgCQSZ] <= 0;
        else begin // commit
            if (cqpop1 & regscqid[cqrda][`lgCQSZ-1:0] == cqfront)
                regscqid[cqrda] <= 0;
            if (cqpop2 & regscqid[cqrdap1][`lgCQSZ-1:0] == cqfrontp1)
                regscqid[cqrdap1] <= 0;
            if (cqpush & |in_ex.rda) regscqid[in_ex.rda] <= {1'b1, cqrear};
        end
    always_comb for (int i = 0; i < 2; i++)
        cqraddr[i] = regscqid[raddr[i]][`lgCQSZ-1:0];
    logic [1:0][6:0] raddr_r;
    logic [1:0][64:0] rvalue_r;
    always_ff @(posedge clk) if (ena_ex) raddr_r <= raddr; // sync with ID stage output
    always_ff @(posedge clk) if (ena_ex) for (int i = 0; i < 2; i++)
        if (in_ex.valid & raddr[i] == in_ex.rda) rvalue_r[i] <= in_ex.rd;
        else if (regscqid[raddr[i]][`lgCQSZ]) begin
            if (cqrvalue[i][64] & late_done == cqrvalue[i][`lgCQSZ:0])
                rvalue_r[i] <= late_val;
            else rvalue_r[i] <= cqrvalue[i];
        end else rvalue_r[i] <= {1'b0, regsval[i]}; else rvalue_r <= rvalue;
    always_comb for (int i = 0; i < 2; i++)
        if (raddr_r[i] == 0) rvalue[i] = 0;
        else if (in_ex.valid & raddr_r[i] == in_ex.rda) rvalue[i] = in_ex.rd;
        else if (rvalue_r[i][64] & late_done == rvalue_r[i][`lgCQSZ:0])
            rvalue[i] = late_val;
        else rvalue[i] = rvalue_r[i];
endmodule

module pending_table(input logic clk, input logic rst, input logic flush,
    input id_ex_t in_ex, output logic get_ex,
    output id_ex_t out_ex, input logic ena_ex,
    input logic [`lgCQSZ:0] cqid_in, output logic [`lgCQSZ:0] cqid_out,
    input logic [`lgCQSZ:0] late_done, input logic [64:0] late_val,
    input logic [6:0] rda_id, input logic [6:0] rda_ex
);
    logic [`lgPTSZ-1:0] in, out;
    logic in_ena, out_ena;
    logic [`lgCQSZ:0] id[`PTSZ-1:0];
    logic [64:0] a[`PTSZ-1:0], b[`PTSZ-1:0], fwd_a[`PTSZ:0], fwd_b[`PTSZ:0];
    id_ex_t in_ex_fwd, data_out;
    regfile #(.dwidth(`PTLEN), .rports(1), .wports(1), .awidth(`lgPTSZ), .depth(`PTSZ))
        data_inst(.clk(clk), .rst(rst), .raddr(out), .rvalue(data_out),
            .waddr(in), .wvalue(in_ex_fwd), .wena(~flush & in_ex_fwd.valid & in_ena));
    always_comb get_ex = in_ena;
    always_comb begin
        {in_ena, out_ena, in, out} = 0;
        for (int i = `PTSZ-1; i >= 0; i--)
            if (~fwd_a[i][64] & ~fwd_b[i][64]) {out_ena, out} = {1'b1, i[`lgPTSZ-1:0]};
        for (int i = `PTSZ-1; i >= 0; i--)
            if (~id[i][`lgCQSZ]) {in_ena, in} = {1'b1, i[`lgPTSZ-1:0]};
        in_ex_fwd = in_ex;
        {in_ex_fwd.a, in_ex_fwd.b} = {fwd_a[`PTSZ], fwd_b[`PTSZ]};
    end
    always_comb for (int i = 0; i <= `PTSZ; i++)
        if (i == `PTSZ | id[i][`lgCQSZ]) begin
            if (i == `PTSZ) {fwd_a[i], fwd_b[i]} = {in_ex.a, in_ex.b};
            else {fwd_a[i], fwd_b[i]} = {a[i], b[i]};
            if (fwd_a[i][64] & late_done == fwd_a[i][`lgCQSZ:0]) fwd_a[i] = late_val;
            if (fwd_b[i][64] & late_done == fwd_b[i][`lgCQSZ:0]) fwd_b[i] = late_val;
        end else {fwd_a[i], fwd_b[i]} = {1'b1, 64'd0, 1'b1, 64'd0};
    always_ff @(posedge clk) if (rst | flush) {out_ex.valid, cqid_out} <= 0;
        else if (ena_ex) begin
            out_ex <= data_out;
            {out_ex.a, out_ex.b} <= {fwd_a[{1'b0, out}], fwd_b[{1'b0, out}]};
            cqid_out <= id[out];
            out_ex.valid <= out_ena;
        end
    always_ff @(posedge clk)
        if (rst | flush) for (int i = 0; i < `PTSZ; i++) id[i] <= 0;
        else begin
            if (in_ex_fwd.valid & in_ena) id[in] <= cqid_in;
            if (out_ena & ena_ex) id[out] <= 0;
        end
    always_ff @(posedge clk) for (int i = 0; i < `PTSZ; i++)
        if (in_ex_fwd.valid & in_ena & i[`lgPTSZ-1:0] == in)
            {a[i], b[i]} <= {in_ex_fwd.a, in_ex_fwd.b};
        else {a[i], b[i]} <= {fwd_a[i], fwd_b[i]};
endmodule

module lsu(input logic clk, input logic rst, input logic flush,
    input logic ena, output logic get,
    input logic cmt, input logic cmtp1,
    input logic [11:0] fence, input logic [1:0] rsrv, input logic [1:0] aqrl,
    input logic [`lgCQSZ:0] rqst, input logic wena, input logic csr,
    input logic [64:0] addr, input logic [2:0] bits,
    output logic [`lgCQSZ:0] done, output logic excp,
    output logic [64:0] rdata, input logic [64:0] wdata,
    input logic [`lgCQSZ:0] late_done, input logic [64:0] late_val,
    input logic [`lgCQSZ:0] addr_done, input logic [63:0] addr_val,
    output logic [11:0] csr_addr, output logic csr_wena, output logic [2:0] csr_func,
    input logic [63:0] csr_rval, output logic [63:0] csr_wval, input logic csr_excp,
    output logic [`lgCQSZ:0] dcache_rqst,
    output logic       [1:0] dcache_rsrv,
    output logic             dcache_wena,
    output logic      [63:0] dcache_addr,
    output logic       [2:0] dcache_bits,
    input  logic [`lgCQSZ:0] dcache_done,
    input  logic      [63:0] dcache_rdat,
    output logic      [63:0] dcache_wdat,
    output logic             dcache_flsh
);
    logic [`LSQSZ-1:0][`lgCQSZ:0] lsqrqst;
    logic [`LSQSZ-1:0][64:0] lsqaddr;
    logic [`LSQSZ-1:0][64:0] lsqdata;
    logic [`LSQSZ-1:0][2:0] lsqbits;
    logic [`LSQSZ-1:0][1:0] lsqrsrv;
    logic [`LSQSZ-1:0] lsqsent, lsqcmt, lsqwena, lsqfwd, lsqraq, lsqcsr;
    logic [`lgLSQSZ-1:0] front, rear, frontp1, rearp1;
    logic full, empty, push, pop, th, thr, ready;
    logic [`lgCQSZ:0] fwd; logic [64:0] fwddata, fwdval; logic [2:0] fwdbits;
    logic [`lgCQSZ:0] thrrqst; logic [1:0] thrrsrv; logic thrwena;
    logic [64:0] thraddr; logic [2:0] thrbits;
    always_comb ready = ~empty & ~lsqsent[front] & ~lsqaddr[front][64] &
        (~lsqwena[front] | ~lsqdata[front][64] & (cmt | lsqcmt[front]));
    always_comb push = rqst[`lgCQSZ] & (~full | pop) &
        ~(fwd[`lgCQSZ] & dcache_done[`lgCQSZ]);
    always_comb pop = ~empty & ~thr & ~|lsqrqst[front];
    always_comb frontp1 = front + 1;
    always_comb rearp1 = rear + 1;
    always_comb get = ~rqst[`lgCQSZ] | push;
    always_comb begin
        th = rqst[`lgCQSZ] & ~wena & ~addr[64];
        for (int i = 0; i < `LSQSZ; i++) if (lsqrqst[i][`lgCQSZ])
            if (lsqaddr[i][64] | addr[63:3] == lsqaddr[i][63:3]) th = 0;
        for (int i = 0; i < `LSQSZ; i++) if (lsqrqst[i][`lgCQSZ] & lsqraq[i]) th = 0;
        if (aqrl[0] | csr) th = 0;
    end
    always_comb case (fwdbits[1:0])
        0: fwdval = {1'b0, {56{fwddata[7] & ~fwdbits[2]}}, fwddata[7:0]};
        1: fwdval = {1'b0, {48{fwddata[15] & ~fwdbits[2]}}, fwddata[15:0]};
        2: fwdval = {1'b0, {32{fwddata[31] & ~fwdbits[2]}}, fwddata[31:0]};
        3: fwdval = fwddata;
    endcase
    always_ff @(posedge clk)
        if (push) begin
            {fwd, fwddata} <= 0; thr <= th;
            if (rqst[`lgCQSZ] & ~wena)
                for (int i = 0; i < `LSQSZ; i++) if (lsqrqst[i][`lgCQSZ])
                    if (~lsqaddr[i][64] & ~lsqdata[i][64] &
                        lsqfwd[i] & lsqwena[i] & ~|lsqrsrv[i] &
                        addr[63:0] == lsqaddr[i][63:0] & bits[1:0] == lsqbits[i][1:0])
                        {fwd, fwdbits, fwddata} <= {rqst, bits, lsqdata[i]};
            for (int i = 0; i < `LSQSZ; i++)
                if (lsqrqst[i][`lgCQSZ] & lsqraq[i]) fwd <= 0;
            if (aqrl[0] | csr) fwd <= 0;
        end else if (~dcache_done[`lgCQSZ]) {fwd, thr} <= 0; else thr <= 0;
    always_ff @(posedge clk) if (rst | flush) {front, rear, full, empty} <= 1;
        else begin
            if (pop & ~push & frontp1 == rear) empty <= 1;
            if (push & ~pop & rearp1 == front) full <= 1;
            if (push) empty <= 0;
            if (pop & ~push) full <= 0;
            if (pop) begin lsqrqst[front] <= 0; front <= frontp1; end
            if (push) begin
                rear <= rearp1;
                lsqsent[rear] <= th; lsqcmt[rear] <= cmt & empty;
                lsqfwd[rear] <= ~addr[64]; lsqraq[rear] <= aqrl[1]; lsqcsr[rear] <= csr;
                lsqrqst[rear] <= rqst; lsqrsrv[rear] <= rsrv; lsqwena[rear] <= wena;
                lsqaddr[rear] <= addr; lsqdata[rear] <= wdata; lsqbits[rear] <= bits;
                thrrqst <= rqst; thrrsrv <= rsrv; thrwena <= wena;
                thraddr <= addr; thrbits <= bits;
                if (addr[64] & addr_done == addr[`lgCQSZ:0])
                    lsqaddr[rear] <= {1'b0, addr_val};
                if (wdata[64] & late_done == wdata[`lgCQSZ:0])
                    lsqdata[rear] <= late_val;
            end
            if (cmt) lsqcmt[front] <= 1;
            if (cmtp1) lsqcmt[frontp1] <= 1;
            if (~thr & ready) lsqsent[front] <= 1;
            for (int i = 0; i < `LSQSZ; i++) if (lsqrqst[i][`lgCQSZ]) begin
                if (push & (addr[64] | addr[64:3] == lsqaddr[i][64:3])) lsqfwd[i] <= 0;
                if (done == lsqrqst[i]) lsqrqst[i] <= 0;
                if (lsqaddr[i][64] & addr_done == lsqaddr[i][`lgCQSZ:0])
                    lsqaddr[i] <= {1'b0, addr_val};
                if (lsqdata[i][64] & late_done == lsqdata[i][`lgCQSZ:0])
                    lsqdata[i] <= late_val;
            end
            for (int i = 0; i < `LSQSZ; i++) if (lsqrqst[i][`lgCQSZ])
                if (lsqwena[i] &  fence[1] & fence[4] |
                    ~lsqwena[i] & (fence[1] & fence[5] | fence[11]))
                    lsqraq[i] <= 1;
        end
    always_comb dcache_rqst = thr ? thrrqst :
        (ready & ~lsqcsr[front] ? lsqrqst[front] : 0);
    always_comb dcache_rsrv = thr ? thrrsrv : lsqrsrv[front];
    always_comb dcache_wena = thr ? thrwena : lsqwena[front];
    always_comb dcache_addr = thr ? thraddr[63:0] : lsqaddr[front][63:0];
    always_comb dcache_bits = thr ? thrbits : lsqbits[front];
    always_comb done = dcache_done[`lgCQSZ] ? dcache_done : (
        fwd[`lgCQSZ] ? fwd : (ready & lsqcsr[front] ? lsqrqst[front] : 0));
    always_comb rdata = dcache_done[`lgCQSZ] ? {1'b0, dcache_rdat} : (
        fwd[`lgCQSZ] ? fwdval : {1'b0, csr_rval});
    always_comb excp = csr_excp | 0;
    always_comb dcache_wdat = lsqdata[front][63:0];
    always_comb dcache_flsh = flush;
    always_comb csr_addr = lsqaddr[front][11:0];
    always_comb csr_wena = ready & lsqcsr[front];
    always_comb csr_func = lsqbits[front];
    always_comb csr_wval = lsqdata[front][63:0];
endmodule

module arbiter(
    input logic [`LATENUM-1:0][`lgCQSZ:0] done_in,
    input logic [`LATENUM-1:0][64:0] val_in,
    input logic [`LATENUM-1:0] exc_in,
    output logic [`lgCQSZ:0] done_out,
    output logic [64:0] val_out, output logic exc_out,
    output logic [`LATENUM-1:0] get);
    always_comb begin
        get = 0;
        {done_out, val_out, exc_out} = 0;
        for (int i = `LATENUM - 1; i >= 0; i--) if (done_in[i][`lgCQSZ]) begin
            get = 1 << i;
            {done_out, val_out, exc_out} = {done_in[i], val_in[i], exc_in[i]};
        end
        for (int i = 0; i < `LATENUM; i++) if (~done_in[i][`lgCQSZ]) get[i] = 1;
    end
endmodule

/********************************** CSR map ***********************************
User-level CSR:
    0x000 -- 0x005:
        0x000 -> ustatus    0x001 -> fflags    0x002 -> frm    0x003 -> fcsr
        0x004 -> uie        0x005 -> utvec
    0x040 -- 0x044:
        0x040 -> uscratch    0x041 -> uepc    0x042 -> ucause    0x043 -> utval
        0x044 -> uip
    0xc00 -- 0xc1f:
        0xc00 -> cycle    0xc01 -> time    0xc02 -> instret
        0xc03-0xc1f -> hpmcounter
Supervisor-level CSR:
    0x100 -- 0x106:
        0x100 -> sstatus    0x102 -> sedeleg    0x103 -> sideleg    0x104 -> sie
        0x105 -> stvec      0x106 -> scounteren
    0x140 -- 0x144:
        0x140 -> sscrach    0x141 -> sepc    0x142 -> scause    0x143 -> stval
        0x144 -> sip
    0x180 -- 0x180:
        0x180 -> satp
Machine-level CSR:
    0x300 -- 0x306:
        0x300 -> mstatus    0x301 -> misa     0x302 -> medeleg    0x303 -> mideleg
        0x304 -> mie        0x305 -> mtvec    0x306 -> mcounteren
    0x320 -- 0x33f:
        0x320 -> mcounterinhibit    0x323-0x33f -> mhpmevent
    0x340 -- 0x344:
        0x340 -> mscratch    0x341 -> mepc    0x342 -> mcause    0x343 -> mtval
        0x344 -> mip
    0x3a0 -- 0x3bf:
        0x3a0,0x3a2 -> pmpcfg    0x3b0-0x3bf -> pmpaddr
    0x7a0 -- 0x7a3:
        0x7a0 -> tselect    0x7a1-0x7a3 -> tdata
    0x7b0 -- 0x7b3:
        0x7b0 -> dcsr       0x7b1 -> dpc    0x7b2-0x7b3 -> dscratch
    0xb00 -- 0xb1f:
        0xb00 -> mcycle    0xb02 -> minstret    0xb03-0xb1f -> mhpmcounter
    0xf11 -- 0xf14:
        0xf11 -> mvendorid    0xf12 -> marchid    0xf13 -> mimpid    0xf14 -> mhartid
******************************************************************************/
module csr(input logic clk, input logic rst,
    input logic [11:0] addr, output logic [63:0] rval,
    input logic wena, input logic [63:0] wval, input logic [2:0] func,
    input logic [63:0] nret, output logic excp
);
    logic [63:0] wres;
    logic [63:0] misa, mvendorid, marchid, mimpid, mhartid;
    logic [63:0] mstatus, mtvec, medeleg, mideleg;
    logic [63:0] utvec, mcycle, minstret;
    always_comb case (func[1:0])
        2'b00: wres = 0;
        2'b01: wres = wval;
        2'b10: wres = wval | rval;
        2'b11: wres = ~wval & rval;
    endcase
    always_comb case (addr)
        12'h301: rval = misa;    12'hf11: rval = mvendorid;
        12'hf12: rval = marchid; 12'hf13: rval = mimpid;
        12'hf14: rval = mhartid; 12'h300: rval = mstatus;
        12'h305: rval = mtvec;   12'h302: rval = medeleg;
        12'h303: rval = mideleg;

        12'h005: rval = utvec;
        12'hb00: rval = mcycle;
        12'hb02: rval = minstret;
        default: rval = 0;
    endcase
    always_ff @(posedge clk) begin
        if (rst) misa <= {2'h2, 36'h0, 26'h112D}; if (wena & addr == 12'h301) begin
            misa[0] <= wres[0]; misa[3:2] <= wres[3:2]; misa[4] <= ~wres[8];
            misa[8:5] <= wres[8:5]; misa[13:12] <= wres[13:12]; misa[16] <= wres[16];
            misa[18] <= wres[18]; misa[20] <= wres[20]; misa[23] <= wres[23];
            if (wres[5]) {misa[3], misa[16]} <= 0;
        end
        if (rst) mvendorid <= 0; else if (wena & addr == 12'hf11) excp <= 1;
        if (rst) marchid <= 0; else if (wena & addr == 12'hf12) excp <= 1;
        if (rst) mimpid <= 0; else if (wena & addr == 12'hf13) excp <= 1;
        if (rst) mhartid <= 0; else if (wena & addr == 12'hf13) excp <= 1;
        if (rst) mstatus <= {32'ha, 19'h1, 13'h0};
        else if (wena & addr == 12'h300) begin
            mstatus <= wres;
            mstatus[63] <= mstatus[16:15] == 2'b11 | mstatus[14:13] == 2'b11;
            {mstatus[62:36], mstatus[31:23]} <= 0;
            {mstatus[10:9], mstatus[6], mstatus[2]} <= 0;
        end
        if (rst) mtvec <= 0; else if (wena & addr == 12'h305) begin
            mtvec <= wres; mtvec[1] <= 0;
        end
        if (rst) medeleg <= 0; else if (wena & addr == 12'h302) begin
            medeleg <= wres; medeleg[11] <= 0;
        end
        if (rst) mideleg <= 0; else if (wena & addr == 12'h303) mideleg <= wres;

        if (rst) mcycle <= 0; else mcycle <= mcycle + 64'd1;
        if (rst) minstret <= 0; else minstret <= minstret + nret;
        if (wena & addr == 12'h005) utvec    <= wres;
        if (wena & addr == 12'hb00) mcycle   <= wres;
        if (wena & addr == 12'hb02) minstret <= wres;
        if (rst) excp <= 0;
    end
    logic [63:0] tvec;
    always_comb tvec = mtvec;
endmodule

module ci2i(input logic [31:0] ci, output logic [31:0] i);
    logic [7:0][4:0] map;
    always_comb map = {5'd15, 5'd14, 5'd13, 5'd12, 5'd11, 5'd10, 5'd9, 5'd8};
    always_comb
        if (ci[1:0] == 2'b11) i = ci; // normal 32-bit instruction
        else case ({ci[15:13], ci[1:0]})
            5'b00000:
                if (ci[12:5] == 8'd0) // illegal
                    i = 0;
                else // C.ADDI4SPN ==> addi rd', x2, nzuimm
                    i = {{2'd0, ci[10:7], ci[12:11], ci[5], ci[6], 2'd0},
                         5'd2, 3'd0, map[ci[4:2]], 7'h13};
            5'b00100: // C.FLD ==> FLD rd', offset(rs1')
                i = {{4'd0, ci[6:5], ci[12:10], 3'd0},
                     map[ci[9:7]], 3'b011, map[ci[4:2]], 7'h7};
            5'b01000: // C.LW ==> LW rd', offset(rs1')
                i = {{5'd0, ci[5], ci[12:10], ci[6], 2'd0},
                     map[ci[9:7]], 3'b010, map[ci[4:2]], 7'h3};
            5'b01100: // C.LD ==> LD rd', offset(rs1')
                i = {{4'd0, ci[6:5], ci[12:10], 3'd0},
                     map[ci[9:7]], 3'b011, map[ci[4:2]], 7'h3};
            5'b10100: // C.FSD ==> FSD rs2', offset(rs1')
                i = {{4'd0, ci[6:5], ci[12]}, map[ci[4:2]], map[ci[9:7]],
                     3'b011, {ci[11:10], 3'd0}, 7'h27};
            5'b11000: // C.SW ==> SW rs2', offset(rs1')
                i = {{5'd0, ci[5], ci[12]}, map[ci[4:2]], map[ci[9:7]],
                     3'b010, {ci[11:10], ci[6], 2'd0}, 7'h23};
            5'b11100: // C.SD ==> SD rs2', offset(rs1')
                i = {{4'd0, ci[6:5], ci[12]}, map[ci[4:2]], map[ci[9:7]],
                     3'b011, {ci[11:10], 3'd0}, 7'h23};
            5'b00001: // C.ADDI / C.NOP ==> ADDI rd, rd, nzimm / NOP
                i = {{{7{ci[12]}}, ci[6:2]}, ci[11:7], 3'b000, ci[11:7], 7'h13};
            5'b00101: // C.ADDIW ==> ADDIW rd, rd, imm
                i = {{{7{ci[12]}}, ci[6:2]}, ci[11:7], 3'b000, ci[11:7], 7'h1B};
            5'b01001: // C.LI ==> ADDI rd, x0, imm
                i = {{{7{ci[12]}}, ci[6:2]}, 5'd0, 3'b000, ci[11:7], 7'h13};
            5'b01101:
                if ({ci[12], ci[6:2]} == 6'd0) // illegal
                    i = 0;
                else if (ci[11:7] == 5'd2) // C.ADDI16SP ==> ADDI x2, x2, nzimm
                    i = {{{3{ci[12]}}, ci[4:3], ci[5], ci[2], ci[6], 4'd0},
                         5'd2, 3'b000, 5'd2, 7'h13};
                else // C.LUI ==> LUI rd, nzimm
                    i = {{{3{ci[12]}}, ci[6:2], 12'd0}, ci[11:7], 7'h37};
            5'b10001:
                if (ci[11] == 1'd0) // C.SRLI/C.SRAI ==> SRLI/SRAI rd', rd', shamt
                    i = {ci[11:10], 4'd0, {ci[12], ci[6:2]},
                         map[ci[9:7]], 3'b101, map[ci[9:7]], 7'h13};
                else if (ci[10] == 1'd0) // C.ANDI ==> ANDI rd', rd', imm
                    i = {{{7{ci[12]}}, ci[6:2]},
                         map[ci[9:7]], 3'b111, map[ci[9:7]], 7'h13};
                else case ({ci[12], ci[6:5]})
                    3'b000: // C.SUB ==> SUB rd', rd', rs2'
                        i = {7'b0100000, map[ci[4:2]], map[ci[9:7]],
                             3'b000, map[ci[9:7]], 7'h33};
                    3'b001: // C.XOR ==> XOR rd', rd', rs2'
                        i = {7'd0, map[ci[4:2]], map[ci[9:7]],
                             3'b100, map[ci[9:7]], 7'h33};
                    3'b010: // C.OR ==> OR rd', rd', rs2'
                        i = {7'd0, map[ci[4:2]], map[ci[9:7]],
                             3'b110, map[ci[9:7]], 7'h33};
                    3'b011: // C.AND ==> AND rd', rd', rs2'
                        i = {7'd0, map[ci[4:2]], map[ci[9:7]],
                             3'b111, map[ci[9:7]], 7'h33};
                    3'b100: // C.SUBW ==> SUBW rd', rd', rs2'
                        i = {7'b0100000, map[ci[4:2]], map[ci[9:7]],
                             3'b000, map[ci[9:7]], 7'h3B};
                    3'b101: // C.ADDW ==> ADDW rd', rd', rs2'
                        i = {7'b0000000, map[ci[4:2]], map[ci[9:7]],
                             3'b000, map[ci[9:7]], 7'h3B};
                    default: i = 0;
                endcase
            5'b10101: // C.J ==> JAL x0, offset
                i = {ci[12], ci[8], ci[10:9], ci[6], ci[7], ci[2], ci[11], ci[5:3],
                     ci[12], {8{ci[12]}}, 5'd0, 7'h6F};
            5'b11001: // C.BEQZ ==> BEQ rs1', x0, offset
                i = {{4{ci[12]}}, ci[6:5], ci[2], 5'd0, map[ci[9:7]], 3'b000,
                     ci[11:10], ci[4:3], ci[12], 7'h63};
            5'b11101: // C.BNEZ ==> BNE rs1', x0, offset
                i = {{4{ci[12]}}, ci[6:5], ci[2], 5'd0, map[ci[9:7]], 3'b001,
                     ci[11:10], ci[4:3], ci[12], 7'h63};
            5'b00010:
                if ({ci[12], ci[6:2]} == 6'd0) // illegal
                    i = 0;
                else // C.SLLI ==> SLLI rd, rd, shamt
                    i = {6'd0, {ci[12], ci[6:2]}, ci[11:7], 3'b001, ci[11:7], 7'h13};
            5'b00110: // C.FLDSP ==> FLD rd, offset(x2)
                i = {{3'd0, ci[4:2], ci[12], ci[6:5], 3'd0},
                     5'd2, 3'b011, ci[11:7], 7'h7};
            5'b01010: // C.LWSP ==> LW rd, offset(x2)
                i = {{4'd0, ci[3:2], ci[12], ci[6:4], 2'd0},
                     5'd2, 3'b010, ci[11:7], 7'h3};
            5'b01110: // C.LDSP ==> LD rd, offset(x2)
                i = {{3'd0, ci[4:2], ci[12], ci[6:5], 3'd0},
                     5'd2, 3'b011, ci[11:7], 7'h3};
            5'b10010:
                if (ci[12] == 1'd0)
                    if (ci[6:2] == 5'd0)
                        if (ci[11:7] == 5'd0) // illegal
                            i = 0;
                        else // C.JR ==> JALR x0, 0(rs1)
                            i = {12'd0, ci[11:7], 3'b000, 5'd0, 7'h67};
                    else // C.MV ==> ADD rd, x0, rs2
                        i = {7'd0, ci[6:2], 5'd0, 3'b000, ci[11:7], 7'h33};
                else if (ci[6:2] == 5'd0)
                    if (ci[11:7] == 5'd0) // C.EBREAK ==> EBREAK
                        i = {12'd1, 5'd0, 3'd0, 5'd0, 7'h73};
                    else // C.JALR ==> JALR x1, 0(rs1)
                        i = {12'd0, ci[11:7], 3'b000, 5'd1, 7'h67};
                else // C.ADD ==> ADD rd, rd, rs2
                    i = {7'd0, ci[6:2], ci[11:7], 3'b000, ci[11:7], 7'h33};
            5'b10110: // C.FSDSP ==> FSD rs2, offset(x2)
                i = {{3'd0, ci[9:7], ci[12]}, ci[6:2], 5'd2, 3'b011,
                     {ci[11:10], 3'd0}, 7'h27};
            5'b11010: // C.SWSP ==> SW rs2, offset(x2)
                i = {{4'd0, ci[8:7], ci[12]}, ci[6:2], 5'd2, 3'b010,
                     {ci[11:9], 2'd0}, 7'h23};
            5'b11110: // C.SDSP ==> SD rs2, offset(x2)
                i = {{3'd0, ci[9:7], ci[12]}, ci[6:2], 5'd2, 3'b011,
                     {ci[11:10], 3'd0}, 7'h23};
            default: i = 0; // an illegal instruction
        endcase
endmodule
