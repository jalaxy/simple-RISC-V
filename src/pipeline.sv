`define RST_PC 64'h400000 // reset pc
`define PTSZ 8    // pending table size
`define CQSZ 16 // commit queue size
`define lgCQSZ 4
`define LATENUM 4 // number of late components
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
`define EX_MINU   6'd12
`define EX_MAXU   6'd13
`define EX_MUL    6'd14
`define EX_MULH   6'd15
`define EX_MULHSU 6'd16
`define EX_MULHU  6'd17
`define EX_DIV    6'd18
`define EX_DIVU   6'd19
`define EX_REM    6'd20
`define EX_REMU   6'd21
`define EX_ADDW   6'd22
`define EX_SUBW   6'd23
`define EX_SLLW   6'd24
`define EX_SRLW   6'd25
`define EX_SRAW   6'd26
`define EX_MULW   6'd27
`define EX_DIVW   6'd28
`define EX_DIVUW  6'd29
`define EX_REMW   6'd30
`define EX_REMUW  6'd31
`define EX_FADD   6'd32
`define EX_FSUB   6'd33
`define EX_FMUL   6'd34
`define EX_FNMUL  6'd35
`define EX_FDIV   6'd36
`define EX_FSQRT  6'd37
`define EX_FSGNJ  6'd38
`define EX_FSGNJN 6'd39
`define EX_FSGNJX 6'd40
`define EX_FMIN   6'd41
`define EX_FMAX   6'd42
`define EX_FEQ    6'd43
`define EX_FLT    6'd44
`define EX_FLE    6'd45
`define EX_FMVFI  6'd46
`define EX_FCLASS 6'd47
`define EX_FMVIF  6'd48
`define EX_FCVTIF 6'd49
`define EX_FCVTFI 6'd50
`define EX_FCVTSD 6'd51
`define EX_FCVTDS 6'd52
`define ID_PT  4'd1 // late component ID
`define ID_LSU 4'd2
`define ID_MUL 4'd4
`define ID_DIV 4'd8
`define MUL_MUL    3'd0 // multiplier operations
`define MUL_MULH   3'd1
`define MUL_MULHSU 3'd2
`define MUL_MULHU  3'd3
`define MUL_MULW   3'd4

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
    logic [52:0] exop;
    logic [2:0] frm, bmask;
    logic fdouble, bneg, j;
    logic [64:0] base;
    logic [63:0] offset;
    logic mr, mw;
    logic [2:0] bits;
    logic [6:0] rmwa;
    logic [64:0] a, b;
    logic [6:0] rda;
} id_ex_t;
typedef struct packed {
    logic valid;
    logic mw;
    logic [63:0] mwaddr;
    logic [2:0] mwbits;
    logic [6:0] rmwa;
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

    output logic [`lgCQSZ:0] dcache_r_rqst,
    output logic      [63:0] dcache_r_addr,
    output logic       [2:0] dcache_r_bits,
    input  logic [`lgCQSZ:0] dcache_r_done,
    input  logic      [63:0] dcache_r_data,

    output logic        dcache_w_rqst,
    output logic [63:0] dcache_w_addr,
    output logic  [2:0] dcache_w_bits,
    output logic [63:0] dcache_w_data,
    input  logic        dcache_w_done
);
    if_pc_t data_if_pc; logic get_if_pc;
    pc_if_t data_pc_if; logic get_pc_if;
    if_id_t data_if_id; logic get_if_id;
    id_ex_t data_id_ex; logic get_id_ex;
    ex_wb_t data_ex_wb; logic get_ex_wb;
    ex_pc_t data_ex_pc; logic get_ex_pc;
    id_ex_t data_ex_pt; logic get_ex_pt;
    id_ex_t data_pt_ex;
    logic [2:0][6:0] raddr; logic [2:0][64:0] rvalue;
    logic [`lgCQSZ:0] cqid_new, cqid_old;
    logic lsu_w_rqst; logic [2:0] lsu_w_bits;
    logic [`lgCQSZ:0] lsu_r_rqst, lsu_r_done; logic [2:0] lsu_r_bits; logic lsu_r_exc;
    logic [`lgCQSZ:0] mul_rqst, mul_done; logic [4:0] mul_op; logic mul_exc;
    logic [`lgCQSZ:0] div_rqst, div_done; logic [4:0] div_op; logic div_exc;
    logic [63:0] lsu_r_addr, lsu_w_addr;
    logic [64:0] lsu_r_data, lsu_w_data;
    logic [63:0] mul_a, mul_b, div_a, div_b;
    logic [64:0] mul_r, div_r;

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
        .out_ex(data_id_ex), .ena_ex(get_id_ex & ~|wb_stage_inst.cqexc));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst),
        .in_id(data_id_ex), .get_id(get_id_ex),
        .in_pt(data_pt_ex),
        .out_wb(data_ex_wb), .ena_wb(get_ex_wb),
        .out_pc(data_ex_pc), .ena_pc(get_ex_pc),
        .out_pt(data_ex_pt), .ena_pt(get_ex_pt),
        .raddr(raddr), .rvalue(rvalue),
        .cqid_id(cqid_new), .cqid_pt(cqid_old),
        .mul_rqst(mul_rqst), .mul_op(mul_op),
        .mul_a(mul_a), .mul_b(mul_b),
        .lsu_rqst(lsu_r_rqst), .lsu_addr(lsu_r_addr),
        .lsu_bits(lsu_r_bits));
    wb_stage wb_stage_inst(.clk(clk), .rst(rst),
        .in_ex(data_ex_wb), .get_ex(get_ex_wb),
        .raddr(raddr), .rvalue(rvalue), .cqid(cqid_new),
        .lsu_rqst(lsu_w_rqst), .lsu_addr(lsu_w_addr),
        .lsu_bits(lsu_w_bits), .lsu_data(lsu_w_data),
        .late_done({div_done, mul_done, lsu_r_done, ex_stage_inst.late_id}),
        .late_val({div_r, mul_r, lsu_r_data, ex_stage_inst.late_val}),
        .late_exc({div_exc, mul_exc, lsu_r_exc, ex_stage_inst.out_pc.valid}));
    pending_table pending_table_inst(.clk(clk), .rst(rst),
        .flush(wb_stage_inst.recover),
        .in_ex(data_ex_pt), .get_ex(get_ex_pt),
        .out_ex(data_pt_ex),
        .cqid_in(cqid_new), .cqid_out(cqid_old),
        .late_done({div_done, mul_done, lsu_r_done, ex_stage_inst.late_id}),
        .late_val({div_r, mul_r, lsu_r_data, ex_stage_inst.late_val}),
        .rda_id(data_id_ex.valid ? data_id_ex.rda : 0),
        .rda_ex(data_ex_wb.valid ? data_ex_wb.rda : 0));
    lsu lsu_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover),
        .rrqst(lsu_r_rqst), .rbits(lsu_r_bits), .raddr(lsu_r_addr),
        .rdone(lsu_r_done), .rdata(lsu_r_data), .rexcp(lsu_r_exc),
        .wrqst(lsu_w_rqst), .wbits(lsu_w_bits), .waddr(lsu_w_addr), .wdata(lsu_w_data),
        .dcache_r_rqst(dcache_r_rqst), .dcache_r_addr(dcache_r_addr),
        .dcache_r_bits(dcache_r_bits), .dcache_r_done(dcache_r_done),
        .dcache_r_data(dcache_r_data), .dcache_w_rqst(dcache_w_rqst),
        .dcache_w_addr(dcache_w_addr), .dcache_w_bits(dcache_w_bits),
        .dcache_w_data(dcache_w_data), .dcache_w_done(dcache_w_done)
    );
    mul mul_inst(.clk(clk), .rst(rst), .flush(wb_stage_inst.recover), .op(mul_op),
        .rqst(mul_rqst), .a(mul_a), .b(mul_b), .done(mul_done), .r(mul_r), .e(mul_exc));
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
    always_ff @(posedge clk) sent <= rst ? 1'b0 : icache_rqst | sent & ~icache_done;
    always_ff @(posedge clk)
        hold_data <= rst ? 1'b0 : (ena_id ? 1'b0 : icache_done | hold_data);
    always_comb icache_addr = in_pc.pc;
    always_comb icache_flsh = flush;
    always_comb out_id.valid = ~flush & (sent & icache_done | hold_data);
    always_comb out_id.b = in_pc_r.b;
    always_comb out_id.c = icache_data[1:0] != 2'b11;
    always_comb out_id.pc = in_pc_r.pc;
    always_comb out_id.bpc = in_pc_r.bpc;
    always_comb out_id.ir = icache_data;
    always_comb out_pc.valid = ~flush & (sent & icache_done);
    always_comb out_pc.c = icache_data[1:0] != 2'b11;
    always_ff @(posedge clk) in_pc_r <= rst ? 0 : (icache_rqst ? in_pc : in_pc_r);
endmodule

module id_stage(input logic clk, input logic rst, input logic flush,
    input  if_id_t in_if, output logic get_if,
    output id_ex_t out_ex, input logic ena_ex
);
    logic [31:0] ir, op;
    logic [63:0] imm;
    id_ex_t [2:0] out_ex_q;
    logic [2:0][52:0] exop;
    ci2i ci2i_inst(.ci(in_if.ir), .i(ir));
    always_comb for (int i = 0; i < 32; i++)
        op[i] = ir[6:2] == i[4:0];
    always_comb imm =
        {{53{ir[31]}}, ir[30:20]} & {64{
            op[`LOAD] | op[`LOAD_FP] | op[`OP_IMM] |
            op[`OP_IMM_32] | op[`JALR] | op[`SYSTEM]}} | // I type
        {{32{ir[31]}}, ir[31:12], 12'd0} & {64{
            op[`AUIPC] | op[`LUI]}} | // U type
        {{53{ir[31]}}, ir[30:25], ir[11:7]} & {64{
            op[`STORE] | op[`STORE_FP]}} | // S type
        {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} & {64{op[`BRANCH]}} | // B type
        {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} & {64{op[`JAL]}}; // J type
    always_comb begin
        exop[0] = 0;
        exop[0][`EX_ADD] =
            op[`JALR] | op[`AUIPC] | op[`LUI] | op[`JAL] |
            op[`OP_IMM] & ir[14:12] == 3'b000 |
            op[`OP] & ir[14:12] == 3'b000 & ir[31:25] == 7'd0;
        exop[0][`EX_SUB] =
            op[`BRANCH] |
            op[`OP] & ir[14:12] == 3'b000 & ir[31:25] == 7'b0100000;
        exop[0][`EX_SLL] =
            op[`OP_IMM] & ir[14:12] == 3'b001 & ir[31:26] == 6'd0 |
            op[`OP] & ir[14:12] == 3'b001 & ir[31:25] == 7'd0;
        exop[0][`EX_SLT] =
            op[`OP_IMM] & ir[14:12] == 3'b010 |
            op[`OP] & ir[14:12] == 3'b010 & ir[31:25] == 7'd0;
        exop[0][`EX_SLTU] =
            op[`OP_IMM] & ir[14:12] == 3'b011 |
            op[`OP] & ir[14:12] == 3'b011 & ir[31:25] == 7'd0;
        exop[0][`EX_XOR] =
            op[`OP_IMM] & ir[14:12] == 3'b100 |
            op[`OP] & ir[14:12] == 3'b100 & ir[31:25] == 7'd0;
        exop[0][`EX_SRL] =
            op[`OP_IMM] & ir[14:12] == 3'b101 & ir[31:26] == 6'd0 |
            op[`OP] & ir[14:12] == 3'b101 & ir[31:25] == 7'd0;
        exop[0][`EX_SRA] =
            op[`OP_IMM] & ir[14:12] == 3'b101 & ir[31:26] == 6'b010000 |
            op[`OP] & ir[14:12] == 3'b101 & ir[31:25] == 7'b0100000;
        exop[0][`EX_OR] =
            op[`OP_IMM] & ir[14:12] == 3'b110 |
            op[`OP] & ir[14:12] == 3'b110 & ir[31:25] == 7'd0;
        exop[0][`EX_AND] =
            op[`OP_IMM] & ir[14:12] == 3'b111 |
            op[`OP] & ir[14:12] == 3'b111 & ir[31:25] == 7'd0;
        exop[0][`EX_MUL] = op[`OP] & ir[14:12] == 3'b000 & ir[31:25] == 7'b1;
        exop[0][`EX_MULH] = op[`OP] & ir[14:12] == 3'b001 & ir[31:25] == 7'b1;
        exop[0][`EX_MULHSU] = op[`OP] & ir[14:12] == 3'b010 & ir[31:25] == 7'b1;
        exop[0][`EX_MULHU] = op[`OP] & ir[14:12] == 3'b011 & ir[31:25] == 7'b1;
        exop[0][`EX_DIV] = op[`OP] & ir[14:12] == 3'b100 & ir[31:25] == 7'b1;
        exop[0][`EX_DIVU] = op[`OP] & ir[14:12] == 3'b101 & ir[31:25] == 7'b1;
        exop[0][`EX_REM] = op[`OP] & ir[14:12] == 3'b110 & ir[31:25] == 7'b1;
        exop[0][`EX_REMU] = op[`OP] & ir[14:12] == 3'b111 & ir[31:25] == 7'b1;
        exop[0][`EX_ADDW] =
            op[`OP_IMM_32] & ir[14:12] == 3'b000 |
            op[`OP_32] & ir[14:12] == 3'b000 & ir[31:25] == 7'd0;
        exop[0][`EX_SUBW] = op[`OP_32] & ir[14:12] == 3'b000 & ir[31:25] == 7'b0100000;
        exop[0][`EX_SLLW] =
            op[`OP_IMM_32] & ir[14:12] == 3'b001 & ir[31:25] == 7'd0 |
            op[`OP_32] & ir[14:12] == 3'b001 & ir[31:25] == 7'd0;
        exop[0][`EX_SRLW] =
            op[`OP_IMM_32] & ir[14:12] == 3'b101 & ir[31:25] == 7'd0 |
            op[`OP_32] & ir[14:12] == 3'b101 & ir[31:25] == 7'd0;
        exop[0][`EX_SRAW] =
            op[`OP_IMM_32] & ir[14:12] == 3'b101 & ir[31:25] == 7'b0100000 |
            op[`OP_32] & ir[14:12] == 3'b101 & ir[31:25] == 7'b0100000;
        exop[0][`EX_MULW] = op[`OP_32] & ir[14:12] == 3'b000 & ir[31:25] == 7'b1;
        exop[0][`EX_DIVW] = op[`OP_32] & ir[14:12] == 3'b100 & ir[31:25] == 7'b1;
        exop[0][`EX_DIVUW] = op[`OP_32] & ir[14:12] == 3'b101 & ir[31:25] == 7'b1;
        exop[0][`EX_REMW] = op[`OP_32] & ir[14:12] == 3'b110 & ir[31:25] == 7'b1;
        exop[0][`EX_REMUW] = op[`OP_32] & ir[14:12] == 3'b111 & ir[31:25] == 7'b1;
        exop[0][`EX_FADD] = op[`OP_FP] & ir[31:26] == 6'b000000;
        exop[0][`EX_FSUB] = op[`OP_FP] & ir[31:26] == 6'b000010;
        exop[0][`EX_FMUL] = op[`OP_FP] & ir[31:26] == 6'b000100 |
            op[`MADD] & ir[26:25] == 2'd0 |
            op[`MSUB] & ir[26:25] == 2'd0;
        exop[0][`EX_FNMUL] =
            op[`NMADD] & ir[26:25] == 2'd0 |
            op[`NMSUB] & ir[26:25] == 2'd0;
        exop[0][`EX_FDIV] = op[`OP_FP] & ir[31:26] == 6'b000110;
        exop[0][`EX_FSQRT] = op[`OP_FP] & ir[31:26] == 6'b010110 & ir[24:20] == 5'd0;
        exop[0][`EX_FSGNJ] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b000;
        exop[0][`EX_FSGNJN] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b001;
        exop[0][`EX_FSGNJX] = op[`OP_FP] & ir[31:26] == 6'b001000 & ir[14:12] == 3'b010;
        exop[0][`EX_FMIN] = op[`OP_FP] & ir[31:26] == 6'b001010 & ir[14:12] == 3'b000;
        exop[0][`EX_FMAX] = op[`OP_FP] & ir[31:26] == 6'b001010 & ir[14:12] == 3'b001;
        exop[0][`EX_FEQ] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b010;
        exop[0][`EX_FLT] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b001;
        exop[0][`EX_FLE] = op[`OP_FP] & ir[31:26] == 6'b101000 & ir[14:12] == 3'b000;
        exop[0][`EX_FMVFI] = op[`OP_FP] & ir[31:26] == 6'b111000 &
            ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
        exop[0][`EX_FCLASS] = op[`OP_FP] & ir[31:26] == 6'b111000 &
            ir[14:12] == 3'b001 & ir[24:20] == 5'd0;
        exop[0][`EX_FMVIF] = op[`OP_FP] & ir[31:26] == 6'b111100 &
            ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
        exop[0][`EX_FCVTIF] = op[`OP_FP] & ir[31:26] == 6'b110000;
        exop[0][`EX_FCVTFI] = op[`OP_FP] & ir[31:26] == 6'b110100;
        exop[0][`EX_FCVTSD] = op[`OP_FP] & ir[31:25] == 7'b0100000 & ir[24:20] == 5'd1;
        exop[0][`EX_FCVTDS] = op[`OP_FP] & ir[31:25] == 7'b0100001 & ir[24:20] == 5'd0;
    end
    always_comb exop[1] =
        (1 << `EX_ADD)  & {53{op[`AMO] & ir[31:27] == 5'b00001}} |
        (1 << `EX_ADD)  & {53{op[`AMO] & ir[31:27] == 5'b00000}} |
        (1 << `EX_XOR)  & {53{op[`AMO] & ir[31:27] == 5'b00100}} |
        (1 << `EX_AND)  & {53{op[`AMO] & ir[31:27] == 5'b01100}} |
        (1 << `EX_OR)   & {53{op[`AMO] & ir[31:27] == 5'b01000}} |
        (1 << `EX_MIN)  & {53{op[`AMO] & ir[31:27] == 5'b10000}} |
        (1 << `EX_MAX)  & {53{op[`AMO] & ir[31:27] == 5'b10100}} |
        (1 << `EX_MINU) & {53{op[`AMO] & ir[31:27] == 5'b11000}} |
        (1 << `EX_MAXU) & {53{op[`AMO] & ir[31:27] == 5'b11100}} |
        (1 << `EX_FADD) & {53{op[`MADD] | op[`NMADD]}} |
        (1 << `EX_FSUB) & {53{op[`MSUB] | op[`NMSUB]}};
    always_comb exop[2] = (1 << `EX_ADD) & {53{op[`AMO]}};
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
            out_ex_q[0].a <=
                {1'd1, 59'd0, ir[19:15]} & {65{
                    op[`LOAD]   | op[`LOAD_FP]  | op[`OP_IMM] | op[`OP_IMM_32] |
                    op[`STORE]  | op[`STORE_FP] | op[`OP]     | op[`OP_32]     |
                    op[`BRANCH] | op[`AMO]}} |
                {1'd1, 59'd1, ir[19:15]} & {{65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                    op[`OP_FP]}}} |
                {1'd0, in_if.pc} & {65{op[`JALR] | op[`JAL] | op[`AUIPC]}};
            if (exop[0][`EX_FCVTIF] | exop[0][`EX_FCVTFI])
                out_ex_q[0].b <= {60'd0, ir[24:20]};
            else if (exop[0][`EX_FSQRT] | exop[0][`EX_FCVTDS] | exop[0][`EX_FCVTSD] |
                exop[0][`EX_FMVIF] | exop[0][`EX_FMVFI])
                out_ex_q[0].b <= 65'd0;
            else out_ex_q[0].b <=
                {1'd1, 59'd0, ir[24:20]} & {65{
                    op[`OP] | op[`OP_32] | op[`OP_FP] | op[`BRANCH]}} |
                {1'd1, 59'd1, ir[24:20]} & {{65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                    op[`OP_FP]}}} |
                {1'd0, imm} & {65{
                    op[`LOAD]  | op[`LOAD_FP] | op[`OP_IMM] | op[`OP_IMM_32] |
                    op[`AUIPC] | op[`LUI]     | op[`STORE]  | op[`STORE_FP]}} |
                (in_if.c ? 65'd2 : 65'd4) & {65{op[`JAL] | op[`JALR]}};
            out_ex_q[0].exop <= exop[0];
            out_ex_q[0].bmask <= {3{op[`BRANCH]}} &
                {ir[14:13] == 2'b00, ir[14:13] == 2'b10, ir[14:13] == 2'b11};
            out_ex_q[0].bneg <= ir[12];
            out_ex_q[0].j <= op[`JAL] | op[`JALR];
            out_ex_q[0].base <= {1'b0, in_if.pc} & {65{op[`JAL] | op[`BRANCH]}}|
                                {1'b1, 59'd0, ir[19:15]} & {65{op[`JALR]}};
            out_ex_q[0].offset <= imm & {64{op[`JAL] | op[`JALR] | op[`BRANCH]}};
            out_ex_q[0].frm <= ir[14:12];
            out_ex_q[0].fdouble <= ir[25];
            out_ex_q[0].mr <= op[`LOAD] | op[`LOAD_FP] | op[`AMO];
            out_ex_q[0].mw <= op[`STORE] | op[`STORE_FP];
            out_ex_q[0].bits <= ir[14:12];
            out_ex_q[0].rmwa <=
                {2'b0, ir[24:20]} & {7{op[`STORE]}} |
                {2'b1, ir[24:20]} & {7{op[`STORE_FP]}};
            out_ex_q[0].rda <=
                {2'd0, ir[11:7]} & {7{
                    op[`LOAD] | op[`OP_IMM] | op[`AUIPC] | op[`OP_IMM_32] |
                    op[`OP]   | op[`LUI]    | op[`OP_32] | op[`JALR]      |
                    op[`JAL]  | op[`AMO]}} |
                {2'd1, ir[11:7]} & {7{op[`LOAD_FP] | op[`OP_FP]}} |
                {2'd2, 5'd0} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};

            out_ex_q[1].valid <=
                op[`AMO] | op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD];
            out_ex_q[1].a <=
                {1'd1, 59'd0, ir[11:7]} & {65{op[`AMO]}} |
                {1'd1, 59'd3, 5'd0} & {65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};
            out_ex_q[1].b <=
                {1'd1, 59'd0, ir[24:20]} & {65{op[`AMO]}} |
                {1'd1, 59'd1, ir[31:27]} & {65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};
            out_ex_q[1].exop <= exop[1];
            out_ex_q[1].bmask <= 0;
            out_ex_q[1].j <= 0;
            out_ex_q[1].frm <= ir[14:12];
            out_ex_q[1].fdouble <= ir[25];
            out_ex_q[1].mr <= 0;
            out_ex_q[1].mw <= 0;
            out_ex_q[1].rda <=
                {2'd2, 5'd0} & {7{op[`AMO]}} |
                {2'd1, ir[11:7]} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};

            out_ex_q[2].valid <= op[`AMO];
            out_ex_q[2].a <= {1'd1, 59'd0, ir[19:15]} & {65{op[`AMO]}};
            out_ex_q[2].b <= {1'd0, imm} & {65{op[`AMO]}};
            out_ex_q[2].exop <= exop[2];
            out_ex_q[2].bmask <= 0;
            out_ex_q[2].j <= 0;
            out_ex_q[2].mr <= 0;
            out_ex_q[2].mw <= op[`AMO];
            out_ex_q[2].bits <= ir[14:12];
            out_ex_q[2].rmwa <= {2'd2, 5'd0} & {7{op[`AMO]}};
            out_ex_q[2].rda <= 0;
        end else if (ena_ex)
            out_ex_q[1:0] <= {out_ex_q[2], out_ex_q[1]};
endmodule

module ex_stage(input logic clk, input logic rst,
    input id_ex_t in_id, output logic get_id,
    input id_ex_t in_pt,
    output ex_wb_t out_wb, input logic ena_wb,
    output ex_pc_t out_pc, input logic ena_pc, // always enabled
    output id_ex_t out_pt, input logic ena_pt,
    output logic [2:0][6:0] raddr, input logic [2:0][64:0] rvalue,
    input logic [`lgCQSZ:0] cqid_id, input logic [`lgCQSZ:0] cqid_pt,
    output logic [`lgCQSZ:0] mul_rqst, output logic [4:0] mul_op,
    output logic [63:0] mul_a, output logic [63:0] mul_b,
    output logic [`lgCQSZ:0] lsu_rqst,
    output logic [63:0] lsu_addr, output logic [2:0] lsu_bits
);
    id_ex_t in;
    always_comb in = in_pt.valid ? in_pt : in_id;
    always_comb raddr[0] = in.valid & in.a[64] ? in.a[6:0] : 7'd0;
    always_comb raddr[1] = in.valid & in.b[64] ? in.b[6:0] : 7'd0;
    always_comb raddr[2] = in.valid & in.base[64] ? in.base[6:0] : 7'd0;
    logic [52:0] op;
    logic [63:0] a, b;
    logic jump;
    logic [63:0] jpc;
    logic [64:0] sub, res;
    logic [63:0] add, sll, srl, sra;
    logic mul, div;
    logic [2:0] bflag;
    logic ready, mul_valid, lsu_vaild;
    logic [`lgCQSZ:0] late_id;
    logic [64:0] late_val;
    logic [`lgCQSZ:0] cqid;
    always_comb op = in.exop;
    always_comb a = in.a[64] ? rvalue[0][63:0] : in.a[63:0];
    always_comb b = in.b[64] ? rvalue[1][63:0] : in.b[63:0];
    always_comb sub = {1'b0, a} - {1'b0, b};
    always_comb add = a + b;
    always_comb sll = a << b[5:0];
    always_comb srl = a >> b[5:0];
    always_comb sra = $signed($signed(a) >>> b[5:0]);
    always_comb bflag = {~|sub, sub[63], sub[64]}; // zero, negative, carry
    always_comb begin
        out_pt = in;
        out_pt.valid = in.valid & (rvalue[0][64] | rvalue[1][64] | rvalue[2][64]);
        out_pt.a = in.a[64] ? rvalue[0] : in.a;
        out_pt.b = in.b[64] ? rvalue[1] : in.b;
    end
    always_comb get_id = ~in_id.valid | cqid_id[`lgCQSZ] &
        ~in_pt.valid & (out_pt.valid & ena_pt | ~out_pt.valid & ena_wb);
    always_comb ready = (get_id & in_id.valid | in_pt.valid) & ~out_pt.valid;
    always_comb cqid = in_pt.valid ? cqid_pt : cqid_id;
    always_comb mul = op[`EX_MUL] | op[`EX_MULH] | op[`EX_MULHSU] | op[`EX_MULHU] |
                      op[`EX_MULW];
    always_comb mul_valid = ~rst & ready & mul;
    always_comb mul_rqst = {`lgCQSZ+1{mul_valid}} & cqid;
    always_comb mul_op = {op[`EX_MUL], op[`EX_MULH], op[`EX_MULHSU], op[`EX_MULHU],
                          op[`EX_MULW]};
    always_comb {mul_a, mul_b} = {a, b};
    always_comb div = op[`EX_DIV]  | op[`EX_DIVU]  | op[`EX_REM]  | op[`EX_REMU] |
                      op[`EX_DIVW] | op[`EX_DIVUW] | op[`EX_REMW] | op[`EX_REMUW];
    always_comb lsu_vaild = ~rst & ready & in.mr;
    always_ff @(posedge clk)
        if (lsu_vaild) begin
            lsu_rqst <= cqid;
            lsu_addr <= add;
            lsu_bits <= in.bits;
        end else lsu_rqst <= 0;
    always_comb if (out_pt.valid) res = {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
        else if (in.valid & in.mr) res = {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
        else res =
            {65{op[`EX_ADD]}}  & {1'b0, add} |
            {65{op[`EX_SUB]}}  & {1'b0, sub[63:0]} |
            {65{op[`EX_SLL]}}  & {1'b0, sll} |
            {65{op[`EX_SLT]}}  & {64'd0, sub[63]} |
            {65{op[`EX_SLTU]}} & {64'd0, sub[64]} |
            {65{op[`EX_XOR]}}  & {1'b0, a ^ b} | 
            {65{op[`EX_SRL]}}  & {1'b0, srl} |
            {65{op[`EX_SRA]}}  & {1'b0, sra} |
            {65{op[`EX_OR]}}   & {1'b0, a | b} |
            {65{op[`EX_AND]}}  & {1'b0, a & b} |
            {65{op[`EX_MIN]}}  & {1'b0, sub[63] ? a : b} |
            {65{op[`EX_MAX]}}  & {1'b0, sub[63] ? b : a} |
            {65{op[`EX_MINU]}} & {1'b0, sub[64] ? a : b} |
            {65{op[`EX_MAXU]}} & {1'b0, sub[64] ? b : a} |
            {65{op[`EX_ADDW]}} & {1'b0, {32{add[31]}}, add[31:0]} |
            {65{op[`EX_SUBW]}} & {1'b0, {32{sub[31]}}, sub[31:0]} |
            {65{op[`EX_SLLW]}} & {1'b0, {32{sll[31]}}, sll[31:0]} |
            {65{op[`EX_SRLW]}} & {1'b0, {32{srl[31]}}, srl[31:0]} |
            {65{op[`EX_SRAW]}} & {1'b0, {32{sra[31]}}, sra[31:0]} |
            {65{mul}}          & {1'b1, {63-`lgCQSZ{1'd0}}, cqid} |
            {65{div}}          & {1'b1, {63-`lgCQSZ{1'd0}}, cqid};
    always_ff @(posedge clk) if (rst) late_id <= 0;
        else if (in_pt.valid) {late_id, late_val} <= {cqid_pt, res}; else late_id <= 0;
    always_ff @(posedge clk)
        if (rst) out_wb.valid <= 0;
        else if (get_id & in_id.valid) begin
            out_wb.valid <= 1'b1;
            out_wb.rda <= in.rda;
            out_wb.rd <= res;
            if (in.mw) begin
                out_wb.mw <= 1'b1;
                out_wb.mwaddr <= add;
                out_wb.mwbits <= in.bits;
                out_wb.rmwa <= in.rmwa;
            end else out_wb.mw <= 1'b0;
        end else if (ena_wb) out_wb.valid <= 0;
    always_comb jpc =
        (in.base[64] ? rvalue[2][63:0] : in.base[63:0]) + in.offset;
    always_comb jump = in.j | |in.bmask & in.bneg != |(in.bmask & bflag);
    always_ff @(posedge clk)
        if (rst) out_pc.valid <= 1'b0;
        else if (ena_pc)
            if (ready & (jump & ~(in.branch & in.bpc == jpc) | ~jump & in.branch)) begin
                out_pc.valid <= 1'b1;
                out_pc.pc <= in.pc;
                out_pc.npc <= jump ? jpc : in.pc + (in.c ? 64'd2 : 64'd4);
            end else out_pc.valid <= 1'b0;
endmodule

module wb_stage(input logic clk, input logic rst,
    input ex_wb_t in_ex, output logic get_ex,
    input logic [2:0][6:0] raddr, output logic [2:0][64:0] rvalue,
    output logic [`lgCQSZ:0] cqid,
    output logic lsu_rqst, output logic [63:0] lsu_addr,
    output logic [2:0] lsu_bits, output logic [64:0] lsu_data,
    input logic [`LATENUM-1:0][`lgCQSZ:0] late_done,
    input logic [`LATENUM-1:0][64:0] late_val,
    input logic [`LATENUM-1:0] late_exc
);
    // register number: 00_xxxxx -> integer, 01_xxxxx -> float, 10_00000 -> tmp
    logic [64:0][`lgCQSZ:0] regscqid;
    logic [64:0][63:0] regsval; // registers holding valid values, committed inorder
    logic [`lgCQSZ-1:0] cqfront, cqfrontp1, cqfrontp2, cqrear, cqrearp1; // commit queue
    logic cqempty, cqfull, cqpush, cqpop1, cqpop2;
    logic [`CQSZ-1:0] cqexc;
    logic [`CQSZ-1:0][6:0] cqrda;
    logic [`CQSZ-1:0][64:0] cqval;
    logic recover;
    always_comb get_ex = ~in_ex.valid | cqpush;
    always_comb cqfrontp1 = cqfront + 1;
    always_comb cqfrontp2 = cqfront + 2;
    always_comb cqrearp1 = cqrear + 1;
    always_comb if (cqfull) cqid = 0;
        else if (cqpush & ~cqpop1 & cqfront == cqrearp1) cqid = 0;
        else cqid = {1'b1, cqpush ? cqrearp1 : cqrear};
    always_comb cqpush = in_ex.valid & (~cqfull | cqpop1);
    always_comb cqpop1 = ~cqval[cqfront][64] & ~cqexc[cqfront] & ~cqempty;
    always_comb cqpop2 = ~cqval[cqfrontp1][64] & ~cqexc[cqfrontp1] &
        cqfrontp1 != cqrear & cqpop1; // more than one value
    always_comb recover = ~cqempty & cqexc[cqfront];
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
            if (cqpush) {cqval[cqrear], cqrda[cqrear]} <= {in_ex.rd, in_ex.rda};
            for (int i = 0; i < `LATENUM; i++) if (late_done[i][`lgCQSZ]) begin
                cqexc[late_done[i][`lgCQSZ-1:0]] <= late_exc[i];
                cqval[late_done[i][`lgCQSZ-1:0]] <= late_val[i];
            end
            assert((cqempty | cqfull) == (cqfront == cqrear));
            if (cqpush & in_ex.rd[64]) assert(in_ex.rd[3:0] == cqrear);
        end
    always_ff @(posedge clk) if (rst | recover) {regscqid, regsval[0]} <= 0;
        else begin // commit
            if (cqpop1 & |cqrda[cqfront])
                regsval[cqrda[cqfront]] <= cqval[cqfront][63:0];
            if (cqpop2 & cqrda[cqfrontp1] != 0)
                regsval[cqrda[cqfrontp1]] <= cqval[cqfrontp1][63:0];
            if (cqpop1 & regscqid[cqrda[cqfront]][`lgCQSZ-1:0] == cqfront)
                regscqid[cqrda[cqfront]] <= 0;
            if (cqpop2 & regscqid[cqrda[cqfrontp1]][`lgCQSZ-1:0] == cqfrontp1)
                regscqid[cqrda[cqfrontp1]] <= 0;
            if (cqpush & |in_ex.rda) regscqid[in_ex.rda] <= {1'b1, cqrear};
        end
    always_comb for (int i = 0; i < 3; i++)
        if (in_ex.valid & raddr[i] == in_ex.rda) rvalue[i] = in_ex.rd;
        else if (regscqid[raddr[i]][`lgCQSZ]) begin
            rvalue[i] = cqval[regscqid[raddr[i]][`lgCQSZ-1:0]];
            for (int j = 0; j < `LATENUM; j++)
                if (rvalue[i][64] & late_done[j] == rvalue[i][`lgCQSZ:0])
                    rvalue[i] = late_val[j];
        end else rvalue[i] = {1'b0, regsval[raddr[i]]};
    always_comb lsu_rqst = get_ex & in_ex.valid & in_ex.mw;
    always_comb lsu_addr = in_ex.mwaddr;
    always_comb lsu_bits = in_ex.mwbits;
    // always_comb lsu_data = regs[in_ex.rmwa];
endmodule

module pending_table(input logic clk, input logic rst, input logic flush,
    input id_ex_t in_ex, output logic get_ex,
    output id_ex_t out_ex,
    input logic [`lgCQSZ:0] cqid_in, output logic [`lgCQSZ:0] cqid_out,
    input logic [`LATENUM-1:0][`lgCQSZ:0] late_done,
    input logic [`LATENUM-1:0][64:0] late_val,
    input logic [6:0] rda_id, input logic [6:0] rda_ex
);
    logic [`PTSZ-1:0] in, out;
    logic [`PTSZ-1:0][`lgCQSZ:0] id;
    id_ex_t [`PTSZ-1:0] data;
    logic [`PTSZ:0][64:0] fwd_a, fwd_b;
    id_ex_t in_ex_fwd;
    always_comb get_ex = |in;
    always_comb for (int i = 0; i <= `PTSZ; i++)
        if (i == `PTSZ | id[i][`lgCQSZ]) begin
            if (i == `PTSZ) {fwd_a[i], fwd_b[i]} = {in_ex.a, in_ex.b};
            else {fwd_a[i], fwd_b[i]} = {data[i].a, data[i].b};
            for (int j = 0; j < `LATENUM; j++)
                if (fwd_a[i][64] & late_done[j] == fwd_a[i][`lgCQSZ:0])
                    fwd_a[i] = late_val[j];
            for (int j = 0; j < `LATENUM; j++)
                if (fwd_b[i][64] & late_done[j] == fwd_b[i][`lgCQSZ:0])
                    fwd_b[i] = late_val[j];
        end else {fwd_a[i], fwd_b[i]} = {1'b1, 64'd0, 1'b1, 64'd0};
    always_comb begin
        {in, out} = 0;
        for (int i = `PTSZ-1; i >= 0; i--)
            if (~fwd_a[i][64] & ~fwd_b[i][64]) out = 1 << i;
        for (int i = `PTSZ-1; i >= 0; i--) if (~id[i][`lgCQSZ]) in = 1 << i;
        in_ex_fwd = in_ex;
        {in_ex_fwd.a, in_ex_fwd.b} = {fwd_a[`PTSZ], fwd_b[`PTSZ]};
    end
    always_ff @(posedge clk) if (rst | flush) {out_ex.valid, cqid_out} <= 0;
        else begin {out_ex.valid, cqid_out} <= 0;
            for (int i = 0; i < `PTSZ; i++) if (out[i]) begin
                cqid_out <= id[i];
                out_ex <= data[i];
                out_ex.a <= fwd_a[i];
                out_ex.b <= fwd_b[i];
                if (data[i].rda == rda_id) out_ex.rda <= 0;
            end end
    always_ff @(posedge clk)
        if (rst | flush) id <= 0;
        else for (int i = 0; i < `PTSZ; i++)
            if (in_ex_fwd.valid & in[i]) {id[i], data[i]} <= {cqid_in, in_ex_fwd};
            else if (out[i]) id[i] <= 0;
            else {data[i].a, data[i].b} <= {fwd_a[i], fwd_b[i]};
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

module lsu(input logic clk, input logic rst, input logic flush,
    input logic [`lgCQSZ:0] rrqst, input logic [2:0] rbits,
    input logic [63:0] raddr,
    output logic [`lgCQSZ:0] rdone, output logic [64:0] rdata,
    output logic rexcp,
    input logic wrqst, input logic [2:0] wbits,
    input logic [63:0] waddr, input logic [64:0] wdata,
    output logic [`lgCQSZ:0] dcache_r_rqst,
    output logic      [63:0] dcache_r_addr,
    output logic       [2:0] dcache_r_bits,
    input  logic [`lgCQSZ:0] dcache_r_done,
    input  logic      [63:0] dcache_r_data,
    output logic        dcache_w_rqst,
    output logic [63:0] dcache_w_addr,
    output logic  [2:0] dcache_w_bits,
    output logic [63:0] dcache_w_data,
    input  logic        dcache_w_done
);
    // logic [`SQSZ-1:0][63:0] sqaddr;
    // logic [`SQSZ-1:0][64:0] sqdata;
    always_comb dcache_r_rqst = rrqst;
    always_comb dcache_r_addr = raddr;
    always_comb dcache_r_bits = rbits;
    always_comb rdone = dcache_r_done;
    always_comb rdata = {1'b0, dcache_r_data};
    always_comb rexcp = 0;
    always_comb dcache_w_rqst = wrqst;
    always_comb dcache_w_addr = waddr;
    always_comb dcache_w_bits = wbits;
    always_comb dcache_w_data = wdata[63:0];
endmodule

module mul(input logic clk, input logic rst, input logic flush,
    input logic [`lgCQSZ:0] rqst, input logic [4:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
`define latency 10
    logic [`latency-1:0][63:0] r_q;
    logic [`latency-1:0][`lgCQSZ:0] valid;
    logic [127:0] res;
    always_comb res = {64'd0, a} * {64'd0, b};
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`latency'd0}};
        else begin
            valid <= {rqst, valid[`latency-1:1]};
            r_q <= {res[63:0], r_q[`latency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
endmodule
