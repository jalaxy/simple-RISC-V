`define RST_PC 64'h400000 // reset pc
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
`define EX_ADD    6'd0 // Execution stage operations
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
`define ID_LSU 5'd0 // Async component ID
`define ID_MUL 5'd1
`define ID_DIV 5'd2
`define MUL_MUL    3'd0
`define MUL_MULH   3'd1
`define MUL_MULHSU 3'd2
`define MUL_MULHU  3'd3
`define MUL_MULW   3'd4

typedef struct packed { logic valid; logic [63:0] pc; } pc_if0_t;
typedef struct packed { logic valid; logic [64:0] base; logic [63:0] offset; } xx_pc_t;
typedef struct packed {
    logic valid;
    logic [63:0] pc;
    logic [63:0] data;
    logic [4:0][7:0] offset;
} if0_if1_t;
typedef struct packed {
    logic valid;
    logic [31:0] ir;
    logic [63:0] pc;
    logic compressed;
} if1_id_t;
typedef struct packed {
    logic valid;
    logic [64:0] a, b; // EX stage info
    logic [52:0] exop;
    logic [2:0] frm, bmask;
    logic fdouble, bneg;
    logic [63:0] bbase, boffset;
    logic mr, mw; // MA stage info
    logic [2:0] bits;
    logic [6:0] rmwa;
    logic [6:0] rda; // WB stage info
} id_ex_t;
typedef struct packed {
    logic valid;
    logic [6:0] rda;
    logic [64:0] rd;
    logic mw;
    logic [63:0] mwaddr;
    logic [2:0] mwbits;
    logic [6:0] rmwa;
} ex_wb_t;

module pipeline(
    input  logic        clk,
    input  logic        rst,

    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    input  logic        icache_done,
    input  logic [63:0] icache_data,

    output logic        dcache_r_rqst,
    output logic [63:0] dcache_r_addr,
    output logic  [2:0] dcache_r_bits,
    input  logic        dcache_r_done,
    input  logic [63:0] dcache_r_data,

    output logic        dcache_w_rqst,
    output logic [63:0] dcache_w_addr,
    output logic  [2:0] dcache_w_bits,
    output logic [63:0] dcache_w_data,
    input  logic        dcache_w_done
);
    xx_pc_t data_if0_pc;    logic get_if0_pc;
    pc_if0_t data_pc_if0;   logic get_pc_if0;
    if0_if1_t data_if0_if1; logic get_if0_if1;
    if1_id_t data_if1_id;   logic get_if1_id;
    id_ex_t data_id_ex;     logic get_id_ex;
    ex_wb_t data_ex_wb;     logic get_ex_wb;
    xx_pc_t data_id_pc;     logic get_id_pc;
    xx_pc_t data_ex_pc;     logic get_ex_pc;
    id_ex_t data_ex_pd;     logic get_ex_pd;
    ex_wb_t data_wb_pd;     logic get_wb_pd;
    id_ex_t data_pd_ex;     logic get_pd_ex;
    ex_wb_t data_pd_wb;     logic get_pd_wb;
    logic [2:0][6:0] raddr; logic [2:0][64:0] rvalue;
    logic mul_rqst, mul_done; logic [4:0] mul_op;
    logic div_rqst, div_done; logic [4:0] div_op;
    logic [63:0] mul_a, mul_b, mul_r;
    logic [63:0] div_a, div_b, div_r;

    pc_stage pc_stage_inst(.clk(clk), .rst(rst),
        .in_if0(data_if0_pc), .get_if0(get_if0_pc),
        .in_id(data_id_pc), .get_id(get_id_pc),
        .in_ex(data_ex_pc), .get_ex(get_ex_pc),
        .out_if0(data_pc_if0), .ena_if0(get_pc_if0),
        .raddr(raddr[2]), .rvalue(rvalue[2]));
    if0_stage if0_stage_inst(.clk(clk), .rst(rst | data_id_pc.valid | data_ex_pc.valid),
        .in_pc(data_pc_if0), .get_pc(get_pc_if0),
        .out_pc(data_if0_pc), .ena_pc(get_if0_pc),
        .out_if1(data_if0_if1), .ena_if1(get_if0_if1),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_done(icache_done), .icache_data(icache_data));
    if1_stage if1_stage_inst(.clk(clk), .rst(rst | data_id_pc.valid | data_ex_pc.valid),
        .in_if0(data_if0_if1), .get_if0(get_if0_if1),
        .out_id(data_if1_id), .ena_id(get_if1_id));
    id_stage id_stage_inst(.clk(clk), .rst(rst | data_id_pc.valid | data_ex_pc.valid),
        .in_if1(data_if1_id), .get_if1(get_if1_id),
        .out_ex(data_id_ex), .ena_ex(get_id_ex),
        .out_pc(data_id_pc), .ena_pc(get_id_pc));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst | data_ex_pc.valid),
        .in_id(data_id_ex), .get_id(get_id_ex),
        .in_pd(data_pd_ex), .get_pd(get_pd_ex),
        .out_wb(data_ex_wb), .ena_wb(get_ex_wb),
        .out_pc(data_ex_pc), .ena_pc(get_ex_pc),
        .out_pd(data_ex_pd), .ena_pd(get_ex_pd),
        .raddr(raddr[1:0]), .rvalue(rvalue[1:0]),
        .mul_rqst(mul_rqst), .mul_op(mul_op),
        .mul_a(mul_a), .mul_b(mul_b),
        .dcache_r_rqst(dcache_r_rqst), .dcache_r_addr(dcache_r_addr),
        .dcache_r_bits(dcache_r_bits));
    wb_stage wb_stage_inst(.clk(clk), .rst(rst),
        .in_ex(data_ex_wb), .get_ex(get_ex_wb),
        .in_pd(data_pd_wb), .get_pd(get_pd_wb),
        .out_pd(data_wb_pd), .ena_pd(get_wb_pd),
        .raddr(raddr), .rvalue(rvalue),
        .dcache_w_rqst(dcache_w_rqst), .dcache_w_addr(dcache_w_addr),
        .dcache_w_bits(dcache_w_bits), .dcache_w_data(dcache_w_data));
    pending_queue pending_queue_inst(.clk(clk), .rst(rst),
        .in_ex(data_ex_pd), .get_ex(get_ex_pd),
        .in_wb(data_wb_pd), .get_wb(get_wb_pd),
        .out_ex(data_pd_ex), .ena_ex(get_pd_ex),
        .out_wb(data_pd_wb), .ena_wb(get_pd_wb),
        .async_done({div_done, mul_done, dcache_r_done}),
        .async_val({div_r, mul_r, dcache_r_data}));
    mul mul_inst(.clk(clk), .rst(rst), .op(mul_op),
        .rqst(mul_rqst), .a(mul_a), .b(mul_b), .done(mul_done), .r(mul_r));
endmodule

module pc_stage(input logic clk, input logic rst,
    input  xx_pc_t in_if0,   output logic get_if0,
    input  xx_pc_t in_id,    output logic get_id,
    input  xx_pc_t in_ex,    output logic get_ex,
    output pc_if0_t out_if0, input  logic ena_if0,
    output logic [6:0] raddr, input logic [64:0] rvalue
);
    always_comb raddr = in_id.base[64] ? in_id.base[6:0] : 7'd0;
    logic [63:0] pc, offset;
    logic [64:0] base;
    logic valid;
    always_comb out_if0.pc = pc;
    always_comb out_if0.valid = valid;
    always_comb if (in_ex.valid) {base, offset} = {in_ex.base, in_ex.offset};
        else if (in_id.valid) {base, offset} =
            {in_id.base[64] ? rvalue : in_id.base, in_id.offset};
        else if (in_if0.valid) {base, offset} = {in_if0.base, in_if0.offset};
        else {base, offset} = {1'b0, pc, 64'd0};
    always_ff @(posedge clk)
        if (rst) begin pc <= `RST_PC; valid <= 1'b1; end
        else if (ena_if0)
            if (in_if0.valid | in_id.valid | in_ex.valid) begin
                pc <= base[63:0] + offset;
                valid <= ~base[64];
            end else valid <= 1'b0;
    always_comb get_if0 = ena_if0 | ~in_if0.valid;
    always_comb get_id = ena_if0 | ~in_id.valid;
    always_comb get_ex = ena_if0 | ~in_ex.valid;
endmodule

module if0_stage(input logic clk, input logic rst,
    input  pc_if0_t  in_pc,   output logic get_pc,
    output xx_pc_t  out_pc,   input  logic ena_pc,
    output if0_if1_t out_if1, input  logic ena_if1,
    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    input  logic        icache_done,
    input  logic [63:0] icache_data
);
    logic [7:0] len0, len1, len2, len3, off0, off1, off2, off3, off4; // in bit
    logic [63:0] pc;
    logic sent, hold_data, hold_delta;
    always_comb get_pc = ena_if1 /*& ena_pc*/ & ~(sent & ~icache_done) | ~in_pc.valid;
    always_comb icache_rqst = ~rst & get_pc & in_pc.valid;
    always_ff @(posedge clk) sent <= rst ? 1'b0 : icache_rqst | sent & ~icache_done;
    always_ff @(posedge clk)
        hold_data <= rst ? 1'b0 : (ena_if1 ? 1'b0 : icache_done | hold_data);
    always_ff @(posedge clk)
        hold_delta <= rst ? 1'b0 : (ena_pc ? 1'b0 : icache_done | hold_delta);
    always_comb icache_addr = in_pc.pc;
    always_comb out_if1.valid = sent & icache_done | hold_data;
    always_comb out_if1.pc = pc;
    always_comb out_if1.data = icache_data;
    always_comb out_if1.offset = {off4, off3, off2, off1, off0};
    always_comb off0 = 8'd0;
    always_comb off1 = off0 + len0;
    always_comb off2 = off1 + len1;
    always_comb off3 = off2 + len2;
    always_comb off4 = off3 + len3;
    always_comb len0 = icache_data[off0[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len1 = icache_data[off1[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len2 = icache_data[off2[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len3 = icache_data[off3[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb out_pc.valid = sent & icache_done | hold_delta;
    always_comb out_pc.base = {1'b0, pc};
    always_comb if (off4 <= 8'd64) out_pc.offset = {59'd0, off4[7:3]};
        else if (off3 <= 8'd64) out_pc.offset = {59'd0, off3[7:3]};
        else if (off2 <= 8'd64) out_pc.offset = {59'd0, off2[7:3]};
        else if (off1 <= 8'd64) out_pc.offset = {59'd0, off1[7:3]};
        else out_pc.offset = 64'd0;
    always_ff @(posedge clk) pc <= rst ? 64'd0 : (icache_rqst ? in_pc.pc : pc);
endmodule

module if1_stage(input logic clk, input logic rst,
    input  if0_if1_t in_if0, output logic get_if0,
    output if1_id_t  out_id, input  logic ena_id
);
    logic [63:0] base;
    logic [3:0][7:0] offset;
    logic [3:0][31:0] ir, ic, ix; // instruction compressed and extended
    logic [3:0] ena_q, compressed;
    always_comb get_if0 = ena_id & ~ena_q[1] | ~in_if0.valid;
    always_comb out_id.pc = base + {59'd0, offset[0][7:3]};
    always_comb out_id.ir = ir[0];
    always_comb out_id.compressed = compressed[0];
    always_comb out_id.valid = ena_q[0];
    always_comb for (int i = 0; i < 4; i++)
        ic[i] = in_if0.data[in_if0.offset[i][5:0]+31-:32];
    always_ff @(posedge clk)
        if (rst) ena_q <= 0;
        else if (get_if0 & in_if0.valid) begin
            ena_q <= {in_if0.offset[4] <= 8'd64, in_if0.offset[3] <= 8'd64,
                      in_if0.offset[2] <= 8'd64, in_if0.offset[1] <= 8'd64};
            base <= in_if0.pc;
            offset <= in_if0.offset[3:0];
            ir <= ix;
            compressed <= {ic[3][1:0] != 2'b11, ic[2][1:0] != 2'b11,
                           ic[1][1:0] != 2'b11, ic[0][1:0] != 2'b11};
        end else if (ena_id) begin
            ena_q <= ena_q >> 1;
            compressed <= compressed >> 1;
            offset <= offset >> 8;
            ir <= ir >> 32;
        end
    ci2i ci2i_inst1(.ci(ic[0]), .i(ix[0]));
    ci2i ci2i_inst2(.ci(ic[1]), .i(ix[1]));
    ci2i ci2i_inst3(.ci(ic[2]), .i(ix[2]));
    ci2i ci2i_inst4(.ci(ic[3]), .i(ix[3]));
endmodule

module id_stage(input logic clk, input logic rst,
    input  if1_id_t in_if1, output logic get_if1,
    output id_ex_t  out_ex, input  logic ena_ex,
    output xx_pc_t  out_pc, input  logic ena_pc
);
    logic [31:0] ir, op;
    logic [63:0] imm;
    id_ex_t [2:0] out_ex_q;
    logic [2:0][52:0] exop;
    logic [2:0] ena_q;
    always_comb ir = in_if1.ir;
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
    always_comb get_if1 = ena_ex & ~ena_q[1] | ~in_if1.valid;
    always_comb out_ex = out_ex_q[0];
    always_ff @(posedge clk)
        if (rst)
            out_ex_q <= 0;
        else if (get_if1 & in_if1.valid) begin
            out_ex_q[0].valid <= 1'b1;
            out_ex_q[0].a <=
                {1'd1, 59'd0, ir[19:15]} & {65{
                    op[`LOAD]   | op[`LOAD_FP]  | op[`OP_IMM] | op[`OP_IMM_32] |
                    op[`STORE]  | op[`STORE_FP] | op[`OP]     | op[`OP_32]     |
                    op[`BRANCH] | op[`AMO]}} |
                {1'd1, 59'd1, ir[19:15]} & {{65{
                    op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                    op[`OP_FP]}}} |
                {1'd0, in_if1.pc} & {65{op[`JALR] | op[`JAL] | op[`AUIPC]}};
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
                (in_if1.compressed ? 65'd2 : 65'd4) & {65{op[`JAL] | op[`JALR]}};
            out_ex_q[0].exop <= exop[0];
            out_ex_q[0].bmask <= {3{op[`BRANCH]}} &
                {ir[14:13] == 2'b00, ir[14:13] == 2'b10, ir[14:13] == 2'b11};
            out_ex_q[0].bneg <= ir[12];
            out_ex_q[0].bbase <= in_if1.pc;
            out_ex_q[0].boffset <= imm;
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
            out_ex_q[2].mr <= 0;
            out_ex_q[2].mw <= op[`AMO];
            out_ex_q[2].bits <= ir[14:12];
            out_ex_q[2].rmwa <= {2'd2, 5'd0} & {7{op[`AMO]}};
            out_ex_q[2].rda <= 0;
        end else if (ena_ex) begin
            ena_q <= ena_q >> 1;
            out_ex_q[1:0] <= {out_ex_q[2], out_ex_q[1]};
        end
    always_ff @(posedge clk)
        if (rst) out_pc.valid <= 0;
        else if (ena_pc)
            if (in_if1.valid & (op[`JAL] | op[`JALR])) begin
                out_pc.valid <= 1'b1;
                out_pc.base <= op[`JAL] ? {1'b0, in_if1.pc} : {1'b1, 59'd0, ir[19:15]};
                out_pc.offset <= imm;
            end else out_pc.valid <= 1'b0;
endmodule

module ex_stage(input logic clk, input logic rst,
    input id_ex_t in_id, output logic get_id,
    input id_ex_t in_pd, output logic get_pd,
    output ex_wb_t out_wb, input  logic ena_wb,
    output xx_pc_t out_pc, input  logic ena_pc,
    output id_ex_t out_pd, input  logic ena_pd,
    output logic [1:0][6:0] raddr, input logic [1:0][64:0] rvalue,
    output logic mul_rqst, output logic [4:0] mul_op,
    output logic [63:0] mul_a, output logic [63:0] mul_b,
    output logic        dcache_r_rqst,
    output logic [63:0] dcache_r_addr,
    output logic  [2:0] dcache_r_bits
);
    always_comb raddr[0] = in_id.valid & in_id.a[64] ? in_id.a[6:0] : 7'd0;
    always_comb raddr[1] = in_id.valid & in_id.b[64] ? in_id.b[6:0] : 7'd0;
    logic [52:0] op;
    logic valid;
    logic [63:0] a, b;
    logic [64:0] sub;
    logic [63:0] add, sll, srl, sra;
    logic mul, div;
    logic [2:0] bflag;
    always_comb op = in_id.exop;
    always_comb valid = in_id.valid & ~rvalue[0][64] & ~rvalue[1][64];
    always_comb a = in_id.a[64] ? rvalue[0][63:0] : in_id.a[63:0];
    always_comb b = in_id.b[64] ? rvalue[1][63:0] : in_id.b[63:0];
    always_comb sub = {1'b0, a} - {1'b0, b};
    always_comb add = a + b;
    always_comb sll = a << b[5:0];
    always_comb srl = a >> b[5:0];
    always_comb sra = $signed($signed(a) >>> b[5:0]);
    always_comb bflag = {~|sub, sub[63], sub[64]}; // zero, negative, carry
    always_comb get_id = ~valid | ena_wb & ~in_pd.valid;
    always_comb get_pd = 1'b1;
    always_comb mul = op[`EX_MUL] | op[`EX_MULH] | op[`EX_MULHSU] | op[`EX_MULHU] |
                      op[`EX_MULW];
    always_comb mul_rqst = ~rst & get_id & valid & mul;
    always_comb mul_op = {op[`EX_MUL], op[`EX_MULH], op[`EX_MULHSU], op[`EX_MULHU],
                          op[`EX_MULW]};
    always_comb {mul_a, mul_b} = {a, b};
    always_comb div = op[`EX_DIV]  | op[`EX_DIVU]  | op[`EX_REM]  | op[`EX_REMU] |
                      op[`EX_DIVW] | op[`EX_DIVUW] | op[`EX_REMW] | op[`EX_REMUW];
    always_ff @(posedge clk)
        if (~rst & get_id & valid & in_id.mr) begin
            dcache_r_rqst <= 1'b1;
            dcache_r_addr <= add;
            dcache_r_bits <= in_id.bits;
        end else dcache_r_rqst <= 1'b0;
    always_ff @(posedge clk)
        if (rst) out_wb.valid <= 1'b0;
        else if (ena_wb)
            if (valid) begin
                out_wb.valid <= 1'b1;
                out_wb.rda <= in_id.rda;
                if (in_id.mr)
                    out_wb.rd <= {1'b1, 59'd0, `ID_LSU};
                else out_wb.rd <=
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
                    {65{mul}}          & {1'b1, 59'd0, `ID_MUL} | 
                    {65{div}}          & {1'b1, 59'd0, `ID_DIV};
                if (in_id.mw) begin
                    out_wb.mw <= 1'b1;
                    out_wb.mwaddr <= add;
                    out_wb.mwbits <= in_id.bits;
                    out_wb.rmwa <= in_id.rmwa;
                end else out_wb.mw <= 1'b0;
            end else out_wb.valid <= 1'b0;
    always_ff @(posedge clk)
        if (rst) out_pc.valid <= 1'b0;
        else if (ena_pc)
            if (valid & in_id.bneg != |(in_id.bmask & bflag)) begin
                out_pc.valid <= 1'b1;
                out_pc.base <= {1'b0, in_id.bbase};
                out_pc.offset <= in_id.boffset;
            end else out_pc.valid <= 1'b0;
endmodule

module wb_stage(input logic clk, input logic rst,
    input ex_wb_t in_ex, output logic get_ex,
    input ex_wb_t in_pd, output logic get_pd,
    output ex_wb_t out_pd, input logic ena_pd,
    input logic [2:0][6:0] raddr, output logic [2:0][64:0] rvalue,
    output logic        dcache_w_rqst,
    output logic [63:0] dcache_w_addr,
    output logic  [2:0] dcache_w_bits,
    output logic [63:0] dcache_w_data
);
    logic [64:0][64:0] regs; // 00_xxxxx: integer, 01_xxxxx: float, 10_00000: tmp
    always_comb get_pd = 1'b1;
    always_comb get_ex = ~in_ex.valid | ~(~ena_pd & out_pd.valid) & ~in_pd.valid;
    always_comb for (int i = 0; i < 4; i++)
        if (raddr[i] == 0) rvalue[i] = 65'd0;
        else if (in_ex.valid & raddr[i] == in_ex.rda) rvalue[i] = in_ex.rd;
        else if (in_pd.valid & raddr[i] == in_pd.rda) rvalue[i] = in_pd.rd;
        else rvalue[i] = regs[raddr[i]];
    always_comb out_pd.valid = in_ex.valid & in_ex.rda != 0 & in_ex.rd[64];
    always_comb out_pd.rd = in_ex.rd;
    always_comb out_pd.rda = in_ex.rda;
    always_ff @(posedge clk) begin
        if (in_ex.valid & in_ex.rda != 0)
            regs[in_ex.rda] <= in_ex.rd;
        if (in_pd.valid & in_pd.rda != 0)
            regs[in_pd.rda] <= in_pd.rd; end
    always_comb dcache_w_rqst = get_ex & in_ex.valid & in_ex.mw;
    always_comb dcache_w_addr = in_ex.mwaddr;
    always_comb dcache_w_bits = in_ex.mwbits;
    always_comb dcache_w_data = regs[in_ex.rmwa][63:0];
endmodule

module pending_queue(input logic clk, input logic rst,
    input id_ex_t in_ex, output logic get_ex,
    input ex_wb_t in_wb, output logic get_wb,
    output id_ex_t out_ex, input logic ena_ex, // always enabled
    output ex_wb_t out_wb, input logic ena_wb,
    input logic [2:0] async_done, input logic [2:0][63:0] async_val
);

// TODO: The queue structure is unnecessary as the structure is actually a DAG.

    parameter sz = 8, len = 339;
    logic [sz-1:0] ena_q, ena_t;
    logic [sz-1:0][sz-1:0] data_q;
    logic [sz-1:0][len:0] data_t; // stage(1), data(len)
    logic [sz-1:0] ready_q, ready_t;
    logic [sz-1:0] in1_q, in2_q, out1_q, in1_t, in2_t, out1_t;
    logic [sz:0] gt0_q/*verilator split_var*/,
                 gt0_t/*verilator split_var*/, gt1_t/*verilator split_var*/;
    logic [sz-1:0][sz-1:0] data_out1_q/*verilator split_var*/;
    logic [sz-1:0][len:0] data_out1_t/*verilator split_var*/;
    logic [sz:0] tail_q;
    always_comb get_ex = ~in_ex.valid | |in2_t | |in1_t & ~in_wb.valid;
    always_comb get_wb = ~in_wb.valid | |in1_t;
    always_comb for (int i = 0; i <= sz; i++)
        if (i == 0) tail_q[i] = ~ena_q[0];
        else if (i == sz) tail_q[i] = ena_q[sz - 1];
        else tail_q[i] = ena_q[i - 1] & ~ena_q[i];
    for (genvar i = 0; i < sz; i++) begin
        id_ex_t data_ex;
        ex_wb_t data_wb;
        always_comb data_ex = data_t[i][338:0];
        always_comb data_wb = data_t[i][147:0];
        always_comb if (data_t[i][len])
            ready_t[i] = ena_t[i] & ~data_wb.rd[64] | async_done[data_wb.rd[1:0]]; else
            ready_t[i] = ena_t[i] & (~data_ex.a[64] | async_done[data_ex.a[1:0]]) &
                                    (~data_ex.b[64] | async_done[data_ex.b[1:0]]);
    end
    always_comb begin
        out_wb = data_out1_t[sz-1][147:0];
        if (out_wb.rd[64]) out_wb.rd = {1'b0, async_val[out_wb.rd[1:0]]};
    end

    always_comb for (int i = 0; i < sz; i++)
        ready_q[i] = |(ready_t & data_q[i]);
    always_comb gt0_q[0] = 1'b0;
    always_comb gt0_t[0] = 1'b0;
    always_comb gt1_t[0] = 1'b0;
    for (genvar i = 1; i <= sz; i++) begin
        always_comb gt0_q[i] = gt0_q[i - 1] | ready_q[i - 1];
        always_comb gt0_t[i] = gt0_t[i - 1] | ~ena_t[i - 1] | out1_t[i - 1];
        always_comb gt1_t[i] = gt1_t[i - 1] | gt0_t[i - 1] & ~ena_t[i - 1];
        always_comb in1_q[i - 1] = ~gt0_q[i - 1] & tail_q[i - 1] | gt0_q[i] & tail_q[i];
        always_comb in2_q[i - 1] = i != 1 & in1_q[i - 1];
        always_comb out1_q[i - 1] = ~gt0_q[i - 1] & gt0_q[i];
        always_comb in1_t[i - 1] = ~gt0_t[i - 1] & gt0_t[i];
        always_comb in2_t[i - 1] = ~gt1_t[i - 1] & gt1_t[i]; end
    always_comb data_out1_q[0] = data_q[0] & {sz{out1_q[0]}};
    for (genvar i = 1; i < sz; i++)
        always_comb data_out1_q[i] = data_out1_q[i - 1] | data_q[i] & {sz{out1_q[i]}};
    always_comb out1_t = data_out1_q[sz - 1];
    always_comb data_out1_t[0] = data_t[0] & {(len+1){out1_t[0]}};
    for (genvar i = 1; i < sz; i++)
        always_comb data_out1_t[i] = data_out1_t[i - 1] | data_t[i] & {(len+1){out1_t[i]}};
    for (genvar i = 0; i < sz; i++) always_ff @(posedge clk)
        if (rst) ena_q[i] <= 1'b0;
        else if ((in_ex.valid & get_ex | in_wb.valid & get_wb) & in1_q[i]) begin
            ena_q[i] <= 1'b1;
            data_q[i] <= in1_t;
        end else if ((in_ex.valid & get_ex & in_wb.valid & get_wb) & in2_q[i]) begin
            ena_q[i] <= 1'b1;
            data_q[i] <= in2_t;
        end else if (gt0_q[i + 1]) begin
            ena_q[i] <= i == sz - 1 ? 0 : ena_q[i + 1];
            data_q[i] <= i == sz - 1 ? 0 : data_q[i + 1];
        end
    always_ff @(posedge clk)
        if (rst) ena_t <= 0;
        else for (int i = 0; i < sz; i++)
            if (in_wb.valid & in1_t[i]) begin
                ena_t[i] <= 1'b1;
                data_t[i] <= {1'b1, 191'd0, in_wb};
            end else if (in_ex.valid & (
                ~in_wb.valid & in1_t[i] | in_wb.valid & in2_t[i])) begin
                ena_t[i] <= 1'b1;
                data_t[i] <= {1'b0, in_ex};
            end else if (out1_t[i]) ena_t[i] <= 1'b0;
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

module mul(input clk, input rst,
    input logic rqst, input logic [4:0] op,
    input logic [63:0] a, input logic [63:0] b,
    output logic done, output logic [63:0] r
);
    logic [7:0][63:0] r_q;
    logic [7:0] valid;
    logic [127:0] res;
    always_comb res = {64'd0, a} * {64'd0, b};
    always_ff @(posedge clk)
        if (rst) valid <= 8'd0;
        else begin
            valid <= {rqst, valid[7:1]};
            r_q <= {res[63:0], r_q[7:1]};
        end
    always_comb r = r_q[0];
    always_comb done = valid[0];
endmodule
