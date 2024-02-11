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

// PC -> IF0: PC stage generate an address for IF0 stage.
typedef struct packed { logic valid; logic [63:0] pc; } pc_if0_t;
// IF0 -> PC: IF0 get 64-bit data from icache and calculate the
//            increment back to PC stage to generate new PC.
typedef struct packed { logic valid; logic [4:0] delta; } if0_pc_t;
// IF0 -> IF1: IF0 provide IF1 with 64-bit data to extract
//             several 32-bit instructions.
typedef struct packed {
    logic valid;
    logic [63:0] pc;
    logic [63:0] data;
    logic [4:0][7:0] offset;
} if0_if1_t;
// IF1 -> ID: IF1 gives instruction and its address.
typedef struct packed {
    logic valid;
    logic [31:0] ir;
    logic [63:0] pc;
    logic compressed;
} if1_id_t;
// ID -> EX: ID gives EX infomation of how many registers used, the
//           immediate, operator, and following stages info.
typedef struct packed {
    logic valid;
    logic [64:0] a, b; // EX stage info
    logic [52:0] exop;
    logic [2:0] frm, bmask;
    logic fdouble, bneg, j;
    logic [63:0] jbase, joffset;
    logic mr, mw; // MA stage info
    logic [2:0] bits;
    logic [6:0] rmwa;
    logic [6:0] rda; // WB stage info
} id_ex_t;
typedef struct packed {
    logic valid;
    logic [63:0] res;
    logic mr, mw;
    logic [2:0] bits;
    logic [4:0] rmwa;
    logic [6:0] rda;
} ex_ma_t;
typedef struct packed { logic valid; logic [4:0] rda; logic [63:0] rd; } ma_ex_t;

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
    if0_pc_t data_if0_pc;   logic get_if0_pc;
    pc_if0_t data_pc_if0;   logic get_pc_if0;
    if0_if1_t data_if0_if1; logic get_if0_if1;
    if1_id_t data_if1_id;   logic get_if1_id;
    id_ex_t data_id_ex;     logic get_id_ex;
    ex_ma_t data_ex_ma;     logic get_ex_ma;
    ma_ex_t data_ma_ex;     logic get_ma_ex;

    pc_stage pc_stage_inst(.clk(clk), .rst(rst),
        .in_if0(data_if0_pc), .get_if0(get_if0_pc),
        .out_if0(data_pc_if0), .ena_if0(get_pc_if0));
    if0_stage if0_stage_inst(.clk(clk), .rst(rst),
        .in_pc(data_pc_if0), .get_pc(get_pc_if0),
        .out_pc(data_if0_pc), .ena_pc(get_if0_pc),
        .out_if1(data_if0_if1), .ena_if1(get_if0_if1),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_done(icache_done), .icache_data(icache_data));
    if1_stage if1_stage_inst(.clk(clk), .rst(rst),
        .in_if0(data_if0_if1), .get_if0(get_if0_if1),
        .out_id(data_if1_id), .ena_id(get_if1_id));
    id_stage id_stage_inst(.clk(clk), .rst(rst),
        .in_if1(data_if1_id), .get_if1(get_if1_id),
        .out_ex(data_id_ex), .ena_ex(get_id_ex));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst),
        .in_id(data_id_ex), .get_id(get_id_ex),
        .out_ma(data_ex_ma), .ena_ma(get_ex_ma));
    ma_stage ma_stage_inst(.clk(clk), .rst(rst),
        .in_ex(data_ex_ma), .get_ex(get_ex_ma),
        .out_ex(data_ma_ex), .ena_ex(get_ma_ex));
endmodule

module pc_stage(input logic clk, input logic rst,
    input  if0_pc_t in_if0,  output logic get_if0,
    output pc_if0_t out_if0, input  logic ena_if0
);
    logic [63:0] pc;
    logic valid;
    always_comb out_if0.pc = pc;
    always_comb out_if0.valid = valid;
    always_ff @(posedge clk)
        if (rst) begin pc <= `RST_PC; valid <= 1'b1; end
        else if (ena_if0)
            if (0) begin
                pc <= pc;
                valid <= 0;
            end else if (in_if0.valid) begin
                pc <= pc + {59'd0, in_if0.delta};
                valid <= 1'b1;
            end else valid <= 1'b0;
    always_comb get_if0 = ena_if0 | ~in_if0.valid;
endmodule

module if0_stage(input logic clk, input logic rst,
    input  pc_if0_t  in_pc,   output logic get_pc,
    output if0_pc_t  out_pc,  input  logic ena_pc,
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
    always_comb icache_rqst = get_pc & in_pc.valid;
    always_ff @(posedge clk) sent <= rst ? 1'b0 : icache_rqst | sent & ~icache_done;
    always_ff @(posedge clk)
        hold_data <= rst ? 1'b0 : (ena_if1 ? 1'b0 : icache_done | hold_data);
    always_ff @(posedge clk)
        hold_delta <= rst ? 1'b0 : (ena_pc ? 1'b0 : icache_done | hold_delta);
    always_comb icache_addr = in_pc.pc;
    always_comb out_if1.valid = icache_done | hold_data;
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
    always_comb out_pc.valid = icache_done | hold_delta;
    always_comb if (off4 <= 8'd64) out_pc.delta = off4[7:3];
        else if (off3 <= 8'd64) out_pc.delta = off3[7:3];
        else if (off2 <= 8'd64) out_pc.delta = off2[7:3];
        else if (off1 <= 8'd64) out_pc.delta = off1[7:3];
        else out_pc.delta = 5'd0;
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
    output id_ex_t  out_ex, input  logic ena_ex
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
            op[`LOAD]  | op[`LOAD_FP] | op[`JALR]     | op[`AUIPC] |
            op[`LUI]   | op[`STORE]   | op[`STORE_FP] | op[`JAL]   |
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
            out_ex_q[0].j <= op[`JAL] | op[`JALR];
            out_ex_q[0].jbase <=
                in_if1.pc & {64{op[`JAL] | op[`BRANCH]}} |
                {59'd0, ir[19:15]} & {64{op[`JALR]}};
            out_ex_q[0].joffset <= imm;
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
                {2'd3, 5'd0} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};

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
                {2'd3, 5'd0} & {7{op[`AMO]}} |
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
            out_ex_q[2].rmwa <= {2'd3, 5'd0} & {7{op[`AMO]}};
            out_ex_q[2].rda <= 0;
        end
        else if (ena_ex) begin
            ena_q <= ena_q >> 1;
            out_ex_q <= {340'd0, out_ex_q[2], out_ex_q[1]};
        end
endmodule

module ex_stage(input logic clk, input logic rst,
    input  id_ex_t in_id,  output logic get_id,
    output ex_ma_t out_ma, input  logic ena_ma
);
    logic [52:0] op;
    always_comb op = in_id.exop;
    always_comb get_id = ena_ma | ~in_id.valid;
    always_ff @(posedge clk)
        if (rst) out_ma.valid <= 1'b0;
        else if (ena_ma) begin
            out_ma.valid <= in_id.valid & (
                op[`EX_ADD]  | op[`EX_SUB]  | op[`EX_SLL] | op[`EX_SLT] |
                op[`EX_SLTU] | op[`EX_XOR]  | op[`EX_SRL] | op[`EX_SRA] |
                op[`EX_OR]   | op[`EX_AND]  | op[`EX_MIN] | op[`EX_MAX] |
                op[`EX_MINU] | op[`EX_MAXU] | 1'b1);
        end
endmodule

module ma_stage(input logic clk, input logic rst,
    input  ex_ma_t in_ex,  output logic get_ex,
    output ma_ex_t out_ex, input  logic ena_ex
);
    always_comb out_ex.valid = 1'b1;
    always_comb get_ex = 1'b1;
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