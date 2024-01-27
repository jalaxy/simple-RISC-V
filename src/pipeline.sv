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
typedef struct packed { logic pc, if0, if1, id, ex, ma, wb; } stages_t;
typedef struct packed { logic i, u, s, r, r4, b, j; } i_type_t;

typedef struct packed {
    logic valid_delta;
    logic [4:0] delta;
} pc_in_t;
typedef struct packed { logic valid; logic [63:0] pc; } pc_out_t;

typedef struct packed { logic valid; logic [63:0] pc; } if0_in_t;
typedef struct packed {
    logic valid;
    logic [63:0] pc;
    logic [63:0] data;
    logic [4:0][7:0] offset;
    logic [4:0] delta;
} if0_out_t;

typedef struct packed {
    logic valid;
    logic [63:0] pc;
    logic [63:0] data;
    logic [4:0][7:0] offset;
} if1_in_t;
typedef struct packed {
    logic valid;
    logic [31:0] ir;
    logic [63:0] pc;
} if1_out_t;

typedef struct packed { logic valid; logic [31:0] ir; logic [63:0] pc; } id_in_t;
typedef struct packed {
    logic valid;
    logic [63:0] pc;
    logic [31:0] op;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [63:0] imm;
    logic [4:0] rs1addr, rs2addr, rdaddr, uimm;
} id_out_t;

typedef struct packed { logic valid; } ex_in_t;
typedef struct packed { logic valid; } ex_out_t;

typedef struct packed { logic valid; } ma_in_t;
typedef struct packed { logic valid; } ma_out_t;

typedef struct packed { logic valid; } wb_in_t;
typedef struct packed { logic valid; } wb_out_t;

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
    stages_t ena, get, srst;
    always_comb ena.pc  = get.if0;
    always_comb ena.if0 = get.if1;
    always_comb ena.if1 = get.id;
    always_comb ena.id  = get.ex;
    always_comb ena.ex  = get.ma;
    always_comb ena.ma  = get.wb;
    always_comb ena.wb  = 1'b1;
    always_comb srst.pc = rst; // reset signals of each stage
    always_comb srst.if0 = rst;
    always_comb srst.if1 = rst;
    always_comb srst.id = rst;
    always_comb srst.ex = rst;
    always_comb srst.ma = rst;
    always_comb srst.wb = rst;

    pc_in_t  pc_in;
    if0_in_t if0_in;
    if1_in_t if1_in;
    id_in_t  id_in;
    ex_in_t  ex_in;
    ma_in_t  ma_in;
    wb_in_t  wb_in;
    pc_out_t  pc_out;
    if0_out_t if0_out;
    if1_out_t if1_out;
    id_out_t  id_out;
    ex_out_t  ex_out;
    ma_out_t  ma_out;
    wb_out_t  wb_out;

    // PC generation
    // input: (delta)
    // output: (pc)
    always_comb pc_in.delta = if0_out.delta;
    always_comb pc_in.valid_delta = if0_out.valid;
    pc_stage pc_stage_inst(.clk(clk), .rst(srst.pc), .ena(ena.pc),
        .get(get.pc), .in(pc_in), .out(pc_out));

    // Instruction fetch 0 (get data from icache)
    // input: (pc)
    // output: (data offset)
    always_comb if0_in.pc = pc_out.pc;
    always_comb if0_in.valid = pc_out.valid;
    if0_stage if0_stage_inst(.clk(clk), .rst(srst.if0), .ena(ena.if0),
        .get(get.if0), .in(if0_in), .out(if0_out),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_done(icache_done), .icache_data(icache_data));

    // Instruction fetch 1 (convert data to instructions)
    // input: (data)
    // output: (ir pc)
    always_comb if1_in.pc = if0_out.pc;
    always_comb if1_in.data = if0_out.data;
    always_comb if1_in.offset = if0_out.offset;
    always_comb if1_in.valid = if0_out.valid;
    if1_stage if1_stage_inst(.clk(clk), .rst(srst.if1), .ena(ena.if1),
        .get(get.if1), .in(if1_in), .out(if1_out));

    // Instruction decode
    // input: (ir pc)
    // output: (pc op funct3 funct7 imm rs1addr rs2addr rdaddr)
    always_comb id_in.ir = if1_out.ir;
    always_comb id_in.pc = if1_out.pc;
    always_comb id_in.valid = if1_out.valid;
    id_stage id_stage_inst(.clk(clk), .rst(srst.id), .ena(ena.id),
        .get(get.id), .in(id_in), .out(id_out));

    // Execute
    ex_stage ex_stage_inst(.clk(clk), .rst(srst.ex), .ena(ena.ex),
        .get(get.ex), .in(ex_in), .out(ex_out));

    // Memory access
    ma_stage ma_stage_inst(.clk(clk), .rst(srst.ma), .ena(ena.ma),
        .get(get.ma), .in(ma_in), .out(ma_out));

    // Write back and GPR
    wb_stage wb_stage_inst(.clk(clk), .rst(srst.wb), .ena(ena.wb),
        .get(get.wb), .in(wb_in), .out(wb_out));
endmodule

module pc_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input pc_in_t in, output pc_out_t out
);
    always_ff @(posedge clk)
        if (rst) begin out.pc <= `RST_PC; out.valid <= 1'b1; end
        else if (ena)
            if (0) begin
                out.pc <= out.pc;
                out.valid <= 0;
            end else if (in.valid_delta) begin
                out.pc <= out.pc + {59'd0, in.delta};
                out.valid <= 1'b1;
            end else out.valid <= 1'b0;
endmodule

module if0_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input if0_in_t in, output if0_out_t out,
    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    input  logic        icache_done,
    input  logic [63:0] icache_data
);
    logic [7:0] len0, len1, len2, len3, off0, off1, off2, off3, off4; // in bit
    logic [63:0] pc;
    logic sent, hold;
    always_comb get = ena & ~(sent & ~icache_done) | ~in.valid;
    always_comb icache_rqst = get & in.valid;
    always_ff @(posedge clk) sent <= rst ? 1'b0 : icache_rqst | sent & ~icache_done;
    always_ff @(posedge clk) hold <= rst ? 1'b0 : (ena ? 1'b0 : icache_done | hold);
    always_comb icache_addr = in.pc;
    always_comb out.pc = pc;
    always_comb out.data = icache_data;
    always_comb out.valid = icache_done | hold;
    always_comb out.offset = {off4, off3, off2, off1, off0};
    always_comb off0 = 8'd0;
    always_comb off1 = off0 + len0;
    always_comb off2 = off1 + len1;
    always_comb off3 = off2 + len2;
    always_comb off4 = off3 + len3;
    always_comb len0 = icache_data[off0[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len1 = icache_data[off1[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len2 = icache_data[off2[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len3 = icache_data[off3[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb if (off4 <= 8'd64) out.delta = off4[7:3];
        else if (off3 <= 8'd64) out.delta = off3[7:3];
        else if (off2 <= 8'd64) out.delta = off2[7:3];
        else if (off1 <= 8'd64) out.delta = off1[7:3];
        else out.delta = 5'd0;
    always_ff @(posedge clk) pc <= rst ? 64'd0 : (ena ? in.pc : pc);
endmodule

module if1_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input if1_in_t in, output if1_out_t out
);
    logic [63:0] base;
    logic [3:0][7:0] offset;
    logic [3:0][31:0] ir, ix; // instruction extended
    logic [3:0] ena_q;
    always_comb get = ena & ~ena_q[1] | ~in.valid;
    always_comb out.pc = base + {59'd0, offset[0][7:3]};
    always_comb out.ir = ir[0];
    always_comb out.valid = ena_q[0];
    always_ff @(posedge clk)
        if (rst) {ena_q, base, offset, ir} <= 0;
        else if (get & in.valid) begin
            ena_q <= {in.offset[4] <= 8'd64, in.offset[3] <= 8'd64,
                      in.offset[2] <= 8'd64, in.offset[1] <= 8'd64};
            base <= in.pc;
            offset <= in.offset[3:0];
            ir <= ix;
        end else if (ena) begin
            ena_q <= ena_q >> 1;
            offset <= offset >> 8;
            ir <= ir >> 32;
        end
    ci2i ci2i_inst1(.ci(in.data[in.offset[0]+31-:32]), .i(ix[0]));
    ci2i ci2i_inst2(.ci(in.data[in.offset[1]+31-:32]), .i(ix[1]));
    ci2i ci2i_inst3(.ci(in.data[in.offset[2]+31-:32]), .i(ix[2]));
    ci2i ci2i_inst4(.ci(in.data[in.offset[3]+31-:32]), .i(ix[3]));
endmodule

module id_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input id_in_t in, output id_out_t out
);
    // decoding output: op, funct3, funct7, imm, uimm, rs1addr, rs2addr, rdaddr
    logic [31:0] ir, op;
    i_type_t enc; // instruction type
    always_comb ir = in.ir;
    always_comb for (int i = 0; i < 32; i++)
        op[i] = ir[6:2] == i[4:0];
    always_comb enc = {
        op[`LOAD]  | op[`LOAD_FP]   | op[`MISC_MEM] | op[`OP_IMM]  // I
                   | op[`OP_IMM_32] | op[`JALR]     | op[`SYSTEM],
        op[`AUIPC] | op[`LUI],                                     // U
        op[`STORE] | op[`STORE_FP],                                // S
        op[`AMO]   | op[`OP]        | op[`OP_32]    | op[`OP_FP],  // R
        op[`MADD]  | op[`MSUB]      | op[`NMSUB]    | op[`NMADD],  // R4
        op[`BRANCH],                                               // B
        op[`JAL]};                                                 // J
    always_ff @(posedge clk) out.pc <= in.pc;
    always_ff @(posedge clk) out.funct3 <= ir[14:12];
    always_ff @(posedge clk) out.funct7 <= ir[31:25];
    always_ff @(posedge clk) out.uimm <= ir[19:15]; // same position as rs1addr
    always_ff @(posedge clk) out.op <= op;
    always_ff @(posedge clk)
        out.imm <= {64{enc.i}} & {{53{ir[31]}}, ir[30:20]} |
                      {64{enc.u}} & {{32{ir[31]}}, ir[31:12], 12'd0} |
                      {64{enc.s}} & {{53{ir[31]}}, ir[30:25], ir[11:7]} |
                      {64{enc.b}} & {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} |
                      {64{enc.j}} & {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
    always_ff @(posedge clk)
        if (op[`SYSTEM] & ir[14]) out.rs1addr <= 5'd0; // CSR[*]I
        else out.rs1addr <= enc.i | enc.s | enc.r | enc.r4 | enc.b ? ir[19:15] : 5'd0;
    always_ff @(posedge clk) out.rs2addr <= enc.r | enc.s | enc.b ? ir[24:20] : 5'd0;
    always_ff @(posedge clk)
        out.rdaddr <= enc.i | enc.u | enc.r | enc.r4 | enc.j ? ir[11:7] : 5'd0;
    always_ff @(posedge clk)
        if (rst) out.valid <= 1'b0; else if (ena) out.valid <= in.valid;
    always_comb get = 1'b1;
endmodule

module ex_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input ex_in_t in, output ex_out_t out
);
    always_comb out.valid = 1'b1;
    always_comb get = 1'b1;
endmodule

module ma_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input ma_in_t in, output ma_out_t out
);
    always_comb out.valid = 1'b1;
    always_comb get = 1'b1;
endmodule

module wb_stage(
    input logic clk, input logic rst, input logic ena, output logic get,
    input wb_in_t in, output wb_out_t out
);
    always_comb out.valid = 1'b1;
    always_comb get = 1'b1;
endmodule

module ci2i(input logic [31:0] ci, output logic [31:0] i);
    assign i = ci;
endmodule