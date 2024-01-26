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
typedef struct packed { logic pc, iF, id, ex, ma, wb; } stages_t;
typedef struct packed { logic i, u, s, r, r4, b, j; } i_type_t;

typedef struct packed {
    logic valid_offset;
    logic [7:0] offset;
} pc_in_t;
typedef struct packed { logic valid; logic [63:0] pc; } pc_out_t;

typedef struct packed { logic valid; logic [63:0] pc; } if_in_t;
typedef struct packed {
    logic valid_ir;
    logic [31:0] ir;
    logic [63:0] pc;
    logic valid_offset;
    logic [7:0] offset;
} if_out_t;

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
    stages_t ena, free;
    always_comb ena.pc = free.wb & free.ma & free.ex & free.id & free.iF;
    always_comb ena.iF = free.wb & free.ma & free.ex & free.id;
    always_comb ena.id = free.wb & free.ma & free.ex;
    always_comb ena.ex = free.wb & free.ma;
    always_comb ena.ma = free.wb;
    always_comb ena.wb = 1'b1;

    pc_in_t pc_in;
    if_in_t if_in;
    id_in_t id_in;
    ex_in_t ex_in;
    ma_in_t ma_in;
    wb_in_t wb_in;
    pc_out_t pc_out;
    if_out_t if_out;
    id_out_t id_out;
    ex_out_t ex_out;
    ma_out_t ma_out;
    wb_out_t wb_out;

    // PC generation
    // input: (offset)
    // output: (pc)
    always_comb pc_in.offset = if_out.offset;
    always_comb pc_in.valid_offset = if_out.valid_offset;
    pc_stage pc_stage_inst(.clk(clk), .rst(rst), .ena(ena.pc),
        .free(free.pc), .pc_in(pc_in), .pc_out(pc_out));

    // Instruction fetch
    // input: (pc)
    // output: (ir pc) (offset)
    always_comb if_in.pc = pc_out.pc;
    always_comb if_in.valid = pc_out.valid;
    if_stage if_stage_inst(.clk(clk), .rst(rst), .ena(ena.iF),
        .free(free.iF), .if_in(if_in), .if_out(if_out),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_done(icache_done), .icache_data(icache_data));

    // Instruction decode
    // input: (ir)
    // output: (op funct3 funct7 imm rs1addr rs2addr rdaddr)
    always_comb id_in.ir = if_out.ir;
    always_comb id_in.valid = if_out.valid_ir;
    id_stage id_stage_inst(.clk(clk), .rst(rst), .ena(ena.id),
        .free(free.id), .id_in(id_in), .id_out(id_out));

    // Execute
    ex_stage ex_stage_inst(.clk(clk), .rst(rst), .ena(ena.ex),
        .free(free.ex), .ex_in(ex_in), .ex_out(ex_out));

    // Memory access
    ma_stage ma_stage_inst(.clk(clk), .rst(rst), .ena(ena.ma),
        .free(free.ma), .ma_in(ma_in), .ma_out(ma_out));

    // Write back and GPR
    wb_stage wb_stage_inst(.clk(clk), .rst(rst), .ena(ena.wb),
        .free(free.wb), .wb_in(wb_in), .wb_out(wb_out));
endmodule

module pc_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input pc_in_t pc_in, output pc_out_t pc_out
);
    logic [63:0] pc;
    logic valid_pc;
    always_comb pc_out.pc = pc;
    always_comb pc_out.valid = valid_pc;
    always_ff @(posedge clk)
        if (rst) begin pc <= `RST_PC; valid_pc <= 1'b1; end
        else if (ena)
            if (0) begin
                pc <= pc;
                valid_pc <= 0;
            end else begin
                pc <= pc + {59'd0, pc_in.offset[7:3]};
                valid_pc <= pc_in.valid_offset;
            end
endmodule

module if_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input if_in_t if_in, output if_out_t if_out,
    output logic        icache_rqst,
    output logic [63:0] icache_addr,
    input  logic        icache_done,
    input  logic [63:0] icache_data
);
    logic [7:0] off0, off1, off2, off3, len0, len1, len2, len3;
    logic [63:0] pc[0:3]; // PC register
    logic [31:0] ir[0:3], inst_x[0:3];
    logic [3:0] ena_q;
    logic sent, busy;
    always_comb busy = sent & ~icache_done;
    always_comb icache_rqst = if_in.valid & free;
    always_comb icache_addr = if_in.pc;
    always_comb if_out.pc = pc[0];
    always_comb if_out.ir = ir[0];
    always_comb if_out.valid_offset = icache_done;
    always_comb if (icache_done)
        if      (off3 + len3 <= 8'd64) if_out.offset = off3 + len3;
        else if (off2 + len2 <= 8'd64) if_out.offset = off2 + len2;
        else if (off1 + len1 <= 8'd64) if_out.offset = off1 + len1;
        else if (off0 + len0 <= 8'd64) if_out.offset = off0 + len0;
        else if_out.offset = 8'd0; else if_out.offset = 8'd0;
    always_comb free = ~busy & ~ena_q[2];
    always_comb if_out.valid_ir = ena_q[0];
    // calculate instruction length and offset
    always_comb off0 = 8'd0;
    always_comb off1 = len0;
    always_comb off2 = len0 + len1;
    always_comb off3 = len0 + len1 + len2;
    always_comb len0 = icache_data[off0[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len1 = icache_data[off1[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len2 = icache_data[off2[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_comb len3 = icache_data[off3[5:0]+:2] == 2'b11 ? 8'd32 : 8'd16;
    always_ff @(posedge clk) sent <= rst ? 1'b0 : icache_rqst | busy;
    always_ff @(posedge clk)
        if (rst) begin
            ena_q <= 4'b0000;
            pc <= '{64'd0, 64'd0, 64'd0, 64'd0};
            ir <= '{32'd0, 32'd0, 32'd0, 32'd0};
        end else if (icache_done) begin
            ena_q <= {off3 + len3 <= 8'd64, off2 + len2 <= 8'd64,
                      off1 + len1 <= 8'd64, off0 + len0 <= 8'd64};
            pc[0] <= if_in.pc + {59'd0, off0[7:3]};
            pc[1] <= if_in.pc + {59'd0, off1[7:3]};
            pc[2] <= if_in.pc + {59'd0, off2[7:3]};
            pc[3] <= if_in.pc + {59'd0, off3[7:3]};
            ir[0] <= len0 == 8'd32 ? icache_data[off0+31-:32] : inst_x[off0[5:4]];
            ir[1] <= len1 == 8'd32 ? icache_data[off1+31-:32] : inst_x[off1[5:4]];
            ir[2] <= len2 == 8'd32 ? icache_data[off2+31-:32] : inst_x[off2[5:4]];
            ir[3] <= len3 == 8'd32 ? icache_data[off3+31-:32] : inst_x[off3[5:4]];
        end else if (ena) begin
            ena_q <= ena_q >> 4'd1;
            pc <= '{pc[1], pc[2], pc[3], 64'd0};
            ir <= '{ir[1], ir[2], ir[3], 32'd0};
        end
    ci2i ci2i_inst_0(.ci(icache_data[15:00]), .i(inst_x[0]));
    ci2i ci2i_inst_1(.ci(icache_data[31:16]), .i(inst_x[1]));
    ci2i ci2i_inst_2(.ci(icache_data[47:32]), .i(inst_x[2]));
    ci2i ci2i_inst_3(.ci(icache_data[63:48]), .i(inst_x[3]));
endmodule

module id_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input id_in_t id_in, output id_out_t id_out
);
    // decoding output: op, funct3, funct7, imm, uimm, rs1addr, rs2addr, rdaddr
    logic [31:0] ir, op;
    i_type_t enc; // instruction type
    always_comb ir = id_in.ir;
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
    always_ff @(posedge clk) id_out.funct3 <= ir[14:12];
    always_ff @(posedge clk) id_out.funct7 <= ir[31:25];
    always_ff @(posedge clk) id_out.uimm <= ir[19:15]; // same position as rs1addr
    always_ff @(posedge clk) id_out.op <= op;
    always_ff @(posedge clk)
        id_out.imm <= {64{enc.i}} & {{53{ir[31]}}, ir[30:20]} |
                      {64{enc.u}} & {{32{ir[31]}}, ir[31:12], 12'd0} |
                      {64{enc.s}} & {{53{ir[31]}}, ir[30:25], ir[11:7]} |
                      {64{enc.b}} & {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} |
                      {64{enc.j}} & {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
    always_ff @(posedge clk)
        if (op[`SYSTEM] & ir[14]) id_out.rs1addr <= 5'd0; // CSR[*]I
        else id_out.rs1addr <= enc.i | enc.s | enc.r | enc.r4 | enc.b ? ir[19:15] : 5'd0;
    always_ff @(posedge clk) id_out.rs2addr <= enc.r | enc.s | enc.b ? ir[24:20] : 5'd0;
    always_ff @(posedge clk)
        id_out.rdaddr <= enc.i | enc.u | enc.r | enc.r4 | enc.j ? ir[11:7] : 5'd0;
    always_ff @(posedge clk)
        if (rst) id_out.valid <= 1'b0; else if (ena) id_out.valid <= id_in.valid;
    always_comb free = 1'b1;
endmodule

module ex_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input ex_in_t ex_in, output ex_out_t ex_out
);
    always_comb ex_out.valid = 1'b1;
    always_comb free = 1'b1;
endmodule

module ma_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input ma_in_t ma_in, output ma_out_t ma_out
);
    always_comb ma_out.valid = 1'b1;
    always_comb free = 1'b1;
endmodule

module wb_stage(
    input logic clk, input logic rst, input logic ena, output logic free,
    input wb_in_t wb_in, output wb_out_t wb_out
);
    always_comb wb_out.valid = 1'b1;
    always_comb free = 1'b1;
endmodule

module ci2i(input logic [15:0] ci, output logic [31:0] i);
    assign i = {16'd0, ci};
endmodule