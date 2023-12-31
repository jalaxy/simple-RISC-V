`timescale 1ns/1ns
`define OP_IMM 5'b00100
`define LUI    5'b01101
`define AUIPC  5'b00101
`define OP     5'b01100
`define JAL    5'b11011
`define JALR   5'b11001
`define BRANCH 5'b11000
`define LOAD   5'b00000
`define STORE  5'b01000
module core(
    input clk,
    input rst,
    input icache_valid,
    input [31:0] icache_data,
    input dcache_valid,
    input [31:0] dcache_data_out,
    output icache_ena,
    output [31:0] icache_addr,
    output dcache_r_ena,
    output dcache_w_ena,
    output dcache_ext,
    output [1:0] dcache_width,
    output [31:0] dcache_addr,
    output [31:0] dcache_data_in
);
    // Enable and starting signals
    wire ena_pc, ena_if, ena_id, ena_ex, ena_ma, ena_wb;
    reg start_if, start_r_id, start_r_ex, start_ma; // each stage has been started
    wire start_id, start_ex;

    // Stageless signals
    reg [31:0] pc;
    wire [31:0] npc;
    wire [31:0] gpr_valid;

    // Stalling FIFO wires
    // ID stage
    wire [31:0] pc_q_id, pc_id, op_q_id, op_id, imm_q_id, imm_id;
    wire [2:0] funct3_q_id, funct3_id;
    wire [6:0] funct7_q_id, funct7_id;
    wire [4:0] rs1_addr_q_id, rs1_addr_id, rs2_addr_q_id, rs2_addr_id, rd_addr_q_id, rd_addr_id;
    wire q_full_id, q_empty_id, q_pop_id, q_push_id, q_stall_id;
    // EX stage
    wire [31:0] pc_q_ex, pc_ex, op_q_ex, op_ex, imm_q_ex, imm_ex, rs2_q_ex, rs2_ex;
    wire [2:0] funct3_q_ex, funct3_ex;
    wire [6:0] funct7_q_ex, funct7_ex;
    wire [4:0] rs2_addr_q_ex, rs2_addr_ex, rd_addr_q_ex, rd_addr_ex;
    wire q_full_ex, q_empty_ex, q_pop_ex, q_push_ex, q_stall_ex;

    // Multiplier and divider wires
    wire q_mul_empty, q_mul_full, q_mul_push, q_mul_pop;
    wire [31:0] q_mul_front, q_mul_rear;
    wire q_div_empty, q_div_full, q_div_push, q_div_pop;
    wire [31:0] q_div_front, q_div_rear;

    // pipeline wires and registers
    wire        icache_valid_if;
    wire [6:0]  opcode_if;
    wire [5:0]  enc_if;
    wire [31:0] ir_if;
    wire [4:0]  rs1_addr_if;    reg  [4:0]  rs1_addr_r_id;
    reg  [31:0] pc_if;          reg  [31:0] pc_r_id;        reg  [31:0] pc_r_ex;
    wire [2:0]  funct3_if;      reg  [2:0]  funct3_r_id;    reg  [2:0]  funct3_r_ex;
    wire [6:0]  funct7_if;      reg  [6:0]  funct7_r_id;    reg  [6:0]  funct7_r_ex;
    wire [4:0]  rs2_addr_if;    reg  [4:0]  rs2_addr_r_id;  reg  [4:0]  rs2_addr_r_ex;
    wire [31:0] op_if;          reg  [31:0] op_r_id;        reg  [31:0] op_r_ex;       reg  [31:0] op_ma;
    wire [31:0] imm_if;         reg  [31:0] imm_r_id;       reg  [31:0] imm_r_ex;      reg  [31:0] imm_ma;
    wire [4:0]  rd_addr_if;     reg  [4:0]  rd_addr_r_id;   reg  [4:0]  rd_addr_r_ex;  reg  [4:0]  rd_addr_ma;
                                wire [31:0] a_id;
                                wire [31:0] b_id;
                                wire [31:0] rs1_id;
                                wire [31:0] rs1_byp_id;
                                wire [31:0] rs2_id;         reg  [31:0] rs2_r_ex;
                                wire [31:0] rs2_byp_id;     wire [31:0] rs2_byp_ex;
                                                            wire [3:0]  flags_ex;
                                                            wire        b_suc_ex;
                                                            wire [31:0] r_alu_ex;
                                                            wire [31:0] rd_ex;         wire [31:0] rd_ma;
                                                            wire [31:0] r_ex;          reg  [31:0] r_ma;
                                                            wire [31:0] d_out_ex;      wire [31:0] d_out_ma;
                                                                                       wire        dcache_valid_ma;

    // Pipeline start control
    assign start_id = start_r_id | q_pop_id;
    assign start_ex = start_r_ex | q_pop_ex;
    always @(posedge clk) begin // Jump can be implemented by start control as program restart from new PC
        start_if <= rst ? 1'b0 : (ena_if ?
            (start_ex & op_ex[`BRANCH] & b_suc_ex |
             start_id & op_id[`JALR] |
             start_if & op_if[`JAL] ? 1'b0 : 1'b1) : start_if);
        start_r_id <= rst ? 1'b0 : (ena_id ?
            (start_ex & op_ex[`BRANCH] & b_suc_ex |
             start_id & op_id[`JALR] ? 1'b0 : start_if) : start_r_id);
        start_r_ex <= rst ? 1'b0 : (ena_ex ?
            (start_ex & op_ex[`BRANCH] & b_suc_ex ? 1'b0 : start_id) : start_r_ex);
        start_ma <= rst ? 1'b0 : start_ex;
    end

    // PC control
    assign npc = start_ex & op_ex[`BRANCH] & b_suc_ex ? pc_ex + imm_ex :
                (start_id & op_id[`JALR] & ~q_push_id ? rs1_byp_id + imm_id :
                (start_if & op_if[`JAL] ? pc_if + imm_if : pc + 4));
    always @(posedge clk)
        pc <= rst ? 32'h00400000 : (ena_pc ? npc : pc);

    // Enable control
    // EX/MA/WB is enabled only when IF/EX/MA been started (IF/EX/MA registers are ready)
    assign ena_pc = ~q_stall_ex & ~q_stall_id;
    assign ena_if = ~q_stall_ex & ~q_stall_id;
    assign ena_id = ~q_stall_ex & ~q_stall_id & start_if;
    assign ena_ex = ~q_stall_ex & start_id;
    assign ena_ma = start_ex;
    assign ena_wb = start_ma;

    // Bypass control
    assign rd_ex = op_ex[`LOAD] ? d_out_ex : (op_ex[`LUI] ? imm_ex : r_ex);
    assign rd_ma = op_ma[`LOAD] ? d_out_ma : (op_ma[`LUI] ? imm_ma : r_ma);
    assign rs1_byp_id = rs1_addr_id == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs1_addr_id ? rd_ex :
                        (start_ma & rd_addr_ma == rs1_addr_id ? rd_ma : rs1_id));
    assign rs2_byp_id = rs2_addr_id == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs2_addr_id ? rd_ex :
                        (start_ma & rd_addr_ma == rs2_addr_id ? rd_ma : rs2_id));
    assign rs2_byp_ex = rs2_addr_ex == 5'd0 ? 32'd0 :
                        (start_ma & rd_addr_ma == rs2_addr_ex ? rd_ma : rs2_ex);
    assign gpr_valid[0] = 1'b1;
    for (genvar igen = 1; igen < 32; igen = igen + 1) begin : gpreg_valid
        assign gpr_valid[igen[4:0]] = ~(start_ex & rd_addr_ex == igen[4:0] & op_ex[`LOAD]); end

    // Stalling control
    // ID stage
    assign q_push_id = ena_ex & ((~(op_r_id[`AUIPC] | op_r_id[`JAL]) & ~gpr_valid[rs1_addr_r_id]) |
                                 ( (op_r_id[`BRANCH] | op_r_id[`OP]) & ~gpr_valid[rs2_addr_r_id]));
    assign q_pop_id = ~q_empty_id & gpr_valid[rs1_addr_q_id];
    assign q_stall_id = q_push_id & ~q_pop_id & q_full_id | ~q_push_id & q_pop_id & ~op_id[`JALR];
    queue #(.LENGTH(121), .SIZE(4)) queue_id_stall_inst(
        .clk(clk), .rst(rst), .pop(q_pop_id), .push(q_push_id), .empty(q_empty_id), .full(q_full_id),
        .rear({ pc_r_id, op_r_id, imm_r_id, funct3_r_id, funct7_r_id, rs1_addr_r_id, rs2_addr_r_id, rd_addr_r_id}),
        .front({pc_q_id, op_q_id, imm_q_id, funct3_q_id, funct7_q_id, rs1_addr_q_id, rs2_addr_q_id, rd_addr_q_id}));
    assign {pc_id, op_id, imm_id, funct3_id, funct7_id, rs1_addr_id, rs2_addr_id, rd_addr_id} = q_pop_id ?
        {pc_q_id, op_q_id, imm_q_id, funct3_q_id, funct7_q_id, rs1_addr_q_id, rs2_addr_q_id, rd_addr_q_id} :
        {pc_r_id, op_r_id, imm_r_id, funct3_r_id, funct7_r_id, rs1_addr_r_id, rs2_addr_r_id, rd_addr_r_id};
    // EX stage
    assign q_push_ex = ena_ma & (op_r_ex[`OP] & funct7_r_ex == 7'b0000001);
    assign q_pop_ex = q_mul_pop;
    assign q_stall_ex = q_push_ex & ~q_pop_ex & q_full_ex | ~q_push_ex & q_pop_ex;
    queue #(.LENGTH(148), .SIZE(4)) queue_ex_stall_inst(
        .clk(clk), .rst(rst), .pop(q_pop_ex), .push(q_push_ex), .empty(q_empty_ex), .full(q_full_ex),
        .rear({ pc_r_ex, op_r_ex, imm_r_ex, funct3_r_ex, funct7_r_ex, rs2_addr_r_ex, rd_addr_r_ex, rs2_r_ex}),
        .front({pc_q_ex, op_q_ex, imm_q_ex, funct3_q_ex, funct7_q_ex, rs2_addr_q_ex, rd_addr_q_ex, rs2_q_ex}));
    assign {pc_ex, op_ex, imm_ex, funct3_ex, funct7_ex, rs2_addr_ex, rd_addr_ex, rs2_ex} = q_pop_ex ?
        {pc_q_ex, op_q_ex, imm_q_ex, funct3_q_ex, funct7_q_ex, rs2_addr_q_ex, rd_addr_q_ex, rs2_q_ex} :
        {pc_r_ex, op_r_ex, imm_r_ex, funct3_r_ex, funct7_r_ex, rs2_addr_r_ex, rd_addr_r_ex, rs2_r_ex};

    // Multiplier and divider
    assign q_mul_pop = ~q_mul_empty & op_q_ex[`OP] & funct7_q_ex == 7'b0000001;
    mul mul_inst(.clk(clk), .ena(ena_ex & op_id[`OP] & funct7_id == 7'b0000001 & ~funct3_id[2]),
                 .a(a_id), .b(b_id), .funct3(funct3_id), .valid(q_mul_push), .r(q_mul_rear));
    queue #(.LENGTH(32), .SIZE(8)) queue_mul_inst(
            .clk(clk), .rst(rst), .pop(q_mul_pop), .push(q_mul_push),
            .empty(q_mul_empty), .full(q_mul_full), .rear(q_mul_rear), .front(q_mul_front));
    div div_inst(.clk(clk), .ena(ena_ex & op_id[`OP] & funct7_id == 7'b0000001 & funct3_id[2]),
                 .a(a_id), .b(b_id), .funct3(funct3_id), .valid(q_div_push), .r(q_div_rear));

    // Instruction fetch
    assign icache_ena = ena_if;
    assign icache_addr = pc;
    assign icache_valid_if = icache_valid;
    assign ir_if = icache_data;
    always @(posedge clk) pc_if <= ena_if ? pc : pc_if;
    assign opcode_if = ir_if[6:0];
    assign funct3_if = ir_if[14:12];
    assign funct7_if = ir_if[31:25];
    for (genvar igen = 0; igen < 32; igen = igen + 1) begin : decoder
        assign op_if[igen[4:0]] = opcode_if[6:2] == igen[4:0]; end
    assign enc_if = {op_if[`OP], op_if[`OP_IMM] | op_if[`JALR] | op_if[`LOAD], op_if[`STORE],
                     op_if[`BRANCH], op_if[`LUI] | op_if[`AUIPC], op_if[`JAL]}; // R I S B U J
    assign rs1_addr_if = (enc_if & 6'b111100) == 6'd0 ? 5'd0 : ir_if[19:15];
    assign rs2_addr_if = (enc_if & 6'b101100) == 6'd0 ? 5'd0 : ir_if[24:20];
    assign rd_addr_if = (enc_if & 6'b110011) == 6'd0 ? 5'd0 : ir_if[11:7];
    assign imm_if = enc_if[4] ? {{21{ir_if[31]}}, ir_if[30:20]} : (                              // I type
                    enc_if[3] ? {{21{ir_if[31]}}, ir_if[30:25], ir_if[11:7]} : (                 // S type
                    enc_if[2] ? {{20{ir_if[31]}}, ir_if[7], ir_if[30:25], ir_if[11:8], 1'b0} : ( // B type
                    enc_if[1] ? {ir_if[31:12], 12'd0} : (                                        // U type
                    enc_if[0] ? {{12{ir_if[31]}}, ir_if[19:12], ir_if[20], ir_if[30:21], 1'b0} : // J type
                                32'd0))));                                                       // R type

    // Instruction decode
    always @(posedge clk) begin
        pc_r_id       <= ena_id ? pc_if : pc_r_id;
        funct7_r_id   <= ena_id ? funct7_if : funct7_r_id;
        funct3_r_id   <= ena_id ? funct3_if : funct3_r_id;
        op_r_id       <= ena_id ? op_if : op_r_id;
        imm_r_id      <= ena_id ? imm_if : imm_r_id;
        rs1_addr_r_id <= ena_id ? rs1_addr_if : rs1_addr_r_id;
        rs2_addr_r_id <= ena_id ? rs2_addr_if : rs2_addr_r_id;
        rd_addr_r_id  <= ena_id ? rd_addr_if : rd_addr_r_id;
    end
    assign a_id = op_id[`AUIPC] | op_id[`JAL] | op_id[`JALR] ? pc_id : rs1_byp_id;
    assign b_id = op_id[`BRANCH] | op_id[`OP] ? rs2_byp_id : (op_id[`JAL] | op_id[`JALR] ? 32'd4 : imm_id);

    // Execution
    always @(posedge clk) begin
        pc_r_ex       <= ena_ex ? pc_id : pc_r_ex;
        funct3_r_ex   <= ena_ex ? funct3_id : funct3_r_ex;
        funct7_r_ex   <= ena_ex ? funct7_id : funct7_r_ex;
        op_r_ex       <= ena_ex ? op_id : op_r_ex;
        imm_r_ex      <= ena_ex ? imm_id : imm_r_ex;
        rs2_r_ex      <= ena_ex ? rs2_id : rs2_r_ex;
        rs2_addr_r_ex <= ena_ex ? rs2_addr_id : rs2_addr_r_ex;
        rd_addr_r_ex  <= q_push_id & ~q_pop_id ? 5'd0 : (ena_ex ? rd_addr_id : rd_addr_r_ex); // NOP when stall
    end
    alu alu_inst(.clk(clk), .ena(ena_ex), .a(a_id), .b(b_id), .r(r_alu_ex), .c(flags_ex[3]), 
                 .funct3(op_id[`OP] | op_id[`OP_IMM] ? funct3_id : 3'b000),
                 .funct7(op_id[`OP] | op_id[`OP_IMM] & funct3_id == 3'b101 // SRL/SRA (special I-type)
                         ? funct7_id : (op_id[`BRANCH] ? 7'b0100000 : 7'b0000000)));
    assign r_ex = op_ex[`OP] & funct7_ex == 7'b0000001 ? (funct3_ex[2] ? q_div_front : q_mul_front) : r_alu_ex;
    assign flags_ex[2] = r_ex[31];
    assign flags_ex[0] = r_ex == 32'd0; // flags: carry, negative, (placeholder), zero
    assign b_suc_ex = flags_ex[funct3_ex[2:1]] == ~funct3_ex[0];
    assign d_out_ex = 32'd0; // Unable to fetch data of previous instructions due to pipeline

    // Memory access
    always @(posedge clk) begin
        op_ma      <= ena_ma ? op_ex : op_ma;
        imm_ma     <= ena_ma ? imm_ex : imm_ma;
        rd_addr_ma <= q_push_ex & ~q_pop_ex ? 5'd0 : (ena_ma ? rd_addr_ex : rd_addr_ma); // NOP when stall
        r_ma       <= ena_ma ? r_ex : r_ma;
    end
    assign dcache_r_ena = ena_ma & op_ex[`LOAD];
    assign dcache_w_ena = ena_ma & op_ex[`STORE];
    assign dcache_addr = r_ex;
    assign dcache_ext = funct3_ex[2];
    assign dcache_width = funct3_ex[1:0];
    assign dcache_data_in = rs2_byp_ex;
    assign dcache_valid_ma = dcache_valid;
    assign d_out_ma = dcache_data_out;

    // Write back
    gpreg gpreg_inst(.clk(clk), .ena(ena_wb), .rs1_addr(rs1_addr_id), .rs2_addr(rs2_addr_id),
                     .rd_addr(rd_addr_ma), .rd(rd_ma), .rs1(rs1_id), .rs2(rs2_id));
endmodule

module gpreg(
    input clk,
    input ena,
    input [4:0] rs1_addr,
    input [4:0] rs2_addr,
    input [4:0] rd_addr,
    input [31:0] rd,
    output [31:0] rs1,
    output [31:0] rs2
);
    reg [31:0] r[0:31];
    assign rs1 = rs1_addr == 5'd0 ? 32'd0 : r[rs1_addr];
    assign rs2 = rs2_addr == 5'd0 ? 32'd0 : r[rs2_addr];
    always @(posedge clk)
        r[rd_addr] <= ena ? rd : r[rd_addr];
endmodule

module alu(
    input clk,
    input ena,
    input [31:0] a,
    input [31:0] b,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [31:0] r,
    output reg c // only for SUB
);
    wire [31:0] r_arr[0:7];
    wire c_wire;
    assign {c_wire, r_arr[3'd0]} = funct7[5] ? {1'b0, a} - {1'b0, b} : a + b; // ADD/SUB
    assign r_arr[3'd1] = a << b[4:0]; // SLL
    assign r_arr[3'd2] = {31'd0, $signed(a) < $signed(b)}; // SLT
    assign r_arr[3'd3] = {31'd0, a < b}; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = funct7[5] ? $signed($signed(a) >>> b[4:0]) : a >> b[4:0]; // SRA/SRL
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    always @(posedge clk) r <= ena ? r_arr[funct3] : r;
    always @(posedge clk) c <= ena ? c_wire : c;
endmodule

module mul(
    input clk,
    input ena,
    input [31:0] a,
    input [31:0] b,
    input [2:0] funct3,
    output valid,
    output [31:0] r
);
    wire [31:0] a_abs, b_abs;
    assign a_abs = (funct3[1] ^ funct3[0]) & a[31] ? -a : a;
    assign b_abs = (funct3 == 3'b001) & b[31] ? -b : b;

    // unsigned r = a * b and unsigned r = a / b
    // valid and ena are correspondent
    wire [63:0] r_abs, r_neg;
    assign r_abs = ((b_abs[0] ? {32'b0, a_abs} : 64'b0) +
                    (b_abs[1] ? {31'b0, a_abs, 1'b0} : 64'b0) +
                    (b_abs[2] ? {30'b0, a_abs, 2'b0} : 64'b0) +
                    (b_abs[3] ? {29'b0, a_abs, 3'b0} : 64'b0) +
                    (b_abs[4] ? {28'b0, a_abs, 4'b0} : 64'b0) +
                    (b_abs[5] ? {27'b0, a_abs, 5'b0} : 64'b0) +
                    (b_abs[6] ? {26'b0, a_abs, 6'b0} : 64'b0) +
                    (b_abs[7] ? {25'b0, a_abs, 7'b0} : 64'b0) +
                    (b_abs[8] ? {24'b0, a_abs, 8'b0} : 64'b0) +
                    (b_abs[9] ? {23'b0, a_abs, 9'b0} : 64'b0) +
                    (b_abs[10] ? {22'b0, a_abs, 10'b0} : 64'b0) +
                    (b_abs[11] ? {21'b0, a_abs, 11'b0} : 64'b0) +
                    (b_abs[12] ? {20'b0, a_abs, 12'b0} : 64'b0) +
                    (b_abs[13] ? {19'b0, a_abs, 13'b0} : 64'b0) +
                    (b_abs[14] ? {18'b0, a_abs, 14'b0} : 64'b0) +
                    (b_abs[15] ? {17'b0, a_abs, 15'b0} : 64'b0) +
                    (b_abs[16] ? {16'b0, a_abs, 16'b0} : 64'b0) +
                    (b_abs[17] ? {15'b0, a_abs, 17'b0} : 64'b0) +
                    (b_abs[18] ? {14'b0, a_abs, 18'b0} : 64'b0) +
                    (b_abs[19] ? {13'b0, a_abs, 19'b0} : 64'b0) +
                    (b_abs[20] ? {12'b0, a_abs, 20'b0} : 64'b0) +
                    (b_abs[21] ? {11'b0, a_abs, 21'b0} : 64'b0) +
                    (b_abs[22] ? {10'b0, a_abs, 22'b0} : 64'b0) +
                    (b_abs[23] ? {9'b0, a_abs, 23'b0} : 64'b0) +
                    (b_abs[24] ? {8'b0, a_abs, 24'b0} : 64'b0) +
                    (b_abs[25] ? {7'b0, a_abs, 25'b0} : 64'b0) +
                    (b_abs[26] ? {6'b0, a_abs, 26'b0} : 64'b0) +
                    (b_abs[27] ? {5'b0, a_abs, 27'b0} : 64'b0) +
                    (b_abs[28] ? {4'b0, a_abs, 28'b0} : 64'b0) +
                    (b_abs[29] ? {3'b0, a_abs, 29'b0} : 64'b0) +
                    (b_abs[30] ? {2'b0, a_abs, 30'b0} : 64'b0) +
                    (b_abs[31] ? {1'b0, a_abs, 31'b0} : 64'b0));
    assign r_neg = -r_abs;
    wire [31:0] r_arr[0:3];
    assign r_arr[0] = r_abs[31:0];
    assign r_arr[1] = a[31] ^ b[31] ? r_neg[63:32] : r_abs[63:32];
    assign r_arr[2] = a[31] ? r_neg[63:32] : r_abs[63:32];
    assign r_arr[3] = r_abs[63:32];

    reg [31:0] r_reg[1:5];
    reg done[1:5];
    always @(posedge clk) begin
        r_reg[1] <= r_arr[funct3[1:0]]; done[1] <= ena;
        r_reg[2] <= r_reg[1];  done[2] <= done[1];
        r_reg[3] <= r_reg[2];  done[3] <= done[2];
        r_reg[4] <= r_reg[3];  done[4] <= done[3];
        r_reg[5] <= r_reg[4];  done[5] <= done[4];
    end
    assign r = r_reg[5];
    assign valid = done[5];
endmodule

module div(
    input clk,
    input ena,
    input [31:0] a,
    input [31:0] b,
    input [2:0] funct3,
    output reg valid,
    output reg [31:0] r
);
endmodule

module queue #(
    LENGTH = 32,
    SIZE = 8
) (
    input clk,
    input rst,
    input pop,
    input push,
    output empty,
    output full,
    input [LENGTH-1:0] rear,
    output [LENGTH-1:0] front
);
    // structure:
    //          |  valid region   |
    // no.    0 | 1   2   ...   S | S+1
    // ena    1 | x   x   ...   x | 0
    reg [LENGTH-1:0] buf_data [0:SIZE+1];
    reg              buf_ena  [0:SIZE+1];
    wire ins [1:SIZE]; // whether needs external data insertion
    assign empty = ~buf_ena[1];
    assign full  = buf_ena[SIZE];
    assign front = buf_data[1];
    always @(posedge clk) buf_ena[0] <= 1'b1;
    always @(posedge clk) buf_ena[SIZE + 1] <= 1'b0;
    for (genvar igen = 1; igen <= SIZE; igen = igen + 1) begin : buffer_transfer
        assign ins[igen] = push & (pop & buf_ena[igen] & ~buf_ena[igen + 1] | // push and pop
                                  ~pop & buf_ena[igen - 1] & ~buf_ena[igen]); // push and not pop
        always @(posedge clk) begin
            buf_ena[igen]  <= rst ? 1'b0 : (ins[igen] ? 1'b1 : (pop ? buf_ena[igen + 1] : buf_ena[igen]));
            buf_data[igen] <= ins[igen] ? rear : (pop ? buf_data[igen + 1] : buf_data[igen]);
        end end
endmodule
