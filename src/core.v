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
    wire ena_pc, ena_if, ena_id, ena_ex, ena_ma, ena_wb;
    reg [31:0] pc;
    wire [31:0] npc;

    // pipeline wires and registers
    wire        icache_valid_if;
    wire [6:0]  opcode_if;
    wire [5:0]  enc_if;
    wire [31:0] ir_if;
    wire [4:0]  rs1_addr_if;    reg  [4:0]  rs1_addr_id;
    reg  [31:0] pc_if;          reg  [31:0] pc_id;          reg  [31:0] pc_ex;
    wire [2:0]  funct3_if;      reg  [2:0]  funct3_id;      reg  [2:0]  funct3_ex;
    wire [6:0]  funct7_if;      reg  [6:0]  funct7_id;      reg  [6:0]  funct7_ex;
    wire [4:0]  rs2_addr_if;    reg  [4:0]  rs2_addr_id;    reg  [4:0]  rs2_addr_ex;
    wire [31:0] op_if;          reg  [31:0] op_id;          reg  [31:0] op_ex;           reg  [31:0] op_ma;
    wire [31:0] imm_if;         reg  [31:0] imm_id;         reg  [31:0] imm_ex;          reg  [31:0] imm_ma;
    wire [4:0]  rd_addr_if;     reg  [4:0]  rd_addr_id;     reg  [4:0]  rd_addr_ex;      reg  [4:0]  rd_addr_ma;
                                wire        ma_ex_stall_id;
                                wire [31:0] a_id;
                                wire [31:0] b_id;
                                wire [31:0] rs1_id;
                                wire [31:0] rs1_byp_id;
                                wire [31:0] rs2_id;         reg  [31:0] rs2_ex;
                                wire [31:0] rs2_byp_id;     wire [31:0] rs2_byp_ex;
                                                            wire [3:0]  flags_ex;
                                                            wire        b_suc_ex;
                                                            wire [31:0] r_alu_ex;
                                                            wire [31:0] r_muldiv_ex;
                                                            wire        muldiv_valid_ex;
                                                            wire [31:0] rd_ex;           wire [31:0] rd_ma;
                                                            wire [31:0] r_ex;            reg  [31:0] r_ma;
                                                            wire [31:0] d_out_ex;        wire [31:0] d_out_ma;
                                                                                         wire        dcache_valid_ma;

    // Pipeline start control
    reg start_if, start_id, start_ex, start_ma; // each stage has been started
    always @(posedge clk) begin // Jump can be implemented by start control as program restart from new PC
        start_if <= rst | ~ma_ex_stall_id & (start_ex & op_ex[`BRANCH] & b_suc_ex |
                                             start_id & op_id[`JALR] |
                                             start_if & op_if[`JAL]) ? 1'b0 : 1'b1;
        start_id <= rst | ~ma_ex_stall_id & (start_ex & op_ex[`BRANCH] & b_suc_ex |
                                             start_id & op_id[`JALR]) ? 1'b0 : start_if;
        start_ex <= rst | ~ma_ex_stall_id & start_ex & op_ex[`BRANCH] & b_suc_ex ? 1'b0 : start_id;
        start_ma <= rst ? 1'b0 : start_ex;
    end

    // PC control
    assign npc = start_ex & op_ex[`BRANCH] & b_suc_ex ? pc_ex + imm_ex :
                (start_id & op_id[`JALR] ? rs1_byp_id + imm_id :
                (start_if & op_if[`JAL] ? pc_if + imm_if : pc + 4));
    always @(posedge clk)
        pc <= rst ? 32'h00400000 : (ena_pc ? npc : pc);

    // Enable control
    assign ma_ex_stall_id = op_ex[`LOAD] &  // some wires in this expression are already existed
        ((~(op_id[`AUIPC] | op_id[`JAL]) & rs1_addr_id != 5'd0 & start_ex & rd_addr_ex == rs1_addr_id) |
         ( (op_id[`BRANCH] | op_id[`OP]) & rs2_addr_id != 5'd0 & start_ex & rd_addr_ex == rs2_addr_id));
    // EX/MA/WB is enabled only when IF/EX/MA been started (IF/EX/MA registers are ready)
    assign ena_pc = ~ma_ex_stall_id;
    assign ena_if = ~ma_ex_stall_id;
    assign ena_id = ~ma_ex_stall_id & start_if;
    assign ena_ex = ~ma_ex_stall_id & start_id;
    assign ena_ma = start_ex;
    assign ena_wb = start_ma;

    // General purpose registers and bypass control
    reg [31:0] gpr [0:31];
    always @(posedge clk) gpr[0] <= 32'd0;
    for (genvar igen = 1; igen < 32; igen = igen + 1) begin : gpreg_set
        always @(posedge clk)
            gpr[igen[4:0]] <= ena_wb & rd_addr_ma == igen[4:0] ? rd_ma : gpr[igen[4:0]]; end
    assign rs1_id = gpr[rs1_addr_id];
    assign rs2_id = gpr[rs2_addr_id];

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
        pc_id       <= ena_id ? pc_if : pc_id;
        funct7_id   <= ena_id ? funct7_if : funct7_id;
        funct3_id   <= ena_id ? funct3_if : funct3_id;
        op_id       <= ena_id ? op_if : op_id;
        imm_id      <= ena_id ? imm_if : imm_id;
        rs1_addr_id <= ena_id ? rs1_addr_if : rs1_addr_id;
        rs2_addr_id <= ena_id ? rs2_addr_if : rs2_addr_id;
        rd_addr_id  <= ena_id ? rd_addr_if : rd_addr_id;
    end
    assign rs1_byp_id = rs1_addr_id == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs1_addr_id ? rd_ex :
                        (start_ma & rd_addr_ma == rs1_addr_id ? rd_ma : rs1_id));
    assign rs2_byp_id = rs2_addr_id == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs2_addr_id ? rd_ex :
                        (start_ma & rd_addr_ma == rs2_addr_id ? rd_ma : rs2_id));
    assign a_id = op_id[`AUIPC] | op_id[`JAL] | op_id[`JALR] ? pc_id : rs1_byp_id;
    assign b_id = op_id[`BRANCH] | op_id[`OP] ? rs2_byp_id : (op_id[`JAL] | op_id[`JALR] ? 32'd4 : imm_id);

    // Execution
    always @(posedge clk) begin
        pc_ex       <= ena_ex ? pc_id : pc_ex;
        funct3_ex   <= ena_ex ? funct3_id : funct3_ex;
        funct7_ex   <= ena_ex ? funct7_id : funct7_ex;
        op_ex       <= ena_ex ? op_id : op_ex;
        imm_ex      <= ena_ex ? imm_id : imm_ex;
        rs2_ex      <= ena_ex ? rs2_id : rs2_ex;
        rs2_addr_ex <= ena_ex ? rs2_addr_id : rs2_addr_ex;
        rd_addr_ex  <= ma_ex_stall_id ? 5'd0 : (ena_ex ? rd_addr_id : rd_addr_ex); // NOP when stall
    end
    alu alu_inst(.clk(clk), .ena(ena_ex), .a(a_id), .b(b_id), .r(r_alu_ex), .c(flags_ex[3]), 
                 .funct3(op_id[`OP] | op_id[`OP_IMM] ? funct3_id : 3'b000),
                 .funct7(op_id[`OP] | op_id[`OP_IMM] & funct3_id == 3'b101 // SRL/SRA (special I-type)
                         ? funct7_id : (op_id[`BRANCH] ? 7'b0100000 : 7'b0000000)));
    // muldiv muldiv_inst(.clk(clk), .ena(ena_ex), .a(a_id), .b(b_id), .funct3(funct3_id),
    //                    .valid(muldiv_valid_ex), .r(r_muldiv_ex));
    assign r_ex = op_ex[`OP] & funct7_ex == 7'b0000001 ? r_muldiv_ex : r_alu_ex;
    assign flags_ex[2] = r_ex[31];
    assign flags_ex[0] = r_ex == 32'd0; // flags: carry, negative, (placeholder), zero
    assign b_suc_ex = flags_ex[funct3_ex[2:1]] == ~funct3_ex[0];
    assign d_out_ex = 32'd0; // Unable to fetch data of previous instructions due to pipeline
    assign rd_ex = op_ex[`LOAD] ? d_out_ex : (op_ex[`LUI] ? imm_ex : r_ex);
    assign rs2_byp_ex = rs2_addr_ex == 5'd0 ? 32'd0 :
                        (start_ma & rd_addr_ma == rs2_addr_ex ? rd_ma : rs2_ex);

    // Memory access
    always @(posedge clk) begin
        op_ma      <= ena_ma ? op_ex : op_ma;
        imm_ma     <= ena_ma ? imm_ex : imm_ma;
        rd_addr_ma <= ena_ma ? rd_addr_ex : rd_addr_ma;
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
    assign rd_ma = op_ma[`LOAD] ? d_out_ma : (op_ma[`LUI] ? imm_ma : r_ma);
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

module muldiv(
    input clk,
    input ena,
    input [31:0] a,
    input [31:0] b,
    input [2:0] funct3,
    output valid,
    output [31:0] r
);
    wire [31:0] a_abs, b_abs, q_abs, r_abs, q_neg, r_neg;
    wire [63:0] m_abs, m_neg;
    reg a_neg, b_neg;
    reg [2:0] funct3_reg;
    assign a_abs = (~funct3[2] & (funct3[1] ^ funct3[0]) | funct3[2] & ~funct3[0]) & a[31] ? -a : a;
    assign b_abs = (funct3 == 3'b001 | funct3[2] & ~funct3[0]) & b[31] ? -b : b;
    always @(posedge clk) funct3_reg <= ena ? funct3 : funct3_reg;
    always @(posedge clk) a_neg <= a[31];
    always @(posedge clk) b_neg <= b[31];
    mul mul_inst(.A(a_abs), .B(b_abs), .P(m_abs), .CLK(clk), .CE(ena));
    // CE may be not simply ena after using valid
    div div_inst(.s_axis_divisor_tdata(b_abs), .s_axis_divisor_tvalid(1'b1),
                 .s_axis_dividend_tdata(a_abs), .s_axis_dividend_tvalid(1'b1),
                 .m_axis_dout_tdata({q_abs, r_abs}), .m_axis_dout_tvalid(valid),
                 .aclk(clk), .aclken(ena));
    assign m_neg = -m_abs;
    assign q_neg = -q_abs;
    assign r_neg = -r_abs;
    wire [31:0] r_arr[0:7];
    assign r_arr[0] = m_abs[31:0];
    assign r_arr[1] = a_neg ^ b_neg ? m_neg[63:32] : m_abs[63:32];
    assign r_arr[2] = a_neg ? m_neg[63:32] : m_abs[63:32];
    assign r_arr[3] = m_abs[63:32];
    assign r_arr[4] = a_neg ^ b_neg ? q_neg : q_abs;
    assign r_arr[5] = q_abs;
    assign r_arr[6] = a_neg ? r_neg : r_abs;
    assign r_arr[7] = r_abs;
    assign r = r_arr[funct3_reg];
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
    input [LENGTH:0] rear,
    output [LENGTH:0] front
);
    // structure:
    //          |  valid region   |
    // no.    0 | 1   2   ...   S | S+1
    // ena    1 | x   x   ...   x | 0
    reg [LENGTH-1:0] buf_data [0:SIZE+1];
    reg              buf_ena  [0:SIZE+1];
    wire ins [1:SIZE]; // whether needs external data insertion
    assign empty = buf_ena[1];
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
