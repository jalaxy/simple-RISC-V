`timescale 1ns/1ns
module core(
    input clk,
    input rst
);
    wire clk_if, clk_wb, clk_ex;
    assign clk_if = clk;
    assign clk_wb = clk;
    assign clk_ex = clk;

    // pipeline registers
    reg [31:0]  pc_if;      reg [31:0]  pc_ex;      reg [31:0]  pc_ma;    reg [31:0] pc_wb;
    wire [5:0]  mux_if;     reg [5:0]   mux_ex;     reg [5:0]   mux_ma;
    wire [2:0]  funct3_if;  reg [2:0]   funct3_ex;
    wire        mem_w_if;   reg         mem_w_ex;
    wire [31:0] imm_if;     reg [31:0]  imm_ex;     reg [31:0]  imm_ma;
    wire [31:0] rs2_if;     reg [31:0]  rs2_ex;
    wire [4:0]  rd_addr_if; reg [4:0]   rd_addr_ex; reg [4:0]   rd_addr_ma;
                            wire [31:0] r_ex;       reg [31:0]  r_ma;
                            wire        b_suc_ex;   reg         b_suc_ma;

    // Instruction fetch & decode
    wire icache_valid;
    wire [6:0] opcode;
    wire [5:0] enc;
    wire [31:0] ir;
    wire op_imm, lui, auipc, op, jal, jalr, branch, load, store;
    wire [4:0] rs1_addr, rs2_addr;
    icache icache_inst(.clk(clk_if), .addr(pc_wb), .valid(icache_valid), .data(ir_if));
    always@(clk_if)
        pc_if <= pc_wb;
    assign opcode    = ir[6:0];
    assign funct3_if = ir[14:12];
    assign funct7    = ir[31:25];
    assign op_imm    = opcode == 7'b0010011;
    assign lui       = opcode == 7'b0110111;
    assign auipc     = opcode == 7'b0010111;
    assign op        = opcode == 7'b0110011;
    assign jal       = opcode == 7'b1101111;
    assign jalr      = opcode == 7'b1100111;
    assign branch    = opcode == 7'b1100011;
    assign load      = opcode == 7'b0000011;
    assign store     = opcode == 7'b0100011;
    assign mem_w_if  = store;
    assign enc = {op, op_imm | jalr | load, store, branch, lui | auipc, jal}; // R I S B U J
    assign rs1_addr = enc & 6'b111100 == 6'd0 ? 5'd0 : ir[19:15];
    assign rs2_addr = enc & 6'b101100 == 6'd0 ? 5'd0 : ir[24:20];
    assign rd_addr_if = enc & 6'b110011 == 6'd0 ? 5'd0 : ir[11:7];
    assign imm_if = enc[4] ? {{21{ir[31]}}, ir[30:20]} : (                        // I type
                    enc[3] ? {{21{ir[31]}}, ir[30:25], ir[11:7]} : (              // S type
                    enc[2] ? {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} : ( // B type
                    enc[1] ? {ir[31:12], 12'd0} : (                               // U type
                    enc[0] ? {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} : // J type
                             32'd0))));                                           // R type
    // MUX selection signal: H -> L
    // PC input (1), PC adder (1), ALU a (1), ALU b (1), rd input (2)
    assign mux_if = {jal | jalr, branch, auipc | jal, branch | op,
                     load | jal | jalr, load | lui};

    // Execution
    always @(posedge clk_ex) begin
        pc_ex      <= pc_if;
        mux_ex     <= mux_if;
        funct3_ex  <= funct3_if;
        mem_w_ex   <= mem_w_if;
        imm_ex     <= imm_if;
        rs2_ex     <= rs2_ex;
        rd_addr_ex <= rd_addr_if;
    end
    wire [31:0] a, b, r, rs1;
    wire [3:0] flags; // carry, negative, (placeholder), zero
    assign a = mux_if[3] ? pc_if : rs1;
    assign b = mux_if[2] ? rs2_if : imm_if;
    alu alu_inst(.a(a), .b(b), .r(r), .c(flags[3]), 
                 .funct3(op | op_imm ? funct3_if : 3'b000),
                 .funct7(op | op_imm ? funct7 : (branch ? 7'b0100000 : 7'b0000000)));
    reg [31:0] r_reg;
    always @(clk_ex)
        r_reg <= r;
    assign r_ex = r_reg;
    assign flags[2] = r[31];
    assign flags[0] = r == 32'd0;
    assign b_suc_ex = mux_ex[4] & flags[funct3_ex[2:1]] == ~funct3_ex[0];

    // Memory access
    always @(posedge clk_ma) begin
        pc_ma      <= pc_ex;
        mux_ma     <= mux_ex;
        imm_ma     <= imm_ex;
        rd_addr_ma <= rd_addr_ex;
        r_ma       <= r_ex;
        b_suc_ma   <= b_suc_ex;
    end
    wire dcache_valid;
    wire [31:0] d_out;
    dcache dcache_inst(.clk(clk_ma), .w_ena(mem_w_ex), .addr(r_ex),
                       .width(funct3_ex[1:0]), .ext(funct3_ex[2]),
                       .data_in(rs2_ex), .valid(dcache_valid), .data_out(d_out));

    // Write back
    wire [31:0] pc_adder;
    assign pc_adder = pc_ma + (b_suc_ma ? imm_ma : 4);
    always @(posedge clk_wb)
        pc_wb <= mux_ma[5] ? r_ma : pc_adder;
    wire [31:0] rd_ma;
    assign rd_ma = mux_ma[1] ? (mux_ma[0] ? d_out : pc_adder) : (mux_ma[0] ? imm_ma : r_ma);
    gpreg gpreg_inst(.clk(clk_wb), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr),
                     .rd_addr(rd_addr_ma), .rd(rd_ma), .rs1(rs1), .rs2(rs2_if));
    
    always @(posedge rst)
        pc_wb <= 32'd0;
endmodule

module gpreg(
    input clk,
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
        r[rd_addr] <= rd;
endmodule

module alu(
    input [31:0] a,
    input [31:0] b,
    input [2:0] funct3,
    input [6:0] funct7,
    output [31:0] r,
    output c // only for SUB
);
    wire [31:0] r_arr[7:0];
    assign {c, r_arr[3'd0]} = funct7[5] ? {1'b0, a} - {1'b0, b} : a + b; // ADD/SUB
    assign r_arr[3'd1] = a << b[4:0]; // SLL
    assign r_arr[3'd2] = $signed(a) < $signed(b); // SLT
    assign r_arr[3'd3] = a < b; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = funct7[5] ? $signed($signed(a) >>> b[4:0]) : a >> b[4:0]; // SRA/SRL
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    assign r = r_arr[funct3];
endmodule

module icache(
    input clk,
    input [31:0] addr,
    output valid,
    output [31:0] data
);
    assign valid = 1'b1;
    assign data = 32'h00000000;
endmodule

module dcache(
    input clk,
    input w_ena,
    input [31:0] addr,
    input [1:0] width,
    input ext, // if extend with sign
    input [31:0] data_in,
    output valid,
    output [31:0] data_out
);
    assign valid = 1'b1;
    assign data_out = 32'd0;
endmodule
