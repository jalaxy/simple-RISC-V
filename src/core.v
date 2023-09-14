`timescale 1ns/1ns
module core(
    input clk,
    input rst
);
    wire clk_if;
    reg [31:0] pc;
    wire [31:0] ir;
    wire icache_valid;
    wire [31:0] imm;
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire op_imm, lui, auipc, op, jal, jalr, branch, load, store;
    wire [4:0] rs1_addr, rs2_addr, rd_addr;

    always@(posedge clk_if)
        pc = rst ? 32'd0 : npc;

    assign clk_if = clk;
    icache icache_inst(.clk(clk_if), .addr(pc), .valid(icache_valid), .data(ir));
    assign opcode = ir[6:0];
    assign funct3 = ir[14:12];
    assign funct7 = ir[31:25];
    assign op_imm = opcode == 7'b0010011;
    assign lui    = opcode == 7'b0110111;
    assign auipc  = opcode == 7'b0010111;
    assign op     = opcode == 7'b0110011;
    assign jal    = opcode == 7'b1101111;
    assign jalr   = opcode == 7'b1100111;
    assign branch = opcode == 7'b1100011;
    assign load   = opcode == 7'b0000011;
    assign store  = opcode == 7'b0100011;
    assign rs1_addr = encoding & 6'b111100 == 6'd0 ? 5'd0 : ir[19:15];
    assign rs2_addr = encoding & 6'b101100 == 6'd0 ? 5'd0 : ir[24:20];
    assign rd_addr = encoding & 6'b110011 == 6'd0 ? 5'd0 : ir[11:7];
    assign imm = op_imm | jalr | load ? {21{ir[31]}, ir[30:20]} : (          // I type
                 store ? {21{ir[31]}, ir[30:25], ir[11:7]} : (               // S type
                 branch ? {20{ir[31]}, ir[7], ir[30:25], ir[11:8], 1'b0} : ( // B type
                 lui | auipc ? {ir[31:12], 12'd0} : (                        // U type
                 jal ? {12{ir[31]}, ir[19:12], ir[20], ir[30:21], 1'b0} :    // J type
                               32'd0)))));                                   // R type
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
    always@(posedge clk)
        r[rd_addr] <= rd;
endmodule

module alu(
    input [31:0] a,
    input [31:0] b,
    input [2:0] func3,
    input [6:0] func7,
    output [31:0] r
);
    wire [31:0] r_arr[7:0];
    assign r_arr[3'd0] = func7[5] ? a - b : a + b; // ADD/SUB
    assign r_arr[3'd1] = a << b[4:0]; // SLL
    assign r_arr[3'd2] = $signed(a) < $signed(b); // SLT
    assign r_arr[3'd3] = a < b; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = func7[5] ? $signed($signed(a) >>> b[4:0]) : a >> b[4:0]; // SRA/SRL
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    assign r = r_arr[func3];
endmodule

module icache(
    input clk,
    input [31:0] addr,
    output valid,
    output [31:0] data,
);
endmodule

module dcache(
    input clk,
    input w_ena,
    input [31:0] addr,
    input [2:0] width,
    input ext, // if extend with sign
    input [31:0] data_in,
    output valid,
    output [31:0] data_out,
);
endmodule
