`timescale 1ns/1ns
module core(
    input clk,
    input rst
);
    reg ena_if, ena_ex, ena_ma, ena_wb;
    wire clk_if, clk_ex, clk_ma, clk_wb;
    assign clk_if = clk & ena_if;
    assign clk_ex = clk & ena_ex;
    assign clk_ma = clk & ena_ma;
    assign clk_wb = clk & ena_wb;

    // pipeline registers
    reg [31:0]  pc;
    reg [31:0]  pc_if;      reg [31:0]  pc_ex;      reg [31:0]  pc_ma;
    wire [5:0]  mux_if;     reg [5:0]   mux_ex;     reg [5:0]   mux_ma;
    wire [2:0]  funct3_if;  reg [2:0]   funct3_ex;
    wire        mem_w_if;   reg         mem_w_ex;
    wire [31:0] imm_if;     reg [31:0]  imm_ex;     reg [31:0]  imm_ma;
    wire [31:0] rs2_if;     reg [31:0]  rs2_ex;
    wire [4:0]  rd_addr_if; reg [4:0]   rd_addr_ex; reg [4:0]   rd_addr_ma;
                            wire [31:0] r_ex;       reg [31:0]  r_ma;
                            wire [31:0] d_out_ex;   wire [31:0] d_out_ma;

    // Instruction fetch & decode
    wire icache_valid;
    wire [6:0] opcode;
    wire [5:0] enc;
    wire [31:0] ir;
    wire op_imm, lui, auipc, op, jal, jalr, branch, load, store;
    wire [4:0] rs1_addr, rs2_addr;
    wire [6:0] funct7;
    icache icache_inst(.clk(clk_if), .addr(pc), .valid(icache_valid), .data(ir));
    always @(posedge clk_if) pc_if <= pc;
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
    assign rs1_addr = (enc & 6'b111100) == 6'd0 ? 5'd0 : ir[19:15];
    assign rs2_addr = (enc & 6'b101100) == 6'd0 ? 5'd0 : ir[24:20];
    assign rd_addr_if = (enc & 6'b110011) == 6'd0 ? 5'd0 : ir[11:7];
    assign imm_if = enc[4] ? {{21{ir[31]}}, ir[30:20]} : (                        // I type
                    enc[3] ? {{21{ir[31]}}, ir[30:25], ir[11:7]} : (              // S type
                    enc[2] ? {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} : ( // B type
                    enc[1] ? {ir[31:12], 12'd0} : (                               // U type
                    enc[0] ? {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} : // J type
                             32'd0))));                                           // R type
    // MUX selection signal: H -> L
    // PC input (5), PC adder (4), ALU a (3), ALU b (2), rd input (1:0)
    assign mux_if = {jal | jalr, branch, auipc | jal, branch | op,
                     load | jal | jalr, load | lui};
    // Handle data bypass in IF -> EX
    wire [31:0] rd_byp_ex, rd_byp_ma;
    wire [31:0] a, b, rs1_byp, rs2_if_byp, rs1;
    assign rd_byp_ex = mux_ex[1] ? (mux_ex[0] ? d_out_ex : pc_ex + 32'd4) : (mux_ex[0] ? imm_ex : r_ex);
    assign rd_byp_ma = mux_ma[1] ? (mux_ma[0] ? d_out_ma : pc_ma + 32'd4) : (mux_ma[0] ? imm_ma : r_ma);
    assign dout_ex = 32'd0; // unable to fetch previous d_out before current EX stage
    assign rs1_byp    = ena_ma & rd_addr_ex == rs1_addr ? rd_byp_ex :
                       (ena_wb & rd_addr_ma == rs1_addr ? rd_byp_ma : rs1);
    assign rs2_if_byp = ena_ma & rd_addr_ex == rs2_addr ? rd_byp_ex :
                       (ena_wb & rd_addr_ma == rs2_addr ? rd_byp_ma : rs2_if);

    // Execution
    always @(posedge clk_ex) begin
        pc_ex      <= pc_if;
        mux_ex     <= mux_if;
        funct3_ex  <= funct3_if;
        mem_w_ex   <= mem_w_if;
        imm_ex     <= imm_if;
        rs2_ex     <= rs2_if;
        rd_addr_ex <= rd_addr_if;
    end
    wire [3:0] flags; // carry, negative, (placeholder), zero
    assign a = mux_if[3] ? pc_if : rs1_byp;
    assign b = mux_if[2] ? rs2_if_byp : imm_if;
    alu alu_inst(.clk(clk_ex), .a(a), .b(b), .r(r_ex), .c(flags[3]), 
                 .funct3(op | op_imm ? funct3_if : 3'b000),
                 .funct7(op | op_imm & funct3_if == 3'b101 // SRL/SRA (special I-type)
                         ? funct7 : (branch ? 7'b0100000 : 7'b0000000)));
    assign flags[2] = r_ex[31];
    assign flags[0] = r_ex == 32'd0;
    assign b_suc = mux_ex[4] & flags[funct3_ex[2:1]] == ~funct3_ex[0];
    // Handle data bypass in EX -> MA
    wire [31:0] rs2_ex_byp;
    assign rs2_ex_byp = ena_wb & rd_addr_ma == rs2_addr ? rd_byp_ma : rs2_ex;

    // Memory access
    always @(posedge clk_ma) begin
        pc_ma      <= pc_ex;
        mux_ma     <= mux_ex;
        imm_ma     <= imm_ex;
        rd_addr_ma <= rd_addr_ex;
        r_ma       <= r_ex;
    end
    always @(posedge clk)
        pc <= rst ? 32'h00400000
            : (mux_ex[5] & ena_ma ? r_ex : (b_suc & ena_ma ? pc_ex + imm_ex : pc + 4));
    wire dcache_valid;
    dcache dcache_inst(.clk(clk_ma), .w_ena(mem_w_ex), .addr(r_ex),
                       .width(funct3_ex[1:0]), .ext(funct3_ex[2]),
                       .data_in(rs2_ex_byp), .valid(dcache_valid), .data_out(d_out_ma));

    // Write back
    gpreg gpreg_inst(.clk(clk_wb), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr),
                     .rd_addr(rd_addr_ma), .rd(rd_byp_ma), .rs1(rs1), .rs2(rs2_if));

    // Enable control
    wire jump;
    assign jump = mux_ex[5] | b_suc;
    always @(posedge clk) begin
        ena_if <= 1'b1;
        ena_ex <= rst | ena_ex & jump ? 1'b0 : ena_if;
        ena_ma <= rst | ena_ma & jump ? 1'b0 : ena_ex;
        ena_wb <= rst ? 1'b0 : ena_ma;
    end
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
    input clk,
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
    assign r_arr[3'd2] = $signed(a) < $signed(b); // SLT
    assign r_arr[3'd3] = a < b; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = funct7[5] ? $signed($signed(a) >>> b[4:0]) : a >> b[4:0]; // SRA/SRL
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    always @(posedge clk) r <= r_arr[funct3];
    always @(posedge clk) c <= c_wire;
endmodule

module icache(
    input clk,
    input [31:0] addr,
    output valid,
    output reg [31:0] data
);
    assign valid = 1'b1;
    integer fd;
    initial
        fd <= $fopen("/home/ubuntu/Desktop/test.dump", "r");
    always @(posedge clk) begin
        $fseek(fd, ((addr-32'h00400000)>>2)*9, 0);
        $fscanf(fd, "%h", data);
    end
endmodule

module dcache(
    input clk,
    input w_ena,
    input [31:0] addr,
    input [1:0] width,
    input ext, // if extend with sign
    input [31:0] data_in,
    output valid,
    output reg [31:0] data_out
);
    assign valid = 1'b1;
    reg [32:0] mem[0:1024];
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            mem[i] = 1;
    end
    always @(posedge clk) begin
        data_out <= mem[addr - 32'h10010000];
        if (w_ena)
            mem[addr - 32'h10010000] <= data_in;
    end
endmodule
