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
    output signal
);
    wire ena_pc, ena_if, ena_ex, ena_ma, ena_wb;

    // pipeline registers
    reg [31:0]  pc;
    reg [31:0]  pc_if;       reg [31:0]  pc_ex;
    wire [31:0] op_if;       reg [31:0]  op_ex;      reg [31:0]  op_ma;
    wire [2:0]  funct3_if;   reg [2:0]   funct3_ex;
    wire [31:0] imm_if;      reg [31:0]  imm_ex;     reg [31:0]  imm_ma;
    wire [31:0] rs2_if;      reg [31:0]  rs2_ex;
    wire [4:0]  rs2_addr_if; reg [31:0]  rs2_addr_ex;
    wire [4:0]  rd_addr_if;  reg [4:0]   rd_addr_ex; reg [4:0]   rd_addr_ma;
                             wire [31:0] r_ex;       reg [31:0]  r_ma;
                             wire [31:0] d_out_ex;   wire [31:0] d_out_ma;

    // Pipeline start control
    reg start_if, start_ex, start_ma; // each stage has been started
    wire b_suc, ma_ex_stall;
    always @(posedge clk) begin // Jump can be implemented by start control as program restart from new PC
        start_if <= rst | start_ex & op_ex[`BRANCH] & b_suc
                        | start_if & op_if[`JAL]
                        | start_if & op_if[`JALR] & ~ma_ex_stall ? 1'b0 : 1'b1;
        start_ex <= rst | start_ex & op_ex[`BRANCH] & b_suc ? 1'b0 : start_if;
        start_ma <= rst ? 1'b0 : start_ex;
    end

    // Instruction fetch & decode
    wire icache_valid;
    wire [6:0] opcode;
    wire [5:0] enc;
    wire [31:0] ir;
    wire [4:0] rs1_addr;
    wire [6:0] funct7;
    icache_synth icache_inst(.clk(clk), .ena(ena_if), .addr(pc), .valid(icache_valid), .data(ir));
    always @(posedge clk) pc_if <= ena_if ? pc : pc_if;
    assign opcode    = ir[6:0];
    assign funct3_if = ir[14:12];
    assign funct7    = ir[31:25];
    assign op_if[`OP_IMM] = opcode[6:2] == `OP_IMM;
    assign op_if[`LUI]    = opcode[6:2] == `LUI;
    assign op_if[`AUIPC]  = opcode[6:2] == `AUIPC;
    assign op_if[`OP]     = opcode[6:2] == `OP;
    assign op_if[`JAL]    = opcode[6:2] == `JAL;
    assign op_if[`JALR]   = opcode[6:2] == `JALR;
    assign op_if[`BRANCH] = opcode[6:2] == `BRANCH;
    assign op_if[`LOAD]   = opcode[6:2] == `LOAD;
    assign op_if[`STORE]  = opcode[6:2] == `STORE;
    assign enc = {op_if[`OP], op_if[`OP_IMM] | op_if[`JALR] | op_if[`LOAD], op_if[`STORE],
                  op_if[`BRANCH], op_if[`LUI] | op_if[`AUIPC], op_if[`JAL]}; // R I S B U J
    assign rs1_addr = (enc & 6'b111100) == 6'd0 ? 5'd0 : ir[19:15];
    assign rs2_addr_if = (enc & 6'b101100) == 6'd0 ? 5'd0 : ir[24:20];
    assign rd_addr_if = (enc & 6'b110011) == 6'd0 ? 5'd0 : ir[11:7];
    assign imm_if = enc[4] ? {{21{ir[31]}}, ir[30:20]} : (                        // I type
                    enc[3] ? {{21{ir[31]}}, ir[30:25], ir[11:7]} : (              // S type
                    enc[2] ? {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} : ( // B type
                    enc[1] ? {ir[31:12], 12'd0} : (                               // U type
                    enc[0] ? {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} : // J type
                             32'd0))));                                           // R type
    // Handle data bypass in IF -> EX
    wire [31:0] rd_byp_ex, rd_byp_ma;
    wire [31:0] a, b, rs1_byp, rs2_if_byp, rs1;
    assign d_out_ex = 32'd0; // Unable to fetch data of previous instructions due to pipeline
    assign rd_byp_ex = op_ex[`LOAD] ? d_out_ex : (op_ex[`LUI] ? imm_ex : r_ex); // RD of previous instructions
    assign rd_byp_ma = op_ma[`LOAD] ? d_out_ma : (op_ma[`LUI] ? imm_ma : r_ma);
    assign rs1_byp    = rs1_addr == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs1_addr ? rd_byp_ex :
                        (start_ma & rd_addr_ma == rs1_addr ? rd_byp_ma : rs1));
    assign rs2_if_byp = rs2_addr_if == 5'd0 ? 32'd0 :
                        (start_ex & rd_addr_ex == rs2_addr_if ? rd_byp_ex :
                        (start_ma & rd_addr_ma == rs2_addr_if ? rd_byp_ma : rs2_if));

    // Execution
    always @(posedge clk) begin
        pc_ex       <= ena_ex ? pc_if : pc_ex;
        funct3_ex   <= ena_ex ? funct3_if : funct3_ex;
        op_ex       <= ena_ex ? op_if : op_ex;
        imm_ex      <= ena_ex ? imm_if : imm_ex;
        rs2_ex      <= ena_ex ? rs2_if : rs2_ex;
        rs2_addr_ex <= ena_ex ? rs2_addr_if : rs2_addr_ex;
        rd_addr_ex  <= ma_ex_stall ? 5'd0 : (ena_ex ? rd_addr_if : rd_addr_ex); // NOP when stall
    end
    wire [3:0] flags; // carry, negative, (placeholder), zero
    assign a = op_if[`AUIPC] | op_if[`JAL] | op_if[`JALR] ? pc_if : rs1_byp;
    assign b = op_if[`BRANCH] | op_if[`OP] ? rs2_if_byp : (op_if[`JAL] | op_if[`JALR] ? 32'd4 : imm_if);
    alu alu_inst(.clk(clk), .ena(ena_ex), .a(a), .b(b), .r(r_ex), .c(flags[3]), 
                 .funct3(op_if[`OP] | op_if[`OP_IMM] ? funct3_if : 3'b000),
                 .funct7(op_if[`OP] | op_if[`OP_IMM] & funct3_if == 3'b101 // SRL/SRA (special I-type)
                         ? funct7 : (op_if[`BRANCH] ? 7'b0100000 : 7'b0000000)));
    assign flags[2] = r_ex[31];
    assign flags[0] = r_ex == 32'd0;
    assign b_suc = flags[funct3_ex[2:1]] == ~funct3_ex[0];
    // Handle data bypass in EX -> MA
    wire [31:0] rs2_ex_byp;
    assign rs2_ex_byp = rs2_addr_ex == 5'd0 ? 32'd0 :
                        (start_ma & rd_addr_ma == rs2_addr_ex ? rd_byp_ma : rs2_ex);

    // Memory access
    always @(posedge clk) begin
        op_ma      <= ena_ma ? op_ex : op_ma;
        imm_ma     <= ena_ma ? imm_ex : imm_ma;
        rd_addr_ma <= ena_ma ? rd_addr_ex : rd_addr_ma;
        r_ma       <= ena_ma ? r_ex : r_ma;
    end
    wire dcache_valid;
    dcache dcache_inst(.clk(clk), .r_ena(ena_ma), .w_ena(op_ex[`STORE]), .addr(r_ex),
                       .width(funct3_ex[1:0]), .ext(funct3_ex[2]),
                       .data_in(rs2_ex_byp), .valid(dcache_valid), .data_out(d_out_ma));

    // Write back
    gpreg gpreg_inst(.clk(clk), .ena(ena_wb), .rs1_addr(rs1_addr), .rs2_addr(rs2_addr_if),
                     .rd_addr(rd_addr_ma), .rd(rd_byp_ma), .rs1(rs1), .rs2(rs2_if));

    // PC control
    wire [31:0] npc;
    assign npc = start_ex & op_ex[`BRANCH] & b_suc ? pc_ex + imm_ex :
                (start_if & op_if[`JALR] ? rs1_byp + imm_if :
                (start_if & op_if[`JAL] ? pc_if + imm_if : pc + 4));
    always @(posedge clk)
        pc <= rst ? 32'h00400000 : (ena_pc ? npc : pc);

    // Enable control
    assign ma_ex_stall = ((~(op_if[`AUIPC] | op_if[`JAL]) & rs1_addr != 5'd0 & start_ex & rd_addr_ex == rs1_addr) |
                          ( (op_if[`BRANCH] | op_if[`OP]) & rs2_addr_if != 5'd0 & start_ex & rd_addr_ex == rs2_addr_if)) & // LUI???
                            op_ex[`LOAD]; // some wires in this expression are existed
    // EX/MA/WB is enabled only when IF/EX/MA been started (IF/EX/MA registers are ready)
    assign ena_pc = ~ma_ex_stall;
    assign ena_if = ~ma_ex_stall;
    assign ena_ex = ~ma_ex_stall & start_if;
    assign ena_ma = start_ex;
    assign ena_wb = start_ma;

    assign signal = pc == 32'h0040007c;
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
    assign r_arr[3'd2] = $signed(a) < $signed(b); // SLT
    assign r_arr[3'd3] = a < b; // SLTU
    assign r_arr[3'd4] = a ^ b; // XOR
    assign r_arr[3'd5] = funct7[5] ? $signed($signed(a) >>> b[4:0]) : a >> b[4:0]; // SRA/SRL
    assign r_arr[3'd6] = a | b; // OR
    assign r_arr[3'd7] = a & b; // AND
    always @(posedge clk) r <= ena ? r_arr[funct3] : r;
    always @(posedge clk) c <= ena ? c_wire : c;
endmodule

module icache(
    input clk,
    input ena,
    input [31:0] addr,
    output valid,
    output reg [31:0] data
);
    assign valid = 1'b1;
    integer fd, res;
    initial
        fd <= $fopen("/home/ubuntu/Desktop/test.dump", "r");
    always @(posedge clk) begin
        if (ena) begin
            res <= $fseek(fd, ((addr-32'h00400000)>>2)*9, 0);
            res <= $fscanf(fd, "%h", data);
        end
    end
endmodule

module icache_synth(
    input clk,
    input ena,
    input [31:0] addr,
    output valid,
    output reg [31:0] data
);
    assign valid = 1'b1;
    wire [31:0] data_w, addr_relative;
    assign addr_relative = addr - 32'h00400000;
    rom rom_inst(.a(addr_relative[9:0] >> 2), .spo(data_w));
    always @(posedge clk) data <= ena ? data_w : data;
endmodule

module dcache(
    input clk,
    input r_ena,
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
    always @(posedge clk) begin
        if (r_ena)
            data_out <= mem[(addr-32'h10010000)>>2];
        if (w_ena)
            mem[(addr-32'h10010000)>>2] <= data_in;
    end
endmodule
