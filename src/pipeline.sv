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

typedef struct packed { logic i, u, s, r, r4, b, j; } i_type_t;

typedef struct packed { logic valid; logic [63:0] pc; } pc_if0_t;
typedef struct packed { logic valid; logic [4:0] delta; } if0_pc_t;
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
    logic [63:0] pc;
    logic [31:0] op;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [63:0] imm;
    logic [4:0] rs1addr, rs2addr, rdaddr, uimm;
    logic compressed, illegal;
} id_ex_t;
typedef struct packed { logic valid; } ex_ma_t;
typedef struct packed { logic valid; } ma_wb_t;

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
    ma_wb_t data_ma_wb;     logic get_ma_wb;

    pc_stage pc_stage_inst(.clk(clk), .rst(rst),
        .get_if0(get_if0_pc), .in_if0(data_if0_pc),
        .ena_if0(get_pc_if0), .out_if0(data_pc_if0));
    if0_stage if0_stage_inst(.clk(clk), .rst(rst),
        .get_pc(get_pc_if0), .in_pc(data_pc_if0),
        .ena_pc(get_if0_pc), .out_pc(data_if0_pc),
        .ena_if1(get_if0_if1), .out_if1(data_if0_if1),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_done(icache_done), .icache_data(icache_data));
    if1_stage if1_stage_inst(.clk(clk), .rst(rst),
        .get_if0(get_if0_if1), .in_if0(data_if0_if1),
        .ena_id(get_if1_id), .out_id(data_if1_id));
    id_stage id_stage_inst(.clk(clk), .rst(rst),
        .get_if1(get_if1_id), .in_if1(data_if1_id),
        .ena_ex(get_id_ex), .out_ex(data_id_ex));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst),
        .get_id(get_id_ex), .in_id(data_id_ex),
        .ena_ma(get_ex_ma), .out_ma(data_ex_ma));
    ma_stage ma_stage_inst(.clk(clk), .rst(rst),
        .get_ex(get_ex_ma), .in_ex(data_ex_ma),
        .ena_wb(get_ma_wb), .out_wb(data_ma_wb));
    wb_stage wb_stage_inst(.clk(clk), .rst(rst),
        .get_ma(get_ma_wb), .in_ma(data_ma_wb));
endmodule

module pc_stage(input logic clk, input logic rst,
    output logic get_if0, input if0_pc_t in_if0,
    input logic ena_if0, output pc_if0_t out_if0
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
    output logic get_pc, input pc_if0_t in_pc,
    input logic ena_pc, output if0_pc_t out_pc,
    input logic ena_if1, output if0_if1_t out_if1,
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
    output logic get_if0, input if0_if1_t in_if0,
    input logic ena_id, output if1_id_t out_id
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
    output logic get_if1, input if1_id_t in_if1,
    input logic ena_ex, output id_ex_t out_ex
);
    logic [31:0] ir, op;
    i_type_t enc; // instruction type
    always_comb ir = in_if1.ir;
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
    always_ff @(posedge clk) if (ena_ex) out_ex.pc <= in_if1.pc;
    always_ff @(posedge clk) if (ena_ex) out_ex.compressed <= in_if1.compressed;
    always_ff @(posedge clk) if (ena_ex) out_ex.funct3 <= ir[14:12];
    always_ff @(posedge clk) if (ena_ex) out_ex.funct7 <= ir[31:25];
    always_ff @(posedge clk) if (ena_ex) out_ex.uimm <= ir[19:15]; // same as rs1addr
    always_ff @(posedge clk) if (ena_ex) out_ex.op <= op;
    always_ff @(posedge clk)
        out_ex.imm <= {64{enc.i}} & {{53{ir[31]}}, ir[30:20]} |
                   {64{enc.u}} & {{32{ir[31]}}, ir[31:12], 12'd0} |
                   {64{enc.s}} & {{53{ir[31]}}, ir[30:25], ir[11:7]} |
                   {64{enc.b}} & {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} |
                   {64{enc.j}} & {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
    always_ff @(posedge clk)
        if (op[`SYSTEM] & ir[14]) out_ex.rs1addr <= 5'd0; // CSR[*]I
        else if (enc.i | enc.s | enc.r | enc.r4 | enc.b) out_ex.rs1addr <= ir[19:15];
        else out_ex.rs1addr <= 5'd0;
    always_ff @(posedge clk) out_ex.rs2addr <= enc.r | enc.s | enc.b ? ir[24:20] : 5'd0;
    always_ff @(posedge clk)
        out_ex.rdaddr <= enc.i | enc.u | enc.r | enc.r4 | enc.j ? ir[11:7] : 5'd0;
    always_ff @(posedge clk)
        if (rst) {out_ex.valid, out_ex.illegal} <= 0;
        else if (ena_ex) begin
            out_ex.valid <= in_if1.valid;
            out_ex.illegal <= in_if1.ir[1:0] != 2'b11;
        end
    always_comb get_if1 = ena_ex | ~in_if1.valid;
endmodule

module ex_stage(input logic clk, input logic rst,
    output logic get_id, input id_ex_t in_id,
    input logic ena_ma, output ex_ma_t out_ma
);
    always_comb out_ma.valid = 1'b1;
    always_comb get_id = 1'b1;
endmodule

module ma_stage(input logic clk, input logic rst,
    output logic get_ex, input ex_ma_t in_ex,
    input logic ena_wb, output ma_wb_t out_wb
);
    always_comb out_wb.valid = 1'b1;
    always_comb get_ex = 1'b1;
endmodule

module wb_stage(input logic clk, input logic rst,
    output logic get_ma, input ma_wb_t in_ma
);
    always_comb get_ma = 1'b1;
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