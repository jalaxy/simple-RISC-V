`define RST_PC 64'h400000 // reset pc
`define PHTSZ 4096 // patter history table
`define BTBSZ 512 // branch target table
`define PTSZ  8    // pending table
`define CQSZ  16   // commit queue
`define LSQSZ 8    // load store queue
`define lgPHTSZ $clog2(`PHTSZ)
`define lgBTBSZ $clog2(`BTBSZ)
`define lgPTSZ  $clog2(`PTSZ)
`define lgCQSZ  $clog2(`CQSZ)
`define lgLSQSZ $clog2(`LSQSZ)
`define PTLEN $bits(id_ex_t)
`define CQLEN $bits(cqinfo_t)
`define MULLEN $bits(mul_rqst_t)
`define DIVLEN $bits(div_rqst_t)
`define FPULEN $bits(fpu_rqst_t)
`define LSULEN $bits(lsu_rqst_t)
`define RQSTLEN `FPULEN
`define LATENUM 8 // number of late components
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
`define EX_ADD    6'd0 // execution stage operations
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
`define EX_MUL    6'd12
`define EX_MULH   6'd13
`define EX_MULHSU 6'd14
`define EX_MULHU  6'd15
`define EX_DIV    6'd16
`define EX_DIVU   6'd17
`define EX_REM    6'd18
`define EX_REMU   6'd19
`define EX_FADD   6'd20
`define EX_FSUB   6'd21
`define EX_FMUL   6'd22
`define EX_FNMUL  6'd23
`define EX_FDIV   6'd24
`define EX_FSQRT  6'd25
`define EX_FSGNJ  6'd26
`define EX_FSGNJN 6'd27
`define EX_FSGNJX 6'd28
`define EX_FMIN   6'd29
`define EX_FMAX   6'd30
`define EX_FEQ    6'd31
`define EX_FLT    6'd32
`define EX_FLE    6'd33
`define EX_FMVXF  6'd34
`define EX_FCLASS 6'd35
`define EX_FMVFX  6'd36
`define EX_FCVTIF 6'd37
`define EX_FCVTFI 6'd38
`define EX_FCVTSD 6'd39
`define EX_FCVTDS 6'd40
`define EX_LOAD   6'd41
`define EX_STORE  6'd42
`define EX_FENCE  6'd43
`define EX_FENCEI 6'd44
`define EX_SFENCE 6'd45
`define EX_CSR    6'd46
`define EX_ECALL  6'd47
`define EX_EBREAK 6'd48
`define EX_RET    6'd49
`define EX_INV    6'd50
`define EX_END    6'd51

typedef struct packed {
    logic valid, branch;
    logic [63:0] pc;
    logic [3:0] num;
    logic [7:0][1:0] pat;
} pc_if_t;
typedef struct packed {
    logic valid, pf;
    logic [63:0] pc;
    logic [31:0] ir;
    logic [17:0] pat;
} if_id_t;
typedef struct packed {
    logic valid, pf;
    logic [`lgCQSZ:0] cqid;
    logic [2:0] delta;
    logic [63:0] pc;
    logic [17:0] pat;
    logic [`EX_END-1:0] exop;
    logic [1:0][6:0] rsa;
    logic iword, isign;
    logic [2:0] funct3, bmask;
    logic fdouble, bneg, j;
    logic [64:0] base;
    logic [63:0] offset;
    logic [1:0] rsrv, aqrl;
    logic [64:0] a, b;
    logic [6:0] rda;
} id_ex_t;
typedef struct packed {
    logic valid;
    logic [6:0] cause;
    logic [64:0] rd;
    logic [63:0] npc, pc;
    logic mem, patupd, b, c, fencei;
    logic [6:0] rda;
    logic [17:0] pat;
    logic [2:0] ret;
} ex_wb_t;
typedef struct packed {
    logic reinf, redir, c;
    logic [63:0] pc, npc;
    logic [1:0] pat;
    logic [15:0] gh;
} xx_pc_t;
typedef struct packed {
    logic [63:0] pc;
    logic mem, patupd, b, c, fencei;
    logic [6:0] rda;
    logic [17:0] pat;
    logic [2:0] ret;
} cqinfo_t;
typedef struct packed {
    logic [`lgCQSZ:0] id;
    logic [4:0] op;
    logic [63:0] a, b;
} mul_rqst_t;
typedef struct packed {
    logic [`lgCQSZ:0] id;
    logic [7:0] op;
    logic [63:0] a, b;
} div_rqst_t;
typedef struct packed {
    logic [`lgCQSZ:0] id;
    logic [20:0] op;
    logic [63:0] a, b;
    logic [2:0] rm;
    logic double;
} fpu_rqst_t;
typedef struct packed {
    logic [`lgCQSZ:0] id;
    logic [11:0] fence;
    logic csr, wena;
    logic [1:0] rsrv, aqrl;
    logic [64:0] addr, wdat;
    logic [2:0] bits;
} lsu_rqst_t;
typedef struct packed {
    logic mul, div, fpu, lsu;
    logic [`RQSTLEN-1:0] out;
} rqst_t;

module pipeline(
    input logic clk,
    input logic rst,

    output logic [63:0] csr_satp,

    output logic         icache_rqst,
    output logic [63:0]  icache_addr,
    output logic         icache_flsh,
    input  logic         icache_done,
    input  logic         icache_pgft,
    input  logic [127:0] icache_data,

    output logic [`lgCQSZ:0] dcache_rqst,
    output logic       [1:0] dcache_rsrv,
    output logic             dcache_wena,
    output logic      [63:0] dcache_addr,
    output logic       [2:0] dcache_bits,
    input  logic [`lgCQSZ:0] dcache_done,
    input  logic       [1:0] dcache_pgft,
    input  logic      [63:0] dcache_rdat,
    output logic      [63:0] dcache_wdat,
    output logic             dcache_flsh
);
    pc_if_t data_pc_if; logic get_pc_if;
    if_id_t [3:0] data_if_id; logic [3:0] get_if_id;
    id_ex_t [3:0] data_id_ex; logic [3:0] get_id_ex;
    ex_wb_t [3:0] data_ex_wb; logic [3:0] get_ex_wb;
    id_ex_t [3:0] data_pt_ex; logic [3:0] get_pt_ex;
    id_ex_t [3:0] data_ex_pt; logic [3:0] get_ex_pt;
    xx_pc_t data_if_pc; logic get_if_pc;
    xx_pc_t data_wb_pc; logic get_wb_pc;
    logic [3:0][1:0][6:0] raddr; logic[3:0] [1:0][64:0] rvalue;
    logic [3:0][`lgCQSZ:0] late_done; logic [3:0][64:0] late_val;
    logic [3:0][64:0] late_npc; logic [3:0][6:0] late_cause;
    logic [3:0][`lgCQSZ:0] pt_done; logic [3:0][64:0] pt_data;
    logic [3:0] pt_ena; logic [3:0][64:0] pt_npc;
    logic [3:0][`lgCQSZ:0] addr_done; logic [3:0][63:0] addr_val;
    mul_rqst_t [3:0] mul_rqst; div_rqst_t [3:0] div_rqst;
    fpu_rqst_t [3:0] fpu_rqst; lsu_rqst_t [3:0] lsu_rqst; logic [3:0] lsu_free;
    logic [`lgCQSZ:0] mul_done; logic mul_exc, mul_ena; logic [64:0] mul_r;
    logic [`lgCQSZ:0] div_done; logic div_exc, div_ena; logic [64:0] div_r;
    logic [`lgCQSZ:0] fpu_done; logic fpu_exc, fpu_ena; logic [64:0] fpu_r;
    logic [`lgCQSZ:0] lsu_done; logic lsu_ena;
    logic [64:0] lsu_rdat; logic [6:0] lsu_cause;
    logic [`lgCQSZ:0] frontid, nextid; logic redir;
    logic [11:0] csr_addr; logic csr_wena; logic [2:0] csr_func;
    logic [63:0] csr_rval, csr_wval; logic csr_excp;
    logic [6:0] cause; logic [63:0] epc, tval, nret; logic [2:0] ret;
    logic [63:0] csr_tvec, csr_mepc, csr_sepc;

    pc_stage pc_stage_inst(.clk(clk), .rst(rst),
        .in_if(data_if_pc), .get_if(get_if_pc),
        .in_wb(data_wb_pc), .get_wb(get_wb_pc),
        .out_if(data_pc_if), .ena_if(get_pc_if));
    if_stage if_stage_inst(.clk(clk), .rst(rst), .redir(redir),
        .in_pc(data_pc_if), .get_pc(get_pc_if), .redir_wb(data_wb_pc),
        .out_pc(data_if_pc), .ena_pc(get_if_pc),
        .out_id(data_if_id), .ena_id(get_if_id),
        .icache_rqst(icache_rqst), .icache_addr(icache_addr),
        .icache_flsh(icache_flsh), .icache_done(icache_done),
        .icache_pgft(icache_pgft), .icache_data(icache_data));
    id_stage id_stage_inst(.clk(clk), .rst(rst), .redir(redir),
        .in_if(data_if_id), .get_if(get_if_id),
        .out_ex(data_id_ex), .ena_ex(get_id_ex));
    ex_stage ex_stage_inst(.clk(clk), .rst(rst), .redir(redir),
        .in_id(data_id_ex), .get_id(get_id_ex),
        .in_pt(data_pt_ex), .get_pt(get_pt_ex),
        .out_wb(data_ex_wb), .ena_wb(get_ex_wb),
        .out_pt(data_ex_pt), .ena_pt(get_ex_pt),
        .raddr(raddr), .rvalue(rvalue),
        .mul_rqst(mul_rqst), .div_rqst(div_rqst), .fpu_rqst(fpu_rqst),
        .lsu_rqst(lsu_rqst), .lsu_free(lsu_free),
        .late_done(late_done), .late_val(late_val),
        .pt_done(pt_done), .pt_data(pt_data), .pt_npc(pt_npc), .ena_arb(pt_ena),
        .addr_done(addr_done), .addr_val(addr_val), .level(csr_inst.level));
    wb_stage wb_stage_inst(.clk(clk), .rst(rst), .redir(redir),
        .in_ex(data_ex_wb), .get_ex(get_ex_wb),
        .out_pc(data_wb_pc), .ena_pc(get_wb_pc),
        .frontid(frontid), .nextid(nextid),
        .raddr(raddr), .rvalue(rvalue), .tvec(csr_tvec),
        .mepc(csr_mepc), .sepc(csr_sepc),
        .late_done(late_done), .late_val(late_val),
        .late_npc(late_npc), .late_cause(late_cause),
        .nret(nret), .epc(epc), .tval(tval), .cause(cause), .ret(ret));
    pending_table pending_table_inst(.clk(clk), .rst(rst), .flush(redir),
        .in_ex(data_ex_pt), .get_ex(get_ex_pt),
        .out_ex(data_pt_ex), .ena_ex(get_pt_ex),
        .late_done(late_done), .late_val(late_val));
    lsu lsu_inst(.clk(clk), .rst(rst), .flush(redir),
        .ena(lsu_ena), .get(lsu_free), .frontid(frontid), .nextid(nextid),
        .rqst(lsu_rqst), .done(lsu_done), .cause(lsu_cause), .rdata(lsu_rdat),
        .late_done(late_done), .late_val(late_val),
        .addr_done(addr_done), .addr_val(addr_val),
        .csr_addr(csr_addr), .csr_wena(csr_wena), .csr_func(csr_func),
        .csr_rval(csr_rval), .csr_wval(csr_wval), .csr_excp(csr_excp),
        .dcache_rqst(dcache_rqst), .dcache_rsrv(dcache_rsrv),
        .dcache_wena(dcache_wena), .dcache_flsh(dcache_flsh),
        .dcache_addr(dcache_addr), .dcache_bits(dcache_bits),
        .dcache_done(dcache_done), .dcache_pgft(dcache_pgft),
        .dcache_rdat(dcache_rdat), .dcache_wdat(dcache_wdat)
    );
    mul mul_inst(.clk(clk), .rst(rst), .flush(redir), .ena(mul_ena),
        .rqst(mul_rqst), .done(mul_done), .r(mul_r), .e(mul_exc));
    div div_inst(.clk(clk), .rst(rst), .flush(redir), .ena(div_ena),
        .rqst(div_rqst), .done(div_done), .r(div_r), .e(div_exc));
    fpu fpu_inst(.clk(clk), .rst(rst), .flush(redir), .ena(fpu_ena),
        .rqst(fpu_rqst), .done(fpu_done), .r(fpu_r), .e(fpu_exc));
    csr csr_inst(.clk(clk), .rst(rst), .addr(csr_addr), .wena(csr_wena),
        .rval(csr_rval), .wval(csr_wval), .func(csr_func), .eout(csr_excp),
        .nret(nret), .ret(ret), .ein(redir & cause[6]),
        .epc(epc), .tval(tval), .cause(cause[5:0]),
        .csr_tvec(csr_tvec), .csr_mepc(csr_mepc),
        .csr_sepc(csr_sepc), .csr_satp(csr_satp));
    arbiter arbiter_inst( /* PT should have lowest priority to avoid deadlock,
                             or use dynamic priority? */
        .done_in({pt_done, fpu_done, div_done, mul_done, lsu_done}),
        .val_in({pt_data, fpu_r, div_r, mul_r, lsu_rdat}),
        .npc_in({pt_npc, 65'd0, 65'd0, 65'd0, 65'd0}),
        .cause_in({28'd0, 7'd0, 7'd0, 7'd0, lsu_cause}),
        .get({pt_ena, fpu_ena, div_ena, mul_ena, lsu_ena}),
        .done_out(late_done), .val_out(late_val),
        .npc_out(late_npc), .cause_out(late_cause));
endmodule

module pc_stage(input logic clk, input logic rst,
    input  xx_pc_t in_if, output logic get_if,
    input  xx_pc_t in_wb, output logic get_wb,
    output pc_if_t out_if, input logic ena_if
);
    logic [63:0] pc; logic branch; logic [3:0] num; // in half-word
    logic [1:0] pht[`PHTSZ-1:0]; logic [63:0] btb[`BTBSZ-1:0];
    logic [7:0][`lgPHTSZ-1:0] phtra; logic [7:0] [1:0] phtrv;
    logic [7:0][`lgBTBSZ-1:0] btbra; logic [7:0][63:0] btbrv;
    logic [`lgPHTSZ-1:0] phtwa; logic  [1:0] phtwv; logic phtwe;
    logic [`lgBTBSZ-1:0] btbwa; logic [63:0] btbwv; logic btbwe;
    xx_pc_t in_upd;
    always_comb if (in_wb.redir) in_upd = in_wb;
        else    if (in_if.redir) in_upd = in_if;
        else                     in_upd = in_wb;
    always_comb get_wb = ~in_if.redir; // to control branch reinforcement
    always_comb for (int i = 0; i < 8; i++) phtra[i] = pc[`lgPHTSZ:1] + i[`lgPHTSZ-1:0];
    always_comb for (int i = 0; i < 8; i++) btbra[i] = pc[`lgBTBSZ:1] + i[`lgBTBSZ-1:0];
    always_comb phtwe = in_upd.reinf | in_upd.redir;
    always_comb phtwa = in_upd.pc[`lgPHTSZ:1] + (in_upd.c ? 0 : 1);
    always_comb phtwv = in_upd.redir ? (in_upd.pat[0] ? 2'b10 : 2'b01) :
                                       (in_upd.pat[1] ? 2'b11 : 2'b00);
    always_comb btbwe = in_upd.redir & in_upd.pat == 2'b01;
    always_comb btbwa = in_upd.pc[`lgBTBSZ:1] + (in_upd.c ? 0 : 1);
    always_comb btbwv = in_upd.npc;
    always_comb for (int i = 0; i < 8; i++) phtrv[i] = pht[phtra[i]];
    always_comb for (int i = 0; i < 8; i++) btbrv[i] = btb[btbra[i]];
    always_ff @(posedge clk) if (phtwe) pht[phtwa] <= phtwv;
    always_ff @(posedge clk) if (btbwe) btb[btbwa] <= btbwv;
    always_ff @(posedge clk) if (rst) pc <= `RST_PC; else begin
        if (ena_if) pc <= pc + 64'({num, 1'b0});
        if (ena_if) for (int i = 7; i >= 0; i--) if (phtrv[i][1]) pc <= btbrv[i];
        if (in_if.redir) pc <= in_if.npc;
        if (in_wb.redir) pc <= in_wb.npc;
    end
    always_comb begin
        {branch, num} = 8;
        for (int i = 7; i >= 0; i--) if (phtrv[i][1])
            {branch, num} = {1'b1, i[3:0] + 3'd1};
        if (|pc[11:1] & 11'(num) > -pc[11:1]) num = -pc[4:1]; // avoid beyond-page fetch
    end
    always_comb out_if.valid = ~in_wb.redir & ~in_if.redir;
    always_comb out_if.pc = pc;
    always_comb out_if.num = num;
    always_comb out_if.branch = branch;
    always_comb out_if.pat = phtrv;
endmodule

module if_stage(input logic clk, input logic rst, input logic redir,
    input  pc_if_t in_pc,  output logic get_pc, input xx_pc_t redir_wb,
    output xx_pc_t out_pc, input  logic ena_pc, // always enabled
    output if_id_t [3:0] out_id, input  logic [3:0] ena_id,
    output logic         icache_rqst,
    output logic [63:0]  icache_addr,
    output logic         icache_flsh,
    input  logic         icache_done,
    input  logic         icache_pgft,
    input  logic [127:0] icache_data
);
    logic [4:0] remnum, isnum, getnum; logic [3:0] getinstnum, instnum;
    logic [15:0][15:0] remdat, isdat; logic [15:0][63:0] rempc, ispc;
    logic [15:0][1:0] rempat, ispat; logic [15:0] remb, isb, rempf, ispf;
    logic [3:0] icnum; logic [7:0] icb, icpf;
    logic [7:0][15:0] icdat; logic [7:0][63:0] icpc; logic [7:0][1:0] icpat;
    logic [3:0][3:0] irpos; logic [3:0] irvalid, irb, irpf;
    logic [3:0][63:0] irpc; logic [3:0][31:0] irdat;
    logic [3:0][1:0] irpat; logic [3:0][15:0] irgh;
    logic [15:0] isnc; logic busy; logic [15:0] ghr, ghnew; logic [1:0] incomp;
    always_comb isdat = ({128'd0, icache_data} << {remnum, 4'b0}) | remdat;
    always_comb ispc = ({512'd0, icpc} << {remnum, 6'd0}) | rempc;
    always_comb ispat = ({16'd0, icpat} << {remnum, 1'b0}) | rempat;
    always_comb isb = ({8'd0, icb} << remnum) | remb;
    always_comb ispf = ({8'd0, icpf} << remnum) | rempf;
    always_comb isnum = remnum + (icache_done ? {1'b0, icnum} : 5'd0);
    always_comb for (int i = 0; i < 16; i++) isnc[i] = &isdat[i][1:0];
    always_comb for (int i = 0; i < 4; i++) irvalid[i] =
        |isnum & {1'b0, irpos[i]} + (isnc[irpos[i]] ? 5'd2 : 5'd1) <= isnum;
    always_comb for (int i = 0; i < 4; i++) irpc[i] = ispc[irpos[i]];
    always_comb for (int i = 0; i < 4; i++) irpf[i] = ispf[irpos[i]];
    always_comb for (int i = 0; i < 4; i++) irdat[i] = isdat[irpos[i]+1-:2];
    always_comb for (int i = 0; i < 4; i++) irb[i] = irvalid[i] &(
        irdat[i][6:0] == 7'b1100011 |                        // BRANCH
        irdat[i][15:13] == 3'b110 & irdat[i][1:0] == 2'b01 | // C.BEQZ
        irdat[i][15:13] == 3'b111 & irdat[i][1:0] == 2'b01); // C.BNEZ
    always_comb for (int i = 0; i < 4; i++)
        irpat[i] = ispat[irpos[i] + 4'(isnc[irpos[i]])];
    always_comb for (int i = 0; i < 4; i++) begin irgh[i] = ghr;
        for (int j = 0; j <= i; j++) if (irb[j])
            irgh[i] = (irgh[i] << 1) | {15'd0, irpat[j][1]}; end
    always_comb begin ghnew = ghr; for (int i = 0; i < 4; i++)
        if (irvalid[i]) ghnew = irgh[i]; end
    always_comb begin
        irpos[0] = 0;
        if (~isnc[0]) irpos[1] = 1; else irpos[1] = 2;
        if (~isnc[0] & ~isnc[1]) irpos[2] = 2;
        else if (~isnc[0] & isnc[1] | isnc[0] & ~isnc[2]) irpos[2] = 3;
        else irpos[2] = 4;
        if (~isnc[0] & ~isnc[1] & ~isnc[2]) irpos[3] = 3;
        else if (~isnc[0] & ~isnc[1] & isnc[2] | ~isnc[0] & isnc[1] & ~isnc[3] |
            isnc[0] & ~isnc[2] & ~isnc[3]) irpos[3] = 4;
        else if (~isnc[0] & isnc[1] & isnc[3] | isnc[0] & ~isnc[2] & isnc[3] |
            isnc[0] & isnc[2] & ~isnc[4]) irpos[3] = 5;
        else irpos[3] = 6;
    end
    always_ff @(posedge clk) begin
        if (rst | redir) remnum <= 0;
        else if (out_pc.redir) remnum <= irpos[incomp] + 1 - getnum;
        else remnum <= isnum - getnum;
        remdat <= (~(-256'd1 << {isnum, 4'd0}) & isdat) >> {getnum, 4'd0};
        rempc <= (~(-1024'd1 << {isnum, 6'd0}) & ispc) >> {getnum, 6'd0};
        rempat <= (~(-32'd1 << {isnum, 1'd0}) & ispat) >> {getnum, 1'd0};
        rempf <= (~(-16'd1 << isnum) & ispf) >> getnum;
        remb <= (~(-16'd1 << isnum) & isb) >> getnum;
        if (out_pc.redir) remb <= 0; // signal b is only used for incomplete instruction
    end
    always_ff @(posedge clk) if (rst | redir) icnum <= 0;
        else if (icache_rqst) icnum <= in_pc.num;
        else if (icache_done) icnum <= 0;
    always_ff @(posedge clk) if (icache_rqst) begin icb <= 0;
        if (in_pc.branch) icb[in_pc.num - 1] <= 1; end
    always_ff @(posedge clk) if (icache_rqst) icpat <= in_pc.pat;
    always_ff @(posedge clk) if (icache_rqst) for (int i = 0; i < 8; i++)
        icpc[i] <= in_pc.pc + 64'({i, 1'b0});
    always_ff @(posedge clk) if (rst) ghr <= 0;
        else if (redir_wb.redir) ghr <= redir_wb.gh;
        else if (|getinstnum) ghr <= irgh[32'(getinstnum) - 1];
    always_comb busy = |icnum & ~icache_done;
    always_comb begin
        {instnum, getinstnum, getnum} = 0;
        for (int i = 0; i < 4; i++) if (irvalid[i]) instnum++;
        for (int i = 0; i < 4; i++) if (ena_id[i]) getinstnum++; else break;
        if (getinstnum > instnum) getinstnum = instnum;
        for (int i = 0; i < getinstnum; i++)
            if (isnc[irpos[i]]) getnum += 2; else getnum += 1;
    end
    always_comb get_pc = ~in_pc.valid | ~busy & isnum - getnum <= 8;
    for (genvar i = 0; i < 4; i++) begin
        always_comb out_id[i].valid =
            ~redir & irvalid[i] & ~(out_pc.redir & i >= incomp);
        always_comb out_id[i].pc = irpc[i];
        always_comb out_id[i].pat = {irgh[i], irpat[i]};
        always_comb out_id[i].ir = irdat[i];
        always_comb out_id[i].pf = irpf[i];
    end
    always_comb begin {out_pc, incomp} = 0; for (int i = 7; i>= 0; i--)
        if (irvalid[i] & isnc[irpos[i]] & isb[irpos[i]]) begin
            incomp = i[1:0];
            out_pc.reinf = 0;
            out_pc.redir = 1;
            out_pc.c = 1;
            out_pc.pc = ispc[irpos[i]];
            out_pc.npc = ispc[irpos[i]] + 2;
            out_pc.pat = 2'b10;
            out_pc.gh = ghnew;
        end end
    always_comb icdat = icache_data;
    always_comb icpf = {8{icache_pgft}};
    always_comb icache_rqst = ~rst & get_pc & in_pc.valid;
    always_comb icache_addr = in_pc.pc;
    always_comb icache_flsh = redir;
endmodule

module id_stage(input logic clk, input logic rst, input logic redir,
    input  if_id_t [3:0] in_if, output logic [3:0] get_if,
    output id_ex_t [3:0] out_ex, input logic [3:0] ena_ex
);
    id_ex_t [3:0][2:0] res; id_ex_t [3:0] res4; logic [2:0] iter4;
    logic [2:0] numin, numout, numrem; logic [3:0][2:0] numval;
    logic [`lgCQSZ-1:0] cqid;
    for (genvar g = 0; g < 4; g++) begin : decoder
        logic [2:0][`EX_END-1:0] exop;
        logic [31:0] ir, op; logic [63:0] imm;
        logic [2:0][64:0] a, b; logic [2:0] delta;
        ci2i ci2i_inst(.ci(in_if[g].ir), .i(ir));
        always_comb for (int i = 0; i < 32; i++)
            op[i] = ir[6:2] == i[4:0];
        always_comb delta = &in_if[g].ir[1:0] ? 4 : 2;
        always_comb imm =
            {{53{ir[31]}}, ir[30:20]} & {64{
                op[`LOAD] | op[`LOAD_FP] | op[`MISC_MEM] | op[`OP_IMM] |
                op[`OP_IMM_32] | op[`JALR] | op[`SYSTEM]}} | // I type
            {{32{ir[31]}}, ir[31:12], 12'd0} & {64{
                op[`AUIPC] | op[`LUI]}} | // U type
            {{53{ir[31]}}, ir[30:25], ir[11:7]} & {64{
                op[`STORE] | op[`STORE_FP]}} | // S type
            {{52{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0} & {64{
                op[`BRANCH]}} | // B type
            {{44{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} & {64{
                op[`JAL]}}; // J type
        always_comb begin
            exop[1] =
                (1 << `EX_ADD)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00001}} |
                (1 << `EX_ADD)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00000}} |
                (1 << `EX_XOR)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b00100}} |
                (1 << `EX_AND)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b01100}} |
                (1 << `EX_OR)   & {`EX_END{op[`AMO] & ir[31:27] == 5'b01000}} |
                (1 << `EX_MIN)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b10000}} |
                (1 << `EX_MIN)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b11000}} |
                (1 << `EX_MAX)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b10100}} |
                (1 << `EX_MAX)  & {`EX_END{op[`AMO] & ir[31:27] == 5'b11100}} |
                (1 << `EX_FADD) & {`EX_END{op[`MADD] | op[`NMSUB]}} |
                (1 << `EX_FSUB) & {`EX_END{op[`MSUB] | op[`NMADD]}};
            exop[2] = (1 << `EX_STORE) & {`EX_END{op[`AMO] & |exop[1]}};
            exop[0] = 0;
            exop[0][`EX_ADD] =
                op[`JALR] | op[`AUIPC] | op[`LUI] | op[`JAL] |
                (op[`OP_IMM] | op[`OP_IMM_32]) & ir[14:12] == 3'b000 |
                (op[`OP] | op[`OP_32]) & ir[14:12] == 3'b000 & ir[31:25] == 7'd0;
            exop[0][`EX_SUB] = op[`BRANCH] |
                (op[`OP] | op[`OP_32]) & ir[14:12] == 3'b000 & ir[31:25] == 7'b0100000;
            exop[0][`EX_SLL] =
                op[`OP_IMM] & ir[14:12] == 3'b001 & ir[31:26] == 6'd0 |
                (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                    ir[14:12] == 3'b001 & ir[31:25] == 7'd0;
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
                (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                    ir[14:12] == 3'b101 & ir[31:25] == 7'd0;
            exop[0][`EX_SRA] =
                op[`OP_IMM] & ir[14:12] == 3'b101 & ir[31:26] == 6'b010000 |
                (op[`OP] | op[`OP_IMM_32] | op[`OP_32]) &
                    ir[14:12] == 3'b101 & ir[31:25] == 7'b0100000;
            exop[0][`EX_OR] =
                op[`OP_IMM] & ir[14:12] == 3'b110 |
                op[`OP] & ir[14:12] == 3'b110 & ir[31:25] == 7'd0;
            exop[0][`EX_AND] =
                op[`OP_IMM] & ir[14:12] == 3'b111 |
                op[`OP] & ir[14:12] == 3'b111 & ir[31:25] == 7'd0;
            exop[0][`EX_MUL] = (op[`OP] | op[`OP_32]) &
                ir[14:12] == 3'b000 & ir[31:25] == 7'b1;
            exop[0][`EX_MULH] = op[`OP] & ir[14:12] == 3'b001 & ir[31:25] == 7'b1;
            exop[0][`EX_MULHSU] = op[`OP] & ir[14:12] == 3'b010 & ir[31:25] == 7'b1;
            exop[0][`EX_MULHU] = op[`OP] & ir[14:12] == 3'b011 & ir[31:25] == 7'b1;
            exop[0][`EX_DIV] = (op[`OP] | op[`OP_32]) &
                ir[14:12] == 3'b100 & ir[31:25] == 7'b1;
            exop[0][`EX_DIVU] = (op[`OP] | op[`OP_32]) &
                ir[14:12] == 3'b101 & ir[31:25] == 7'b1;
            exop[0][`EX_REM] = (op[`OP] | op[`OP_32]) &
                ir[14:12] == 3'b110 & ir[31:25] == 7'b1;
            exop[0][`EX_REMU] = (op[`OP] | op[`OP_32]) &
                ir[14:12] == 3'b111 & ir[31:25] == 7'b1;
            exop[0][`EX_FADD] = op[`OP_FP] & ir[31:26] == 6'b000000;
            exop[0][`EX_FSUB] = op[`OP_FP] & ir[31:26] == 6'b000010;
            exop[0][`EX_FMUL] = op[`OP_FP] & ir[31:26] == 6'b000100 |
                op[`MADD] | op[`MSUB];
            exop[0][`EX_FNMUL] = op[`NMADD] | op[`NMSUB];
            exop[0][`EX_FDIV] = op[`OP_FP] & ir[31:26] == 6'b000110;
            exop[0][`EX_FSQRT] = op[`OP_FP] & ir[31:26] == 6'b010110 &
                ir[24:20] == 5'd0;
            exop[0][`EX_FSGNJ] = op[`OP_FP] & ir[31:26] == 6'b001000 &
                ir[14:12] == 3'b000;
            exop[0][`EX_FSGNJN] = op[`OP_FP] & ir[31:26] == 6'b001000 &
                ir[14:12] == 3'b001;
            exop[0][`EX_FSGNJX] = op[`OP_FP] & ir[31:26] == 6'b001000 &
                ir[14:12] == 3'b010;
            exop[0][`EX_FMIN] = op[`OP_FP] & ir[31:26] == 6'b001010 &
                ir[14:12] == 3'b000;
            exop[0][`EX_FMAX] = op[`OP_FP] & ir[31:26] == 6'b001010 &
                ir[14:12] == 3'b001;
            exop[0][`EX_FEQ] = op[`OP_FP] & ir[31:26] == 6'b101000 &
                ir[14:12] == 3'b010;
            exop[0][`EX_FLT] = op[`OP_FP] & ir[31:26] == 6'b101000 &
                ir[14:12] == 3'b001;
            exop[0][`EX_FLE] = op[`OP_FP] & ir[31:26] == 6'b101000 &
                ir[14:12] == 3'b000;
            exop[0][`EX_FMVXF] = op[`OP_FP] & ir[31:26] == 6'b111000 &
                ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
            exop[0][`EX_FCLASS] = op[`OP_FP] & ir[31:26] == 6'b111000 &
                ir[14:12] == 3'b001 & ir[24:20] == 5'd0;
            exop[0][`EX_FMVFX] = op[`OP_FP] & ir[31:26] == 6'b111100 &
                ir[14:12] == 3'b000 & ir[24:20] == 5'd0;
            exop[0][`EX_FCVTIF] = op[`OP_FP] & ir[31:26] == 6'b110000;
            exop[0][`EX_FCVTFI] = op[`OP_FP] & ir[31:26] == 6'b110100;
            exop[0][`EX_FCVTSD] = op[`OP_FP] & ir[31:25] == 7'b0100000 &
                ir[24:20] == 5'd1;
            exop[0][`EX_FCVTDS] = op[`OP_FP] & ir[31:25] == 7'b0100001 &
                ir[24:20] == 5'd0;
            exop[0][`EX_LOAD] = op[`LOAD] | op[`LOAD_FP] |
                op[`AMO] & (ir[31:27] == 5'b00010 | |exop[1]);
            exop[0][`EX_STORE] = op[`STORE] | op[`STORE_FP] |
                op[`AMO] & ir[31:27] == 5'b00011;
            exop[0][`EX_FENCE] = op[`MISC_MEM] & ir[14:12] == 3'b000;
            exop[0][`EX_FENCEI] = op[`MISC_MEM] & ir[14:12] == 3'b001;
            exop[0][`EX_SFENCE] = op[`SYSTEM] & ir[31:25] == 7'b0001001;
            exop[0][`EX_CSR] = op[`SYSTEM] & |ir[13:12];
            exop[0][`EX_ECALL] = ir == 32'h00000073;
            exop[0][`EX_EBREAK] = ir == 32'h00100073;
            exop[0][`EX_RET] = (ir & ~(32'd3 << 28)) == 32'h00200073;
            if (ir[1:0] != 2'b11) exop[0] = 0;
            if (~|exop[0]) exop[0][`EX_INV] = 1;
        end
        always_comb numval[g] = |exop[2] ? 3 : (|exop[1] ? 2 : (|exop[0] ? 1 : 0));
        always_comb if (exop[0][`EX_FCVTFI] | exop[0][`EX_FMVFX])
                a[0] = {1'd1, 59'd0, ir[19:15]};
            else a[0] = {~(op[`SYSTEM] & ir[14]), 59'd0, ir[19:15]} & {65{
                        op[`LOAD]   | op[`LOAD_FP]  | op[`OP_IMM] | op[`OP_IMM_32] |
                        op[`STORE]  | op[`STORE_FP] | op[`OP]     | op[`OP_32]     |
                        op[`BRANCH] | op[`AMO]      | op[`SYSTEM]}} |
                    {1'd1, 59'd1, ir[19:15]} & {{65{
                        op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                        op[`OP_FP]}}} |
                    {1'd0, in_if[g].pc} & {65{op[`JALR] | op[`JAL] | op[`AUIPC]}};
        always_comb a[1] = {1'd1, 59'd2, 5'd0} & {65{
            ~(op[`AMO] & ir[31:27] == 5'b00001)}};
        always_comb a[2] = {1'd1, 59'd0, ir[19:15]} & {65{op[`AMO]}};
        always_comb if (exop[0][`EX_FCVTIF] | exop[0][`EX_FCVTFI])
                b[0] = {60'd0, ir[24:20]};
            else if (exop[0][`EX_FSQRT] | exop[0][`EX_FCVTDS] | exop[0][`EX_FCVTSD] |
                exop[0][`EX_FMVFX] | exop[0][`EX_FMVXF])
                b[0] = 65'd0;
            else b[0] = {1'd1, 59'd0, ir[24:20]} & {65{
                        op[`OP] | op[`OP_32] | op[`OP_FP] | op[`BRANCH]}} |
                    {1'd1, 59'd1, ir[24:20]} & {{65{
                        op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                        op[`OP_FP]}}} |
                    {1'd0, imm} & {65{              op[`SYSTEM] | op[`MISC_MEM]  |
                        op[`LOAD]  | op[`LOAD_FP] | op[`OP_IMM] | op[`OP_IMM_32] |
                        op[`AUIPC] | op[`LUI]     | op[`STORE]  | op[`STORE_FP]}} |
                    {62'd0, delta} & {65{op[`JAL] | op[`JALR]}};
        always_comb b[1] = {1'd1, 59'd0, ir[24:20]} & {65{op[`AMO]}} |
                        {1'd1, 59'd1, ir[31:27]} & {65{
                            op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};
        always_comb b[2] = {1'd0, imm} & {65{op[`AMO]}};
        always_comb begin
            res[g][0].valid = |exop[0];
            res[g][0].pf = in_if[g].pf;
            res[g][0].delta = |exop[1] ? 3'd0 : delta;
            res[g][0].pc = in_if[g].pc;
            res[g][0].pat = in_if[g].pat;
            res[g][0].a = a[0];
            res[g][0].b = b[0];
            res[g][0].exop = exop[0];
            res[g][0].bmask = {3{op[`BRANCH]}} &
                {ir[14:13] == 2'b00, ir[14:13] == 2'b10, ir[14:13] == 2'b11};
            res[g][0].bneg = ir[12];
            res[g][0].j = op[`JAL] | op[`JALR];
            res[g][0].base = {1'b0, in_if[g].pc} & {65{op[`JAL] | op[`BRANCH]}}|
                                {1'b1, 59'd0, ir[19:15]} & {65{op[`JALR]}};
            res[g][0].offset = imm & {64{op[`JAL] | op[`JALR] | op[`BRANCH]}};
            res[g][0].iword = op[`OP_32] | op[`OP_IMM_32];
            res[g][0].isign = (op[`OP_32] | op[`OP_IMM_32]) & ir[30];
            res[g][0].funct3 = exop[0][`EX_RET] ? {1'b0, ir[29:28]} : ir[14:12];
            res[g][0].fdouble = ir[25];
            res[g][0].rsrv = {op[`AMO] & |exop[1], op[`AMO] & ~|exop[1]};
            res[g][0].aqrl = {op[`AMO] & ir[26], op[`AMO] & ir[25]};
            if (a[0][64]) res[g][0].rsa[0] = a[0][6:0];
                else if (op[`JALR]) res[g][0].rsa[0] = {2'd0, ir[19:15]};
                else res[g][0].rsa[0] = 0;
            if (b[0][64]) res[g][0].rsa[1] = b[0][6:0];
                else if (op[`STORE]) res[g][0].rsa[1] = {2'b0, ir[24:20]};
                else if (op[`STORE_FP]) res[g][0].rsa[1] = {2'b1, ir[24:20]};
                else if (op[`AMO] & ir[31:27] == 5'b00011)
                    res[g][0].rsa[1] = {2'b0, ir[24:20]};
                else res[g][0].rsa[1] = 0;
            if (exop[0][`EX_FEQ] | exop[0][`EX_FLT] | exop[0][`EX_FLE] |
                exop[0][`EX_FMVXF] | exop[0][`EX_FCLASS] | exop[0][`EX_FCVTIF])
                res[g][0].rda = {2'd0, ir[11:7]};
            else res[g][0].rda =
                {2'd0, ir[11:7]} & {7{
                    op[`LOAD] | op[`OP_IMM] | op[`AUIPC] | op[`OP_IMM_32] |
                    op[`OP]   | op[`LUI]    | op[`OP_32] | op[`JALR]      |
                    op[`JAL]  | op[`SYSTEM] | op[`AMO] & ~|exop[1]}} |
                {2'd1, ir[11:7]} & {7{op[`LOAD_FP] | op[`OP_FP]}} |
                {2'd2, 5'd0} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD] |
                                    op[`AMO] & |exop[1]}};
            res[g][1].valid = |exop[1];
            res[g][1].delta = |exop[2] ? 3'd0 : delta;
            res[g][1].pc = in_if[g].pc;
            res[g][1].pat = in_if[g].pat;
            res[g][1].a = a[1];
            res[g][1].b = b[1];
            res[g][1].exop = exop[1];
            res[g][1].iword = op[`AMO] & ir[14:12] == 3'b010;
            res[g][1].isign = op[`AMO] & ~ir[30];
            res[g][1].rsrv = 0;
            res[g][1].bmask = 0;
            res[g][1].j = 0;
            res[g][1].funct3 = ir[14:12];
            res[g][1].fdouble = ir[25];
            res[g][1].rsa[0] = a[1][6:0];
            res[g][1].rsa[1] = b[1][6:0];
            res[g][1].rda =
                {2'd2, 5'd0} & {7{op[`AMO]}} |
                {2'd1, ir[11:7]} & {7{op[`MADD] | op[`MSUB] | op[`NMSUB] | op[`NMADD]}};
            res[g][2].valid = |exop[2];
            res[g][2].delta = delta;
            res[g][2].pc = in_if[g].pc;
            res[g][2].pat = in_if[g].pat;
            res[g][2].a = a[2];
            res[g][2].b = b[2];
            res[g][2].exop = exop[2];
            res[g][2].rsrv = {op[`AMO] & |exop[1], 1'b0};
            res[g][2].bmask = 0;
            res[g][2].j = 0;
            res[g][2].funct3 = ir[14:12];
            res[g][2].rsa[0] = a[2][6:0];
            res[g][2].rsa[1] = {2'd2, 5'd0};
            res[g][2].rda = {2'd0, ir[11:7]};
        end
    end
    always_comb begin numrem = 0; numout = 0;
        for (int i = 0; i < 4; i++) if (out_ex[i].valid)
            if (ena_ex[i]) numout++; else numrem++; end
    always_comb begin numin = 0; get_if = 0;
        for (int i = 0; i < 4; i++) if (in_if[i].valid)
            if (numrem + numin + numval[i] <= 4) begin
                numin = numin + numval[i];
                get_if[i] = 1;
            end else break; end
    always_comb begin res4 = 0; iter4 = 0;
        for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++)
            if (res[i][j].valid & iter4 < 4) begin
                res4[iter4[1:0]] = res[i][j]; iter4++; end end
    always_ff @(posedge clk) for (int i = 0; i < 4; i++)
        if (rst | redir) out_ex[i].valid <= 0;
        else if (i[2:0] < numrem) out_ex[i] <= out_ex[i + 32'(numout)];
        else if (i[2:0] < numrem + numin) begin
            out_ex[i] <= res4[i - 32'(numrem)];
            out_ex[i].cqid <= {1'b1, cqid + `lgCQSZ'(i) - `lgCQSZ'(numrem)};
        end else out_ex[i] <= 0;
    always_ff @(posedge clk) if (rst | redir) cqid <= 0;
        else cqid <= cqid + `lgCQSZ'(numin);
endmodule

module ex_stage(input logic clk, input logic rst, input logic redir,
    input id_ex_t [3:0] in_id, output logic [3:0] get_id,
    input id_ex_t [3:0] in_pt, output logic [3:0] get_pt,
    output ex_wb_t [3:0] out_wb, input logic [3:0] ena_wb,
    output logic [3:0][1:0][6:0] raddr, input logic [3:0][1:0][64:0] rvalue,
    output id_ex_t [3:0] out_pt, input logic [3:0] ena_pt,
    output mul_rqst_t [3:0] mul_rqst, output div_rqst_t [3:0] div_rqst,
    output fpu_rqst_t [3:0] fpu_rqst,
    output lsu_rqst_t [3:0] lsu_rqst, input logic [3:0] lsu_free,
    input logic [3:0][`lgCQSZ:0] late_done, input logic [3:0][64:0] late_val,
    output logic [3:0][`lgCQSZ:0] pt_done, output logic [3:0][64:0] pt_data,
    output logic [3:0][64:0] pt_npc, input logic [3:0] ena_arb,
    output logic [3:0][`lgCQSZ:0] addr_done, output logic [3:0][63:0] addr_val,
    input logic [1:0] level
);
    logic [3:0] get_id_g, get_pt_g; ex_wb_t [3:0] out_wb_g;
    logic [`CQSZ-1:0] cqidocc; rqst_t [3:0] rqst_g; id_ex_t [3:0] out_pt_g;
    logic [1:0] mul_i, div_i, fpu_i, lsu_i, pt_i;
    logic [2:0] numrem, numin, numout;
    always_comb begin numrem = 0; numout = 0;
        for (int i = 0; i < 4; i++) if (out_wb[i].valid)
            if (ena_wb[i]) numout++; else numrem++; end
    always_comb begin
        get_id = 0; get_pt = 0; numin = 0;
        mul_i = 0; div_i = 0; fpu_i = 0; lsu_i = 0; pt_i = 0;
        mul_rqst = 0; div_rqst = 0; fpu_rqst = 0; lsu_rqst = 0; out_pt = 0;
        for (int i = 0; i < 4; i++) begin
            get_id[i] = get_id_g[i];
            for (int j = 0; j < i; j++) if (|in_id[j].rda & (
                raddr[i][0] == in_id[j].rda | raddr[i][1] == in_id[j].rda))
                    get_id[i] = 0;
            if (get_id[i] & rqst_g[i].mul) begin
                mul_rqst[mul_i] = rqst_g[i].out[`MULLEN-1:0]; mul_i++; end
            if (get_id[i] & rqst_g[i].div) begin
                div_rqst[div_i] = rqst_g[i].out[`DIVLEN-1:0]; div_i++; end
            if (get_id[i] & rqst_g[i].fpu) begin
                fpu_rqst[fpu_i] = rqst_g[i].out[`FPULEN-1:0]; fpu_i++; end
            if (get_id[i] & rqst_g[i].lsu)
                if (lsu_free[lsu_i] & ~(out_pt_g[i].valid & ~ena_pt[pt_i])) begin
                    lsu_rqst[lsu_i] = rqst_g[i].out[`LSULEN-1:0]; lsu_i++;
                end else get_id[i] = 0;
            if (get_id[i] & out_pt_g[i].valid)
                if (ena_pt[pt_i] & get_id[i]) begin
                    out_pt[pt_i] = out_pt_g[i]; pt_i++;
                end else get_id[i] = 0;
            if (get_id[i]) numin++;
            if (~get_id[i] | numrem + numin == 4) break;
        end
        for (int i = 0; i < 4; i++) if (get_pt_g[i])
            if (rqst_g[i].mul) begin
                mul_rqst[mul_i] = rqst_g[i].out[`MULLEN-1:0]; mul_i++; get_pt[i] = 1;
            end else if (rqst_g[i].div) begin
                div_rqst[div_i] = rqst_g[i].out[`DIVLEN-1:0]; div_i++; get_pt[i] = 1;
            end else if (rqst_g[i].fpu) begin
                fpu_rqst[fpu_i] = rqst_g[i].out[`FPULEN-1:0]; fpu_i++; get_pt[i] = 1;
            end else get_pt[i] = 1;
    end
    always_ff @(posedge clk) for (int i = 0; i < 4; i++)
        if (rst | redir) out_wb[i].valid <= 0;
        else if (i[2:0] < numrem) begin
            out_wb[i] <= out_wb[i + 32'(numout)];
            for (int j = 0; j < 4; j++)
                if (out_wb[i + 32'(numout)].rd[64] &
                    late_done[j] == out_wb[i + 32'(numout)].rd[`lgCQSZ:0])
                    out_wb[i].rd <= late_val[j];
        end else if (i[2:0] < numrem + numin) out_wb[i] <= out_wb_g[i - 32'(numrem)];
        else out_wb[i] <= 0;
    for (genvar g = 0; g < 4; g++) begin : execution_unit
        logic frompt, fromid;
        id_ex_t in; logic [1:0][64:0] rval;
        logic mul, div, fpu, lsu, pt;
        always_comb frompt = in_pt[g].valid & (ena_arb[g] | ~pt_done[g][`lgCQSZ] |
            in_pt[g].exop[`EX_LOAD] | in_pt[g].exop[`EX_STORE]);
        always_comb fromid = in_id[g].valid & get_id[g];
        always_comb in = frompt ? in_pt[g] : in_id[g];
        always_comb raddr[g] = in_id[g].rsa;
        always_comb rval = frompt ? 0 : rvalue[g];
        logic [`EX_END-1:0] op;
        logic [63:0] a, b;
        logic jump;
        logic [63:0] jpc;
        logic [64:0] sub, res;
        logic [63:0] add, sll, srl, sra;
        logic [2:0] bflag;
        mul_rqst_t mul_rqst; div_rqst_t div_rqst;
        fpu_rqst_t fpu_rqst; lsu_rqst_t lsu_rqst;
        always_comb op = in.valid ? in.exop : 0;
        always_comb begin
            if (frompt & in_pt[g].j) a = in.pc; // JALR in PT
            else a = in.a[64] ? rval[0][63:0] : in.a[63:0];
            b = in.b[64] ? rval[1][63:0] : in.b[63:0];
            if (in.iword) a = {{32{a[31] & in.isign}}, a[31:0]};
            if (in.iword) b = {{32{b[31] & in.isign}}, b[31:0]};
            if (in.iword & (in.exop[`EX_SLL] | in.exop[`EX_SRL] | in.exop[`EX_SRA]))
                b[5] = 0;
        end
        always_comb sub = {1'b0, a} - {1'b0, b};
        always_comb add = a + b;
        always_comb sll = a << b[5:0];
        always_comb srl = a >> b[5:0];
        always_comb sra = $signed($signed(a) >>> b[5:0]);
        always_comb bflag = {~|sub, sub[63], sub[64]}; // zero, negative, carry
        always_comb if (op[`EX_CSR]) pt = 0;
            else if (lsu) pt = in.valid & rval[0][64];
            else pt = in.valid & (rval[0][64] | rval[1][64]);
        always_comb begin
            out_pt_g[g] = in; out_pt_g[g].valid = pt;
            out_pt_g[g].a = in.a[64] | in.base[64] ? rval[0] : in.a;
            out_pt_g[g].b = in.b[64] ? rval[1] : in.b;
        end
        always_comb begin
            get_pt_g[g] = frompt;
            get_id_g[g] = ~cqidocc[in.cqid[`lgCQSZ-1:0]] & ~frompt & ena_wb[g];
        end
        always_comb begin
            mul_rqst.op = {5{~pt}} & {op[`EX_MUL] & in.iword,
                op[`EX_MULHU], op[`EX_MULHSU], op[`EX_MULH], op[`EX_MUL] & ~in.iword};
            mul = |mul_rqst.op;
            mul_rqst.id = {`lgCQSZ+1{mul}} & in.cqid;
            {mul_rqst.a, mul_rqst.b} = {a, b};
        end
        always_comb begin
            div_rqst.op = {8{~pt}} &
                {op[`EX_REMU] & in.iword,  op[`EX_REM] & in.iword,
                 op[`EX_DIVU] & in.iword,  op[`EX_DIV] & in.iword,
                 op[`EX_REMU] & ~in.iword, op[`EX_REM] & ~in.iword,
                 op[`EX_DIVU] & ~in.iword, op[`EX_DIV] & ~in.iword};
            div = |div_rqst.op;
            div_rqst.id = {`lgCQSZ+1{div}} & in.cqid;
            {div_rqst.a, div_rqst.b} = {a, b};
        end
        always_comb begin
            fpu_rqst.op = {21{~pt}} & {
                op[`EX_FCVTDS], op[`EX_FCVTSD], op[`EX_FCVTFI], op[`EX_FCVTIF],
                op[`EX_FMVFX],  op[`EX_FCLASS], op[`EX_FMVXF],  op[`EX_FLE],
                op[`EX_FLT],    op[`EX_FEQ],    op[`EX_FMAX],   op[`EX_FMIN],
                op[`EX_FSGNJX], op[`EX_FSGNJN], op[`EX_FSGNJ],  op[`EX_FSQRT],
                op[`EX_FDIV],   op[`EX_FNMUL],  op[`EX_FMUL],   op[`EX_FSUB],
                op[`EX_FADD]};
            fpu = |fpu_rqst.op;
            fpu_rqst.id = {`lgCQSZ+1{fpu}} & in.cqid;
            {fpu_rqst.a, fpu_rqst.b} = {a, b};
            {fpu_rqst.rm, fpu_rqst.double} = {in.funct3, in.fdouble};
        end
        always_comb lsu = ~frompt & (
            op[`EX_LOAD] | op[`EX_STORE] | op[`EX_FENCE] | op[`EX_CSR]);
        always_comb begin
            lsu_rqst.id = lsu & ~op[`EX_FENCE] ? in.cqid : 0;
            lsu_rqst.fence = op[`EX_FENCE] ? in.b[11:0] : 0;
            lsu_rqst.wena = op[`EX_STORE] | op[`EX_CSR];
            /* maybe using LSQ id instead of CQ id as index is better */
            lsu_rqst.addr = pt ? res : {1'b0, add};
            lsu_rqst.bits = in.funct3;
            lsu_rqst.wdat = rval[1];
            lsu_rqst.rsrv = in.rsrv;
            lsu_rqst.aqrl = in.aqrl;
            lsu_rqst.csr = op[`EX_CSR];
            if (op[`EX_CSR]) lsu_rqst.addr = {1'b0, b};
            if (op[`EX_CSR]) lsu_rqst.wdat = in.a[64] ? rval[0] : in.a;
        end
        always_comb begin
            rqst_g[g].mul = mul; rqst_g[g].div = div;
            rqst_g[g].fpu = fpu; rqst_g[g].lsu = lsu;
            rqst_g[g].out = {`RQSTLEN{mul}} & `RQSTLEN'(mul_rqst) |
                            {`RQSTLEN{div}} & `RQSTLEN'(div_rqst) |
                            {`RQSTLEN{fpu}} & `RQSTLEN'(fpu_rqst) |
                            {`RQSTLEN{lsu}} & `RQSTLEN'(lsu_rqst);
        end
        always_comb begin
            res = {65{op[`EX_ADD]}}  & {1'b0, add} |
                  {65{op[`EX_SUB]}}  & {1'b0, sub[63:0]} |
                  {65{op[`EX_SLL]}}  & {1'b0, sll} |
                  {65{op[`EX_SRL]}}  & {1'b0, srl} |
                  {65{op[`EX_SRA]}}  & {1'b0, sra} |
                  {65{op[`EX_SLT]}}  & {64'd0, sub[63]} |
                  {65{op[`EX_SLTU]}} & {64'd0, sub[64]} |
                  {65{op[`EX_XOR]}} & {1'b0, a ^ b} |
                  {65{op[`EX_OR]}}  & {1'b0, a | b} |
                  {65{op[`EX_AND]}} & {1'b0, a & b} |
                  {65{op[`EX_MIN]}} & {1'b0, (in.isign ? sub[63] : sub[64]) ? a : b} |
                  {65{op[`EX_MAX]}} & {1'b0, (in.isign ? sub[63] : sub[64]) ? b : a};
            if (in.iword) res[63:0] = {{32{res[31]}}, res[31:0]};
            if (pt | mul | div | fpu | lsu & ~op[`EX_FENCE])
                res = {1'b1, {63-`lgCQSZ{1'd0}}, in.cqid};
        end
        always_ff @(posedge clk) if (rst | redir) pt_done[g] <= 0;
            else if (frompt & ~(op[`EX_LOAD] | op[`EX_STORE]))
                if (res[64]) pt_done[g] <= 0;
                else {pt_done[g], pt_data[g], pt_npc[g]} <=
                    {in.cqid, res, 1'b1, jump ? jpc : in.pc + {61'd0, in.delta}};
            else if (ena_arb[g]) pt_done[g] <= 0;
        always_ff @(posedge clk) if (rst | redir) addr_done[g] <= 0;
            else if (frompt & (op[`EX_LOAD] | op[`EX_STORE]))
                {addr_done[g], addr_val[g]} <= {in.cqid, add[63:0]};
            else addr_done[g] <= 0;
        always_comb if (op[`EX_ECALL] | op[`EX_EBREAK]) jpc = 0;
            else if (frompt & in_pt[g].j) jpc = in.a[63:0] + in.offset; // JALR
            else if (in.base[64]) jpc = rval[0][63:0] + in.offset;
            else jpc = in.base[63:0] + in.offset;
        always_comb jump = in.j | |in.bmask & in.bneg != |(in.bmask & bflag);
        always_comb begin
            out_wb_g[g].valid = in_id[g].valid;
            out_wb_g[g].rda = in.rda;
            out_wb_g[g].rd = res;
            out_wb_g[g].mem = op[`EX_LOAD] | op[`EX_STORE] | op[`EX_CSR];
            out_wb_g[g].patupd = (|in.bmask | in.j) &
                (in.pat[1:0] == 2'b01 | in.pat[1:0] == 2'b10);
            out_wb_g[g].b = |in.bmask;
            out_wb_g[g].c = ~in.delta[2];
            out_wb_g[g].fencei = op[`EX_FENCEI];
            out_wb_g[g].pc = in.pc;
            out_wb_g[g].pat = in.pat;
            out_wb_g[g].npc = jump ? jpc : in.pc + {61'd0, in.delta};
            if (op[`EX_ECALL]) out_wb_g[g].cause = {1'b1, 4'd2, level};
            else if (op[`EX_EBREAK]) out_wb_g[g].cause = {1'b1, 6'd3};
            else if (in.pf) out_wb_g[g].cause = {1'b1, 6'd12};
            else if (op[`EX_INV]) out_wb_g[g].cause = {1'b1, 6'd2};
            else out_wb_g[g].cause = 0;
            out_wb_g[g].ret = {op[`EX_RET], in.funct3[1:0]};
        end
    end
    always_ff @(posedge clk) if (rst | redir) cqidocc <= 0; else begin
        for (int i = 0; i < 4; i++) begin
            if (out_pt[i].valid)         cqidocc[out_pt[i].cqid[`lgCQSZ-1:0]] <= 1;
            if (mul_rqst[i].id[`lgCQSZ]) cqidocc[mul_rqst[i].id[`lgCQSZ-1:0]] <= 1;
            if (div_rqst[i].id[`lgCQSZ]) cqidocc[div_rqst[i].id[`lgCQSZ-1:0]] <= 1;
            if (fpu_rqst[i].id[`lgCQSZ]) cqidocc[fpu_rqst[i].id[`lgCQSZ-1:0]] <= 1;
            if (lsu_rqst[i].id[`lgCQSZ]) cqidocc[lsu_rqst[i].id[`lgCQSZ-1:0]] <= 1;
        end
        for (int i = 0; i < 4; i++)
            if (late_done[i][`lgCQSZ] & ~late_val[i][64]) begin
                assert(cqidocc[late_done[i][`lgCQSZ-1:0]]);
                cqidocc[late_done[i][`lgCQSZ-1:0]] <= 0;
        end
    end
endmodule

module wb_stage(input logic clk, input logic rst, output logic redir,
    input ex_wb_t [3:0] in_ex, output logic [3:0] get_ex,
    output xx_pc_t out_pc, input logic ena_pc,
    output logic [`lgCQSZ:0] frontid, output logic [`lgCQSZ:0] nextid,
    input logic [3:0][1:0][6:0] raddr, output logic [3:0][1:0][64:0] rvalue,
    input logic [3:0][`lgCQSZ:0] late_done, input logic [3:0][64:0] late_val,
    input logic [3:0][64:0] late_npc, input logic [3:0][6:0] late_cause,
    output logic [63:0] nret, output logic [2:0] ret,
    output logic [63:0] epc, output logic [63:0] tval, output logic [6:0] cause,
    input logic [63:0] tvec, input logic [63:0] mepc, input logic [63:0] sepc
);
    // register number: 00_xxxxx -> integer, 01_xxxxx -> float, 10_00000 -> tmp
    logic [64:0][`lgCQSZ:0] regscqid;
    logic [3:0][1:0][63:0] regsval;
    // commit queue
    logic [`lgCQSZ-1:0] cqfront; logic [`lgCQSZ:0] num, numin, numout;
    logic [3:0] cqpush; logic cqpop_accum, cqpatup_ena;
    logic [3:0] cqvalid, cqexcep, cqpatup, cqredir, cqpop;
    cqinfo_t [3:0] cqinfo; logic [3:0][63:0] cqnpc, cqfnpc;
    logic [3:0][6:0] cqcause, cqfcause; logic [3:0][64:0] cqdata, cqfdata;
    logic [3:0][1:0][`lgCQSZ-1:0] cqraddr; logic [3:0][1:0][64:0] cqrvalue;
    logic [4:0][`lgCQSZ-1:0] front; logic [3:0][`lgCQSZ-1:0] rear;
    logic lastvalid; cqinfo_t lastinfo, outinfo; logic [63:0] lastnpc, outnpc;
    regfile #(.dwidth(64), .rports(8), .wports(4), .awidth(7), .depth(65))
        regs_inst(.clk(clk), .rst(rst), .raddr(raddr), .rvalue(regsval),
            .waddr({cqinfo[3].rda, cqinfo[2].rda, cqinfo[1].rda, cqinfo[0].rda}),
            .wvalue({cqdata[3][63:0], cqdata[2][63:0],
                     cqdata[1][63:0], cqdata[0][63:0]}),
            .wena({cqpop[3] & |cqinfo[3].rda, cqpop[2] & |cqinfo[2].rda,
                   cqpop[1] & |cqinfo[1].rda, cqpop[0] & |cqinfo[0].rda}));
    logic [3:0][`lgCQSZ-1:0] cqwaddr;
    logic [3:0] cqinfowena, cqnpcwena, cqcausewena, cqvalwena;
    cqinfo_t [3:0] cqinfow; logic [3:0][63:0] cqnpcw;
    logic [3:0][6:0] cqcausew; logic [3:0][64:0] cqvalw;
    always_comb for (int i = 0; i < 4; i++) if (late_done[i][`lgCQSZ]) begin
        cqwaddr[i] = late_done[i][`lgCQSZ-1:0];
        cqinfowena[i] = 0; cqinfow[i] = 0;
        cqvalwena[i]  = 1; cqvalw[i]  = late_val[i];
        cqnpcwena[i]   = late_npc[i][64];  cqnpcw[i]   = late_npc[i][63:0];
        cqcausewena[i] = late_cause[i][6]; cqcausew[i] = late_cause[i];
    end else begin
        cqwaddr[i] = rear[i];
        cqinfowena[i] = cqpush[i]; cqvalwena[i] = cqpush[i];
        cqnpcwena[i] = cqpush[i]; cqcausewena[i] = cqpush[i];
        cqinfow[i] = in_ex[i][`CQLEN-1:0]; cqvalw[i] = in_ex[i].rd;
        cqnpcw[i] = in_ex[i].npc; cqcausew[i] = in_ex[i].cause;
    end
    regfile #(.dwidth(`CQLEN), .rports(4), .wports(4), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqinfo_inst(.clk(clk), .rst(rst), .raddr(front[3:0]), .rvalue(cqinfo),
            .waddr(cqwaddr), .wvalue(cqinfow), .wena(cqinfowena));
    regfile #(.dwidth(64), .rports(4), .wports(4), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqnpc_inst(.clk(clk), .rst(rst), .raddr(front[3:0]), .rvalue(cqfnpc),
            .waddr(cqwaddr), .wvalue(cqnpcw), .wena(cqnpcwena));
    regfile #(.dwidth(7), .rports(4), .wports(4), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqcause_inst(.clk(clk), .rst(rst), .raddr(front[3:0]), .rvalue(cqfcause),
            .waddr(cqwaddr), .wvalue(cqcausew), .wena(cqcausewena));
    regfile #(.dwidth(65), .rports(12), .wports(4), .awidth(`lgCQSZ), .depth(`CQSZ))
        cqval_inst(.clk(clk), .rst(rst),
            .raddr({front[3:0], cqraddr}), .rvalue({cqfdata, cqrvalue}),
            .waddr(cqwaddr), .wena(cqvalwena), .wvalue(cqvalw));
    always_comb for (int i = 0; i < 4; i++) begin
        cqdata[i] = cqfdata[i]; cqnpc[i] = cqfnpc[i]; cqcause[i] = cqfcause[i];
        for (int j = 0; j < 4; j++)
            if (late_done[j] == {1'b1, front[i]}) begin
                cqdata[i] = late_val[j];
                if (late_npc[j][64]) cqnpc[i] = late_npc[j][63:0];
                if (late_cause[j][6]) cqcause[i] = late_cause[j];
            end
        if (cqinfo[i].ret[2]) cqnpc[i] = cqinfo[i].ret[1:0] == 2'b01 ? sepc : mepc;
    end
    always_comb for (int i = 0; i < 4; i++) get_ex[i] = ~in_ex[i].valid | cqpush[i];
    always_comb for (int i = 0; i < 5; i++) front[i] = cqfront + i[`lgCQSZ-1:0];
    always_comb for (int i = 0; i < 4; i++) rear[i] = front[i] + num[`lgCQSZ-1:0];
    always_comb for (int i = 0; i < 4; i++) cqvalid[i] = i[`lgCQSZ:0] < num;
    always_comb cqexcep[0] = |num & cqcause[0][6] & cqinfo[0].pc == lastnpc;
    always_comb cqredir[0] = |num &
        cqinfo[0].pc != lastnpc | lastinfo.fencei | cqexcep[0];
    always_comb cqpatup[0] = cqredir[0] | lastvalid & lastinfo.patupd;
    always_comb for (int i = 1; i < 4; i++) cqexcep[i] = cqcause[i][6] & cqvalid[i] &
        cqinfo[i].pc == cqnpc[i - 1];
    always_comb for (int i = 1; i < 4; i++) cqredir[i] = cqexcep[i] | cqvalid[i] &
        cqinfo[i].pc != cqnpc[i - 1] & ~cqdata[i - 1][64] | cqinfo[i - 1].fencei;
    always_comb for (int i = 1; i < 4; i++)
        cqpatup[i] = cqredir[i] | cqvalid[i - 1] & cqinfo[i - 1].patupd;
    always_comb begin cqpatup_ena = ena_pc; cqpop_accum = 1; 
        for (int i = 0; i < 4; i++) begin
            cqpop[i] = cqpop_accum & cqvalid[i] & ~cqdata[i][64] & ~cqredir[i];
            if (cqpatup[i]) cqpop[i] &= cqpatup_ena;
            cqpop_accum = cqpop[i];
            cqpatup_ena &= ~cqpatup[i];
        end end
    always_comb for (int i = 0; i < 4; i++) cqpush[i] =
        in_ex[i].valid & num - numout + i[`lgCQSZ:0] < `CQSZ & ~late_done[i][`lgCQSZ];
    always_comb begin numin = 0; for (int n = 1; n <= 4; n++)
        if (cqpush[n - 1]) numin = n[`lgCQSZ:0]; end
    always_comb begin numout = 0; for (int n = 1; n <= 4; n++)
        if (cqpop[n - 1]) numout = n[`lgCQSZ:0]; end
    always_comb redir = out_pc.redir;
    always_comb begin
        cause = 0; epc = 0; tval = 0;
        for (int i = 3; i >= 0; i--) if (cqredir[i])
            if (cqexcep[i]) begin
                cause = cqcause[i];
                epc = cqinfo[i].pc;
                if (cause[5:0] == 6'd12) tval = cqinfo[i].pc; // I page fault
                else if (cause[5:0] == 6'd13 | cause[5:0] == 6'd15) // L/S page fault
                    tval = cqdata[i][63:0];
            end else cause = 0;
    end
    always_comb begin ret = 0; for (int i = 3; i >= 0; i--)
        if (cqpop[i] & cqinfo[i].ret[2]) ret = cqinfo[i].ret; end
    always_comb begin nret = 0; for (int i = 0; i < 4; i++)
        if (cqpop[i] & ~cqinfo[i].rda[6]) nret++; end
    always_comb frontid = cqinfo[0].pc == lastnpc ? {1'b1, front[0]} : 0;
    always_comb begin nextid = 0; for (int i = 3; i >= 0; i--)
        if (~cqpop[i] & cqvalid[i] & ~cqredir[i]) nextid = {1'b1, front[i]}; end
    always_ff @(posedge clk) if (redir) lastvalid <= 0;
        else for (int i = 0; i < 4; i++)
            if (cqpop[i]) {lastvalid, lastinfo} <= {1'b1, cqinfo[i]};
    always_ff @(posedge clk) if (rst) lastnpc <= `RST_PC;
        else if (~cqpop[0] & cqexcep[0]) lastnpc <= tvec;
        else for (int i = 0; i < 4; i++) if (cqpop[i])
            lastnpc <= i < 3 & cqexcep[i + 1] ? tvec : cqnpc[i];
    always_ff @(posedge clk) if (rst | redir) {cqfront, num} <= 0;
        else {cqfront, num} <= {front[numout], num + numin - numout};
    always_ff @(posedge clk)
        if (rst | redir) for (int i = 0; i < 65; i++) regscqid[i][`lgCQSZ] <= 0;
        else begin // commit
            for (int i = 0; i < 4; i++)
                if (cqpop[i] & regscqid[cqinfo[i].rda][`lgCQSZ-1:0] == front[i])
                    regscqid[cqinfo[i].rda] <= 0;
            for (int i = 0; i < 4; i++)
                if (cqpush[i] & |in_ex[i].rda)
                    regscqid[in_ex[i].rda] <= {1'b1, rear[i]};
        end
    always_comb for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++)
            cqraddr[i][j] = regscqid[raddr[i][j]][`lgCQSZ-1:0];
    always_comb for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) begin
            rvalue[i][j] = {1'b0, regsval[i][j]};
            if (regscqid[raddr[i][j]][`lgCQSZ]) rvalue[i][j] = cqrvalue[i][j];
            for (int k = 0; k < 4; k++)
                if (in_ex[k].valid & raddr[i][j] == in_ex[k].rda)
                    rvalue[i][j] = in_ex[k].rd;
            for (int k = 0; k < 4; k++)
                if (rvalue[i][j][64] & late_done[k] == rvalue[i][j][`lgCQSZ:0])
                    rvalue[i][j] = late_val[k];
            if (raddr[i][j] == 0) rvalue[i][j] = 0;
        end
    always_comb begin {out_pc, outinfo, outnpc} = 0;
        for (int i = 3; i >= 0; i--) if (cqpop[i] & cqpatup[i] | cqredir[i]) begin
            outinfo = i == 0 ? lastinfo : cqinfo[i - 1];
            outnpc = i == 0 ? lastnpc : cqnpc[i - 1];
            out_pc.reinf = cqpop[i] & cqpatup[i];
            out_pc.redir = cqredir[i];
            for (int j = 0; j < i; j++) if (~cqpop[j]) out_pc.redir = 0;
            out_pc.pc = outinfo.pc;
            out_pc.npc = cqexcep[i] ? tvec : outnpc;
            out_pc.gh = outinfo.pat[17:2];
            out_pc.pat = outinfo.pat[1:0];
            out_pc.c = outinfo.c;
            if (cqredir[i] & outinfo.b) out_pc.gh[0] ^= 1;
        end end
endmodule

module pending_table(input logic clk, input logic rst, input logic flush,
    input id_ex_t [3:0] in_ex, output logic [3:0] get_ex,
    output id_ex_t [3:0] out_ex, input logic [3:0] ena_ex,
    input logic [3:0][`lgCQSZ:0] late_done, input logic [3:0][64:0] late_val
);
    logic [`PTSZ-1:0] valid, valid_fwd;
    logic [`PTSZ-1:0][64:0] a, b, a_fwd, b_fwd;
    logic [3:0] in_ena, out_ena; logic [3:0][`lgPTSZ-1:0] in_idx, out_idx;
    id_ex_t [3:0] in_dat, out_dat; logic [2:0] i0, i1, i2;
    regfile #(.dwidth(`PTLEN), .rports(4), .wports(4), .awidth(`lgPTSZ), .depth(`PTSZ))
        data_inst(.clk(clk), .rst(rst), .raddr(out_idx), .rvalue(out_dat),
            .waddr(in_idx), .wvalue(in_dat), .wena(in_ena));
    always_comb begin out_ena = 0; out_idx = 0; i0 = 0; valid_fwd = valid;
        for (int i = 0; i < `PTSZ; i++) if (valid[i] & ~a_fwd[i][64] & ~b_fwd[i][64])
            if (i0 < 4 & (ena_ex[i0[1:0]] | ~out_ex[i0[1:0]].valid)) begin
                out_ena[i0[1:0]] = 1; out_idx[i0[1:0]] = i[`lgPTSZ-1:0];
                valid_fwd[i] = 0; i0++;
            end end
    always_comb begin in_ena = 0; in_idx = 0; i1 = 0;
        for (int i = 0; i < `PTSZ; i++) if (~valid[i])
            if (i1 < 4 & in_ex[i1[1:0]].valid) begin
                in_ena[i1[1:0]] = 1; in_idx[i1[1:0]] = i[`lgPTSZ-1:0]; i1++; end end
    always_comb begin get_ex = 0; i2 = 0;
        for (int i = 0; i < `PTSZ; i++) if (~valid[i])
            if (i2 < 4) begin get_ex[i2[1:0]] = 1; i2++; end end
    always_comb in_dat = in_ex;
    always_comb for (int i = 0; i < `PTSZ; i++) begin a_fwd[i] = a[i]; b_fwd[i] = b[i];
        for (int j = 0; j < 4; j++) begin
            if (a_fwd[i][64] & late_done[j] == a_fwd[i][`lgCQSZ:0])
                a_fwd[i] = late_val[j];
            if (b_fwd[i][64] & late_done[j] == b_fwd[i][`lgCQSZ:0])
                b_fwd[i] = late_val[j];
        end end
    always_ff @(posedge clk) for (int i = 0; i < 4; i++)
        if (rst | flush) out_ex[i].valid <= 0;
        else if (out_ena[3 - i]) begin
            out_ex[i] <= out_dat[3 - i];
            out_ex[i].a <= a_fwd[out_idx[3 - i]];
            out_ex[i].b <= b_fwd[out_idx[3 - i]];
        end else if (ena_ex[i]) out_ex[i].valid <= 0;
    always_ff @(posedge clk)
        if (rst | flush) for (int i = 0; i < `PTSZ; i++) valid[i] <= 0;
        else begin
            for (int i = 0; i < 4; i++) if (out_ena[i]) valid[out_idx[i]] <= 0;
            for (int i = 0; i < 4; i++) if (in_ena[i]) valid[in_idx[i]] <= 1;
        end
    always_ff @(posedge clk) begin
        a <= a_fwd; b <= b_fwd;
        for (int i = 0; i < 4; i++) if (in_ena[i]) begin
            a[in_idx[i]] <= in_ex[i].a;
            b[in_idx[i]] <= in_ex[i].b;
            for (int j = 0; j < 4; j++)
                if (in_ex[i].a[64] & late_done[j] == in_ex[i].a[`lgCQSZ:0])
                    a[in_idx[i]] <= late_val[j];
            for (int j = 0; j < 4; j++)
                if (in_ex[i].a[64] & late_done[j] == in_ex[i].a[`lgCQSZ:0])
                    a[in_idx[i]] <= late_val[j];
        end
    end
endmodule

module lsu(input logic clk, input logic rst, input logic flush, input logic ena,
    input logic [`lgCQSZ:0] frontid, input logic [`lgCQSZ:0] nextid,
    output logic [3:0] get, input lsu_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [6:0] cause, output logic [64:0] rdata,
    input logic [3:0][`lgCQSZ:0] late_done, input logic [3:0][64:0] late_val,
    input logic [3:0][`lgCQSZ:0] addr_done, input logic [3:0][63:0] addr_val,
    output logic [11:0] csr_addr, output logic csr_wena, output logic [2:0] csr_func,
    input logic [63:0] csr_rval, output logic [63:0] csr_wval, input logic csr_excp,
    output logic [`lgCQSZ:0] dcache_rqst,
    output logic       [1:0] dcache_rsrv,
    output logic             dcache_wena,
    output logic      [63:0] dcache_addr,
    output logic       [2:0] dcache_bits,
    input  logic [`lgCQSZ:0] dcache_done,
    input  logic       [1:0] dcache_pgft,
    input  logic      [63:0] dcache_rdat,
    output logic      [63:0] dcache_wdat,
    output logic             dcache_flsh
);
    lsu_rqst_t [`LSQSZ-1:0] lsqrqst; lsu_rqst_t [`LSQSZ+3:0] lsq; lsu_rqst_t rqstf;
    logic [`LSQSZ-1:0] lsqsent, lsqmisa; logic [`LSQSZ+3:0] lsqmi;
    logic [`lgLSQSZ-1:0] lsqfront; logic [`lgLSQSZ:0] num, numout, numin;
    logic [`LSQSZ-1:0][`lgLSQSZ-1:0] front; logic [`lgLSQSZ-1:0] rear;
    logic [3:0] through, fwd, misa; logic [3:0][63:0] fwddata;
    logic fromlsq, fromth, fromdc, fromfwd, fromcsr, cqtop, cqnext;
    always_comb begin
        rqstf = lsqrqst[front[0]];
        for (int i = 0; i < `LSQSZ; i++) lsq[i] = lsqrqst[front[i]];
        for (int i = 0; i < `LSQSZ; i++) lsqmi[i] = lsqmisa[front[i]];
        for (int i = 0; i < 4; i++) lsq[`LSQSZ + i] = rqst[i];
        for (int i = 0; i < 4; i++) lsqmi[`LSQSZ + i] = misa[i];
    end
    always_comb for (int i = 0; i < 4; i++) case (rqst[i].bits[1:0])
        0: misa[i] = 0;                  1: misa[i] = rqst[i].addr[0];
        2: misa[i] = |rqst[i].addr[1:0]; 3: misa[i] = |rqst[i].addr[2:0];
    endcase
    always_comb for (int i = 0; i < 4; i++) if (rqst[i].id[`lgCQSZ]) begin
        through[i] = ~rqst[i].wena & ~rqst[i].addr[64] & ~misa[i];
        for (int j = 0; j < `LSQSZ + i; j++) if (lsq[j].id[`lgCQSZ])
            if (lsq[j].addr[64] | lsqmi[j] | lsq[j].aqrl[1] |
                rqst[i].addr[63:3] == lsq[j].addr[63:3])
                through[i] = 0;
        if (rqst[i].aqrl[0] | rqst[i].csr) through[i] = 0;
        fwd[i] = 0; fwddata[i] = 0;
        for (int j = 0; j < `LSQSZ + i; j++)
            if (lsq[j].id[`lgCQSZ] & lsq[j].wena)
                if (lsq[j].addr[64] | lsqmi[j]) fwd[i] = 0;
                else if (lsq[j].addr      == rqst[i].addr &
                         lsq[j].bits[1:0] == rqst[i].bits[1:0]) begin
                    fwd[i] = ~lsq[j].wdat[64] & ~lsq[j].rsrv[0];
                    fwddata[i] = lsq[j].wdat[63:0];
                    case (rqst[i].bits[1:0])
                        0: fwddata[i] =
                            {{56{fwddata[i][7] & ~rqst[i].bits[2]}}, fwddata[i][7:0]};
                        1: fwddata[i] =
                            {{48{fwddata[i][15] & ~rqst[i].bits[2]}}, fwddata[i][15:0]};
                        2: fwddata[i] =
                            {{32{fwddata[i][31] & ~rqst[i].bits[2]}}, fwddata[i][31:0]};
                        3: fwddata[i] = fwddata[i];
                    endcase
                end else if (lsq[j].addr[63:3] == rqst[i].addr[63:3] &
                             lsq[j].bits[1:0]  != rqst[i].bits[1:0])
                    fwd[i] = 0;
        for (int j = 0; j < `LSQSZ; j++)
            if (lsq[j].id[`lgCQSZ] & lsq[j].aqrl[1]) fwd[i] = 0;
        if (rqst[i].wena | |rqst[i].rsrv | misa[i]) fwd[i] = 0;
        if (rqst[i].aqrl[0] | rqst[i].csr) fwd[i] = 0;
    end else {through[i], fwd[i], fwddata[i]} = 0;
    always_comb numout = (~rqstf.id[`lgCQSZ] | rqstf.id == done |
                           fromlsq & ~rqstf.aqrl[1]) & |num ? 1 : 0;
    always_comb begin get = 0; for (int i = 0; i < 4; i++)
        if (num - numout + i[`lgLSQSZ:0] < `LSQSZ) get[i] = 1; end
    always_comb begin numin = 0; for (int i = 1; i <= 4; i++)
        if (rqst[i - 1].id[`lgCQSZ] & get[i - 1]) numin = i[`lgLSQSZ:0]; end
    always_comb for (int i = 0; i < `LSQSZ; i++) front[i] = lsqfront + i[`lgLSQSZ-1:0];
    always_comb rear = front[num[`lgLSQSZ-1:0]];
    always_ff @(posedge clk) if (rst | flush) {lsqfront, num} <= 0;
        else {lsqfront, num} <= {front[numout], num - numout + numin};
    always_ff @(posedge clk) for (int i = 0; i < `LSQSZ; i++)
        if (rst | flush) lsqrqst[i].id <= 0;
        else if ({1'b0, i[`lgLSQSZ-1:0] - rear} < numin) begin
            lsqrqst[i] <= rqst[2'(i - 32'(rear))];
            lsqmisa[i] <= misa[2'(i - 32'(rear))];
            lsqsent[i] <= through[2'(i - 32'(rear))] | fwd[2'(i - 32'(rear))];
            for (int j = 0; j < 4; j++)
                if (rqst[2'(i - 32'(rear))].addr[64] &
                    rqst[2'(i - 32'(rear))].addr[`lgCQSZ:0] == addr_done[j])
                    lsqrqst[i].addr <= 65'(addr_val[j]);
            for (int j = 0; j < 4; j++)
                if (rqst[2'(i - 32'(rear))].wdat[64] &
                    rqst[2'(i - 32'(rear))].wdat[`lgCQSZ:0] == late_done[j])
                    lsqrqst[i].wdat <= late_val[j];
        end else begin
            for (int j = 0; j < 4; j++)
                if (lsqrqst[i].addr[64] &lsqrqst[i].addr[`lgCQSZ:0] == addr_done[j])
                    lsqrqst[i].addr <= 65'(addr_val[j]);
            for (int j = 0; j < 4; j++)
                if (lsqrqst[i].wdat[64] & lsqrqst[i].wdat[`lgCQSZ:0] == late_done[j])
                    lsqrqst[i].wdat <= late_val[j];
            if ( lsqrqst[i].wena &  rqst[0].fence[1] & rqst[0].fence[4] |
                ~lsqrqst[i].wena & (rqst[0].fence[1] & rqst[0].fence[5] |
                                    rqst[0].fence[11]))
                lsqrqst[i].aqrl[1] <= 1;
            if (i == 32'(front[0]) & fromlsq) lsqsent[i] <= 1;
            if (i == 32'(front[0]) & |numout) lsqrqst[i].id <= 0;
            if (lsqrqst[i].id == done) lsqrqst[i].id <= 0;
        end
    logic [`lgLSQSZ-1:0] thfront, fwdfront; logic [3:0][`lgLSQSZ-1:0] threar, fwdrear;
    logic [`lgLSQSZ:0] thnum, fwdnum;
    lsu_rqst_t thrqst, dcrqst; logic [`lgCQSZ:0] fwdid; logic [63:0] fwdval;
    lsu_rqst_t [3:0] thw; logic [3:0][`lgCQSZ+64:0] fwdw;
    logic [3:0] thwena, fwdwena; logic [2:0] thiter, fwditer;
    always_comb begin thw = 0; thwena = 0; thiter = 0;
        for (int i = 0; i < 4; i++) if (i[`lgLSQSZ:0] < numin & through[i]) begin
            thw[thiter[1:0]] = rqst[i]; thwena[thiter[1:0]] = 1; thiter++; end end
    always_comb begin fwdw = 0; fwdwena = 0; fwditer = 0;
        for (int i = 0; i < 4; i++) if (i[`lgLSQSZ:0] < numin & fwd[i]) begin
            fwdw[fwditer[1:0]] = {rqst[i].id, fwddata[i]};
            fwdwena[fwditer[1:0]] = 1; fwditer++;
        end end
    always_comb for (int i = 0; i < 4; i++)
        threar[i] = thfront + `lgLSQSZ'(thnum) + `lgLSQSZ'(i);
    always_comb for (int i = 0; i < 4; i++)
        fwdrear[i] = fwdfront + `lgLSQSZ'(fwdnum) + `lgLSQSZ'(i);
    always_ff @(posedge clk) if (rst | flush) thfront <= 0;
        else if (fromth) thfront <= thfront + `lgLSQSZ'(1);
    always_ff @(posedge clk) if (rst | flush) fwdfront <= 0;
        else if (fromfwd) fwdfront <= fwdfront + `lgLSQSZ'(1);
    always_ff @(posedge clk) if (rst | flush) thnum <= 0;
        else thnum <= thnum + (`lgLSQSZ+1)'(thiter) - (`lgLSQSZ+1)'(fromth);
    always_ff @(posedge clk) if (rst | flush) fwdnum <= 0;
        else fwdnum <= fwdnum + (`lgLSQSZ+1)'(fwditer) - (`lgLSQSZ+1)'(fromfwd);
    regfile #(.dwidth(`LSULEN), .rports(1), .wports(4),
              .awidth(`lgLSQSZ), .depth(`LSQSZ))
        thbuf_inst(.clk(clk), .rst(rst), .raddr(thfront), .rvalue(thrqst),
            .waddr(threar), .wvalue(thw), .wena(thwena));
    regfile #(.dwidth(`lgCQSZ+65), .rports(1), .wports(4),
              .awidth(`lgLSQSZ), .depth(`LSQSZ))
        fwdbuf_inst(.clk(clk), .rst(rst), .raddr(fwdfront), .rvalue({fwdid, fwdval}),
            .waddr(fwdrear), .wvalue(fwdw), .wena(fwdwena));
    always_comb cqtop = rqstf.id == frontid;
    always_comb cqnext = rqstf.id == nextid;
    always_comb fromth = |thnum;
    always_comb if (~fromth & rqstf.id[`lgCQSZ] & ~lsqsent[front[0]] &
                    ~rqstf.addr[64] & ~rqstf.csr)
        if (~rqstf.wena) fromlsq = ~rqstf.aqrl[0] | cqnext;
        else fromlsq = ~rqstf.wdat[64] & cqnext;
    else fromlsq = 0;
    always_comb fromdc = dcache_done[`lgCQSZ];
    always_comb fromfwd = ~fromdc & |fwdnum;
    always_comb fromcsr = ~fromdc & ~fromfwd & rqstf.id[`lgCQSZ] & rqstf.csr & cqtop;
    always_comb dcrqst = fromth ? thrqst : (fromlsq ? rqstf : 0);
    always_comb dcache_rqst = dcrqst.csr ? 0 : dcrqst.id;
    always_comb dcache_rsrv = dcrqst.rsrv;
    always_comb dcache_wena = dcrqst.wena;
    always_comb dcache_bits = dcrqst.bits;
    always_comb dcache_addr = dcrqst.addr[63:0];
    always_comb dcache_wdat = dcrqst.wdat[63:0];
    always_comb dcache_flsh = flush;
    always_comb csr_addr = lsqrqst[front[0]].addr[11:0];
    always_comb csr_wena = fromcsr;
    always_comb csr_func = lsqrqst[front[0]].bits;
    always_comb csr_wval = lsqrqst[front[0]].wdat[63:0];
    always_comb if (fromdc)  {done, rdata} = {dcache_done, 1'b0, dcache_rdat};
        else    if (fromfwd) {done, rdata} = {fwdid,       1'b0, fwdval};
        else    if (fromcsr) {done, rdata} = {lsqrqst[front[0]].id, 1'b0, csr_rval};
        else                 {done, rdata} = 0;
    always_comb if (csr_excp) cause = {1'b1, 6'd2};
        else if (fromdc & dcache_pgft[1]) cause = {1'b1, 6'd15};
        else if (fromdc & dcache_pgft[0]) cause = {1'b1, 6'd13};
        else cause = 0;
endmodule

module arbiter(
    input logic [`LATENUM-1:0][`lgCQSZ:0] done_in,
    input logic [`LATENUM-1:0][64:0] val_in,
    input logic [`LATENUM-1:0][64:0] npc_in,
    input logic [`LATENUM-1:0][6:0] cause_in,
    output logic [`LATENUM-1:0] get,
    output logic [3:0][`lgCQSZ:0] done_out, output logic [3:0][64:0] val_out,
    output logic [3:0][64:0] npc_out, output logic [3:0][6:0] cause_out
);
    logic [1:0] iter;
    always_comb begin
        get = 0; iter = 3;
        done_out = 0; val_out = 0; npc_out = 0; cause_out = 0;
        for (int i = 0; i < `LATENUM; i++) if (done_in[i][`lgCQSZ]) begin
            done_out[iter] = done_in[i]; val_out[iter] = val_in[i];
            npc_out[iter] = npc_in[i]; cause_out[iter] = cause_in[i];
            get[i] = 1; iter--; if (iter == 3) break;
        end
        for (int i = 0; i < `LATENUM; i++) if (~done_in[i][`lgCQSZ]) get[i] = 1;
    end
endmodule

module csr(input logic clk, input logic rst,
    input logic [11:0] addr, output logic [63:0] rval,
    input logic wena, input logic [63:0] wval, input logic [2:0] func,
    input logic [63:0] nret, output logic eout,
    input logic ein, input logic [63:0] epc, logic [63:0] tval,
    input logic [5:0] cause, input logic [2:0] ret,
    output logic [63:0] csr_tvec, output logic [63:0] csr_mepc,
    output logic [63:0] csr_sepc, output logic [63:0] csr_satp
);
    logic [1:0] level; // 00 -> U  01 -> S  11 -> M
    logic [63:0] wres; logic trapintos;
    logic [63:0] misa, mvendorid, marchid, mimpid, mhartid;
    logic [63:0] mstatus, mtvec, medeleg, mideleg, mip, mie;
    logic [63:0] mtime, mtimecmp; // memory mapped
    logic [63:0] mcycle, minstret, mhpmcounter[31:0], mhpmevent[31:0];
    logic [63:0] mcounteren, mcountinhibit, mscratch, mepc, mcause, mtval;
    logic [63:0] sstatus, stvec, sip, sie, scounteren, sscratch;
    logic [63:0] satp, sepc, scause, stval;
    logic [63:0] utvec;
    always_comb trapintos = (level == 2'b00 | level == 2'b01) & medeleg[cause];
    always_comb case (func[1:0])
        2'b00: wres = 0;
        2'b01: wres = wval;
        2'b10: wres = wval | rval;
        2'b11: wres = ~wval & rval;
    endcase
    always_comb if (addr > 12'hb02 & addr < 12'hb20)
        rval = mhpmcounter[addr[4:0]];
    else if (addr > 12'h322 & addr < 12'h340)
        rval = mhpmevent[addr[4:0]];
    else case (addr)
        12'h301: rval = misa;          12'hf11: rval = mvendorid;
        12'hf12: rval = marchid;       12'hf13: rval = mimpid;
        12'hf14: rval = mhartid;       12'h300: rval = mstatus;
        12'h305: rval = mtvec;         12'h302: rval = medeleg;
        12'h303: rval = mideleg;       12'h344: rval = mip;
        12'h304: rval = mie;           12'hb00: rval = mcycle;
        12'hb02: rval = minstret;      12'h306: rval = mcounteren;
        12'h320: rval = mcountinhibit; 12'h340: rval = mscratch;
        12'h341: rval = mepc;          12'h342: rval = mcause;
        12'h343: rval = mtval;         12'h180: rval = satp;
        12'h100: rval = sstatus;       12'h105: rval = stvec;
        12'h144: rval = sip;
        12'h104: rval = sie;           12'h106: rval = scounteren;
        12'h140: rval = sscratch;      12'h141: rval = sepc;
        12'h142: rval = scause;        12'h143: rval = stval;
        default: rval = 0;
        12'h005: rval = utvec;
    endcase
    always_ff @(posedge clk) begin
        // switch priority mode
        if (ein)
            if (trapintos) begin
                level <= 2'b01; // trap into S mode
                sepc <= epc;
                stval <= tval;
                scause <= {58'd0, cause};
            end else begin
                level <= 2'b11; // trap into M mode
                mepc <= epc;
                mtval <= tval;
                mcause <= {58'd0, cause};
            end
        if (ret[2])
            if (ret[1:0] == 2'b11) begin    // MRET
                level <= mstatus[12:11];    // level -> MPP
                mstatus[12:11] <= 0;        // MPP   -> U
                mstatus[3] <= mstatus[7];   // MIE   -> MPIE
                mstatus[7] <= 1;            // MPIE  -> 1
            end else if (ret[1:0] == 2'b01) begin   // SRET
                level <= {1'b0, mstatus[8]};        // level -> SPP
                mstatus[8] <= 0;                    // SPP   -> U
                mstatus[1] <= mstatus[5];           // SIE   -> SPIE
                mstatus[5] <= 1;                    // SPIE  -> 1
                sstatus[8] <= 0;                    // SPP   -> U
                sstatus[1] <= sstatus[5];           // SIE   -> SPIE
                sstatus[5] <= 1;                    // SPIE  -> 1
            end else if (ret[1:0] == 2'b00) begin       // URET
                level <= 0;                             // level -> UPP
                mstatus[0] <= mstatus[4];               // UIE   -> UPIE
                mstatus[4] <= 1;                        // UPIE  -> 1
                sstatus[0] <= sstatus[4];               // UIE   -> UPIE
                sstatus[4] <= 1;                        // UPIE  -> 1
            end
        if (rst) level <= 2'b11;
        // M-level CSR
                                        // ZY XWVU TSRQ PONM LKJI HGFE DCBA
                                        //          S      M    I   F  DC A
        if (rst) misa <= {2'h2, 36'h0, 26'b00_0000_0100_0001_0001_0010_1101};
        else if (wena & addr == 12'h301) begin
            misa[0] <= wres[0]; misa[3:2] <= wres[3:2]; misa[4] <= ~wres[8];
            misa[8:5] <= wres[8:5]; misa[13:12] <= wres[13:12]; misa[16] <= wres[16];
            misa[18] <= wres[18]; misa[20] <= wres[20]; misa[23] <= wres[23];
            if (wres[5]) {misa[3], misa[16]} <= 0;
        end
        if (rst) mvendorid <= 0; else if (wena & addr == 12'hf11) eout <= 1;
        if (rst) marchid <= 0; else if (wena & addr == 12'hf12) eout <= 1;
        if (rst) mimpid <= 0; else if (wena & addr == 12'hf13) eout <= 1;
        if (rst) mhartid <= 0; else if (wena & addr == 12'hf13) eout <= 1;
        if (rst) mstatus <= {32'ha, 19'h1, 13'h0};
        else if (wena & addr == 12'h300) begin
            mstatus <= wres;
            mstatus[63] <= mstatus[16:15] == 2'b11 | mstatus[14:13] == 2'b11;
            {mstatus[62:36], mstatus[31:23]} <= 0;
            {mstatus[10:9], mstatus[6], mstatus[2]} <= 0;
        end
        if (rst) mtvec <= 0; else if (wena & addr == 12'h305) begin
            mtvec <= wres; mtvec[1] <= 0; end
        if (rst) medeleg <= 0; else if (wena & addr == 12'h302) begin
            medeleg <= wres; medeleg[11] <= 0; end
        if (rst) mideleg <= 0; else if (wena & addr == 12'h303) mideleg <= wres;
        if (rst) mip <= 0; else if (wena & addr == 12'h344) begin
            mip <= wres; mip[63:12] <= 0; mip[10] <= 0; mip[6] <= 0; mip[2] <= 0; end
        if (rst) mie <= 0; else if (wena & addr == 12'h304) begin
            mie <= wres; mie[63:12] <= 0; mie[10] <= 0; mie[6] <= 0; mie[2] <= 0; end
        if (rst) mcycle <= 0; else if (wena & addr == 12'hb00) mcycle <= wres;
        else mcycle <= mcycle + 64'd1;
        if (rst) minstret <= 0; else minstret <=
            (wena & addr == 12'hb02 ? wres : minstret) + (mcountinhibit[2] ? 0 : nret);
        for (int i = 3; i < 32; i++) if (rst) mhpmcounter[i] <= 0;
            else if (wena & addr[11:5] == 7'h58) mhpmcounter[i] <= wres;
        for (int i = 3; i < 32; i++) if (rst) mhpmevent[i] <= 0;
            else if (wena & addr[11:5] == 7'h58) mhpmevent[i] <= wres;
        if (rst) mcounteren <= 0; else if (wena & addr == 12'h306) mcounteren <= wres;
        if (rst) mcountinhibit <= 0;
        else if (wena & addr == 12'h320) mcountinhibit <= wres;
        if (rst) mscratch <= 0; else if (wena & addr == 12'h340) mscratch <= wres;
        if (rst) mepc <= 0; else if (wena & addr == 12'h341) mepc <= wres;
        if (rst) mcause <= 0; else if (wena & addr == 12'h342) mcause <= wres;
        if (rst) mtval <= 0; else if (wena & addr == 12'h343) mtval <= wres;
        // S-level CSR
        if (rst) sstatus <= {32'h2, 19'h1, 13'h0};
        else if (wena & addr == 12'h100) begin
            sstatus <= wres;
            sstatus[63] <= sstatus[16:15] == 2'b11 | sstatus[14:13] == 2'b11;
            {sstatus[62:34], sstatus[31:20]} <= 0;
            {sstatus[12:9], sstatus[7:6], sstatus[3:2]} <= 0;
        end
        if (rst) stvec <= 0; else if (wena & addr == 12'h105) begin
            stvec <= wres; stvec[1] <= 0; end
        if (rst) sip <= 0; else if (wena & addr == 12'h144) begin
            sip <= wres; sip[63:10] <= 0; sip[7:6] <= 0; sip[3:2] <= 0; end
        if (rst) sie <= 0; else if (wena & addr == 12'h104) begin
            sie <= wres; sie[63:10] <= 0; sie[7:6] <= 0; sie[3:2] <= 0; end
        if (rst) scounteren <= 0; else if (wena & addr == 12'h106) scounteren <= wres;
        if (rst) sscratch <= 0; else if (wena & addr == 12'h140) sscratch <= wres;
        if (rst) sepc <= 0; else if (wena & addr == 12'h141) sepc <= wres;
        if (rst) scause <= 0; else if (wena & addr == 12'h142) scause <= wres;
        if (rst) stval <= 0; else if (wena & addr == 12'h143) stval <= wres;
        if (rst) satp <= 0; else if (wena & addr == 12'h180) satp <= wres;
        if (rst) eout <= 0;

        if (wena & addr == 12'h005) utvec <= wres;
    end
    always_comb csr_tvec = trapintos ? stvec : mtvec;
    always_comb csr_mepc = mepc;
    always_comb csr_sepc = sepc;
    always_comb csr_satp = level == 2'b11 ? 64'd0 : satp;
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
                    i = {{{15{ci[12]}}, ci[6:2]}, ci[11:7], 7'h37};
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
