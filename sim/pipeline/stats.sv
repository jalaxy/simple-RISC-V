module stats(
    input  logic         clk,
    input  logic         rst,
    input  logic   [7:0] eiptip,
    input  logic  [63:0] mtime,
    // memory
    output logic  [63:0] csr_satp,
    output logic         icache_rqst,
    output logic  [63:0] icache_addr,
    output logic         icache_flsh,
    input  logic         icache_done,
    input  logic         icache_pgft,
    input  logic [127:0] icache_data,
    output logic  [7:0] dcache_rqst,
    output logic  [1:0] dcache_rsrv,
    output logic        dcache_wena,
    output logic [63:0] dcache_addr,
    output logic  [2:0] dcache_bits,
    input  logic  [7:0] dcache_done,
    input  logic  [1:0] dcache_pgft,
    input  logic [63:0] dcache_rdat,
    output logic [63:0] dcache_wdat,
    output logic        dcache_flsh,
    // commit info
    output logic        cmt       [3:0],
    output logic  [1:0] cmt_level [3:0],
    output logic [63:0] cmt_pc    [3:0],
    output logic [31:0] cmt_ir    [3:0],
    output logic        cmt_gpr   [3:0],
    output logic        cmt_csr   [3:0],
    output logic        cmt_mem   [3:0],
    output logic        cmt_mexc, // related to some CSR change
    output logic        cmt_sexc,
    output logic        cmt_ret,
    output logic [63:0] cmt_mstatus, // CSR deltas are calculated by states
    output logic [63:0] cmt_mcause,
    output logic [63:0] cmt_mepc,
    output logic [63:0] cmt_mtval,
    output logic [63:0] cmt_scause,
    output logic [63:0] cmt_sepc,
    output logic [63:0] cmt_stval,
    output logic        del_gprw  [3:0],
    output logic  [5:0] del_gpra  [3:0],
    output logic [63:0] del_gprv  [3:0],
    output logic        del_csrw,
    output logic [11:0] del_csra,
    output logic [63:0] del_csrv,
    output logic  [7:0] del_memw,
    output logic [63:0] del_mema,
    output logic [63:0] del_memv,
    // stats
    output logic [63:0] stallpc,
    output logic [63:0] misp
);
    /* instantiate */
    pipeline pipeline_inst(clk, rst, eiptip, mtime, csr_satp,
        icache_rqst, icache_addr, icache_flsh,
        icache_done, icache_pgft, icache_data,
        dcache_rqst, dcache_rsrv, dcache_wena, dcache_addr,
        dcache_bits, dcache_done, dcache_pgft, dcache_rdat,
        dcache_wdat, dcache_flsh);

    /* architectural states change */
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            cmt[i] = pipeline_inst.wb_stage_inst.cqpop[i] &
                    ~pipeline_inst.wb_stage_inst.cqinfo[i].rda[6];
            cmt_level[i] = pipeline_inst.csr_inst.level;
            cmt_pc[i] = pipeline_inst.wb_stage_inst.cqinfo[i].pc;
            cmt_ir[i] = pipeline_inst.wb_stage_inst.cqinfo[i].ir;
            cmt_gpr[i] = |pipeline_inst.wb_stage_inst.cqinfo[i].rda[5:0];
            cmt_csr[i] = pipeline_inst.wb_stage_inst.cqinfo[i].csrw;
            cmt_mem[i] = pipeline_inst.wb_stage_inst.cqinfo[i].memw;
            del_gprw[i] = cmt[i] & cmt_gpr[i];
            del_gpra[i] = pipeline_inst.wb_stage_inst.cqinfo[i].rda[5:0];
            del_gprv[i] = pipeline_inst.wb_stage_inst.cqdata[i][63:0];
        end
        if (pipeline_inst.wb_stage_inst.excep)
            for (int i = 0; i < 4; i++) if (~cmt[i]) begin cmt[i] = 1; break; end
        cmt_mexc = pipeline_inst.csr_inst.ein & ~pipeline_inst.csr_inst.trapintos;
        cmt_sexc = pipeline_inst.csr_inst.ein &  pipeline_inst.csr_inst.trapintos;
        cmt_ret = pipeline_inst.csr_inst.ret[2];
        cmt_mstatus = pipeline_inst.csr_inst.mstatus;
        cmt_mcause = pipeline_inst.csr_inst.mcause;
        cmt_mepc = pipeline_inst.csr_inst.mepc;
        cmt_mtval = pipeline_inst.csr_inst.mtval;
        cmt_scause = pipeline_inst.csr_inst.scause;
        cmt_sepc = pipeline_inst.csr_inst.sepc;
        cmt_stval = pipeline_inst.csr_inst.stval;
        del_csrw = pipeline_inst.csr_inst.wena;
        del_csra = pipeline_inst.csr_inst.addr;
        del_csrv = pipeline_inst.csr_inst.wres;
        del_memw = 0;
        del_mema = dcache_addr;
        del_memv = dcache_wdat;
        if (|dcache_rqst & dcache_wena)
            del_memw = 8'(1 << dcache_bits[1:0]);
        if (|dcache_rqst & dcache_rsrv == 2'b1)
            del_memw = {1'b1, dcache_wena, 2'd0, 4'(1 << dcache_bits[1:0])};
    end

    /* other stats */
    always_comb begin stallpc = 0; for (int i = 3; i >= 0; i--)
        if ( pipeline_inst.wb_stage_inst.cqvalid[i] &
            ~pipeline_inst.wb_stage_inst.cqpop[i] & ~pipeline_inst.redir)
            stallpc = pipeline_inst.wb_stage_inst.cqinfo[i].pc; end
    always_ff @(posedge clk) if (rst) misp <= 0;
        else if (pipeline_inst.redir) misp <= misp + 1;
endmodule

module mul(input logic clk, input logic rst, input logic flush,
    input logic ena, input mul_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    mul_rqst_t [3:0] buffer[`CQSZ-1:0]; logic in, out;
    logic [`lgCQSZ-1:0] front; logic [`lgCQSZ:0] num;
    mul_rqst_t [3:0] cur; logic [2:0] curnum; logic [1:0] curi;
    always_comb begin
        in = 0; curnum = 0; curi = 0;
        for (int i = 0; i < 4; i++) if (rqst[i].id[`lgCQSZ]) in = 1;
        for (int i = 0; i < 4; i++) if (cur[i].id[`lgCQSZ]) curnum++;
        for (int i = 3; i >= 0; i--) if (cur[i].id[`lgCQSZ]) curi = 2'(i);
    end
    always_comb out = |num & curnum <= 3'(ena);
    always_ff @(posedge clk) if (rst | flush) front <= 0;
        else front <= front + `lgCQSZ'(out);
    always_ff @(posedge clk) if (rst | flush) num <= 0;
        else num <= num + `lgCQSZ'(in) - `lgCQSZ'(out);
    always_ff @(posedge clk) if (in) buffer[front + num[`lgCQSZ-1:0]] <= rqst;
    always_ff @(posedge clk) if (rst | flush) cur <= 0;
        else if (out) cur <= buffer[front];
        else if (ena) cur[curi].id <= 0;
`define mullatency 6
    logic [`mullatency-1:0][63:0] r_q;
    logic [`mullatency-1:0][`lgCQSZ:0] id;
    logic [127:0] res, aext, bext;
    logic [63:0] rext;
    // op: 0 -> MUL  1 -> MULH  2 -> MULHSU  3 -> MULHU  4 -> MULW
    logic [4:0] op; logic [63:0] a, b;
    always_comb begin
        op = cur[curi].op; a = cur[curi].a; b = cur[curi].b;
        aext = {128{op[0] | op[1] | op[2] | op[4]}} & {{64{a[63]}}, a} |
               {128{op[3]}} & {64'd0, a};
        bext = {128{op[0] | op[1] | op[4]}} & {{64{b[63]}}, b} |
               {128{op[2] | op[3]}} & {64'd0, b};
        res = aext * bext;
        rext = {64{op[0]}} & res[63:0] | {64{op[4]}} & {{32{res[31]}}, res[31:0]} |
               {64{op[1] | op[2] | op[3]}} & res[127:64];
    end
    always_ff @(posedge clk)
        if (rst | flush) id <= {`lgCQSZ+1{`mullatency'd0}};
        else if (ena | ~id[0][`lgCQSZ]) begin
            id <= {cur[curi].id, id[`mullatency-1:1]};
            r_q <= {rext, r_q[`mullatency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = id[0];
endmodule

module div(input logic clk, input logic rst, input logic flush,
    input logic ena, input div_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    div_rqst_t [3:0] buffer[`CQSZ-1:0]; logic in, out;
    logic [`lgCQSZ-1:0] front; logic [`lgCQSZ:0] num;
    div_rqst_t [3:0] cur; logic [2:0] curnum; logic [1:0] curi;
    always_comb begin
        in = 0; curnum = 0; curi = 0;
        for (int i = 0; i < 4; i++) if (rqst[i].id[`lgCQSZ]) in = 1;
        for (int i = 0; i < 4; i++) if (cur[i].id[`lgCQSZ]) curnum++;
        for (int i = 3; i >= 0; i--) if (cur[i].id[`lgCQSZ]) curi = 2'(i);
    end
    always_comb out = |num & curnum <= 3'(ena);
    always_ff @(posedge clk) if (rst | flush) front <= 0;
        else front <= front + `lgCQSZ'(out);
    always_ff @(posedge clk) if (rst | flush) num <= 0;
        else num <= num + `lgCQSZ'(in) - `lgCQSZ'(out);
    always_ff @(posedge clk) if (in) buffer[front + num[`lgCQSZ-1:0]] <= rqst;
    always_ff @(posedge clk) if (rst | flush) cur <= 0;
        else if (out) cur <= buffer[front];
        else if (ena) cur[curi].id <= 0;

// should be un-pipelined

`define divlatency 6
    logic [`divlatency-1:0][63:0] r_q;
    logic [`divlatency-1:0][`lgCQSZ:0] valid;
    logic [63:0] res;
    logic [31:0] res32;
    // op: 0 -> DIV   1 -> DIVU   2 -> REM   3 -> REMU
    //     4 -> DIVW  5 -> DIVUW  6 -> REMW  7 -> REMUW
    logic [7:0] op; logic [63:0] a, b;
    always_comb a = cur[curi].a;
    always_comb b = cur[curi].b;
    always_comb op = cur[curi].op;
    always_comb if (a[31:0] == 32'h80000000 & b[31:0] == 32'hffffffff & (op[4] | op[6]))
            res32 = {32{op[4]}} & a[31:0];
        else if (~|b) res32 = {32{op[4] | op[5]}} & -32'd1 |
                              {32{op[6] | op[7]}} & a[31:0];
        else res32 = {32{op[4]}} & $signed($signed(a[31:0]) / $signed(b[31:0])) |
                     {32{op[5]}} & (a[31:0] / b[31:0]) |
                     {32{op[6]}} & $signed($signed(a[31:0]) % $signed(b[31:0])) |
                     {32{op[7]}} & (a[31:0] % b[31:0]);
    always_comb if (a == 64'h8000000000000000 &
                    b == 64'hffffffffffffffff & (op[0] | op[2]))
            res = {64{op[0]}} & a;
        else if (~|b & |op[3:0]) res = {64{op[0] | op[1]}} & -64'd1 |
                                       {64{op[2] | op[3]}} & a;
        else res = {64{op[0]}} & $signed($signed(a) / $signed(b)) |
                {64{op[1]}} & (a / b) |
                {64{op[2]}} & $signed($signed(a) % $signed(b)) |
                {64{op[3]}} & (a % b) |
                {64{|op[7:4]}} & {{32{res32[31]}}, res32};
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`divlatency'd0}};
        else if (ena | ~valid[0][`lgCQSZ]) begin
            valid <= {cur[curi].id, valid[`divlatency-1:1]};
            r_q <= {res, r_q[`divlatency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
endmodule

function logic [63:0] single2double(input logic [31:0] s);
    logic sgn, ov;
    logic [7:0] e;
    logic [22:0] b;
    {sgn, e, b} = s;
    single2double = {sgn, {3'd0, e} + 11'd896, b, 29'd0};
endfunction

function logic [31:0] double2single(input logic [63:0] d, input logic [2:0] rm);
    logic s, ov;
    logic [10:0] e;
    logic [51:0] b;
    {s, e, b} = d;
    if (b[28]) {ov, b[51:29]} = {1'b0, b[51:29]} + 1; else ov = 0;
    if (ov) begin b = b >> 1; if (e != 1023) e++; end
    if (e < 896) e = 896; else if (e > 1151) e = 1151;
    e = e - 896;
    double2single = {s, e[7:0], b[51:29]};
endfunction

module fpu(input logic clk, input logic rst, input logic flush,
    input logic ena, input fpu_rqst_t [3:0] rqst,
    output logic [`lgCQSZ:0] done, output logic [64:0] r, output logic e
);
    fpu_rqst_t [3:0] buffer[`CQSZ-1:0]; logic in, out;
    logic [`lgCQSZ-1:0] front; logic [`lgCQSZ:0] num;
    fpu_rqst_t [3:0] cur; logic [2:0] curnum; logic [1:0] curi;
    always_comb begin
        in = 0; curnum = 0; curi = 0;
        for (int i = 0; i < 4; i++) if (rqst[i].id[`lgCQSZ]) in = 1;
        for (int i = 0; i < 4; i++) if (cur[i].id[`lgCQSZ]) curnum++;
        for (int i = 3; i >= 0; i--) if (cur[i].id[`lgCQSZ]) curi = 2'(i);
    end
    always_comb out = |num & curnum <= 3'(ena);
    always_ff @(posedge clk) if (rst | flush) front <= 0;
        else front <= front + `lgCQSZ'(out);
    always_ff @(posedge clk) if (rst | flush) num <= 0;
        else num <= num + `lgCQSZ'(in) - `lgCQSZ'(out);
    always_ff @(posedge clk) if (in) buffer[front + num[`lgCQSZ-1:0]] <= rqst;
    always_ff @(posedge clk) if (rst | flush) cur <= 0;
        else if (out) cur <= buffer[front];
        else if (ena) cur[curi].id <= 0;
`define fpulatency 6
    logic [`fpulatency-1:0][63:0] r_q;
    logic [`fpulatency-1:0][`lgCQSZ:0] valid;
    logic [63:0] res, ad, bd;
    logic [9:0] fclass;
    real af, bf;
    logic [20:0] op; logic [63:0] a, b; logic [2:0] rm; logic double;
    always_comb op = cur[curi].op;
    always_comb a = cur[curi].a;
    always_comb b = cur[curi].b;
    always_comb rm = cur[curi].rm;
    always_comb double = cur[curi].double;
    always_comb begin
        if (double) ad = a; else if (a[63:32] == -32'd1) ad = single2double(a[31:0]);
        else ad = {-32'd1, 32'h7fc00000};
        if (double) bd = b; else if (b[63:32] == -32'd1) bd = single2double(b[31:0]);
        else bd = {-32'd1, 32'h7fc00000};
        af = $bitstoreal(ad);
        bf = $bitstoreal(bd);
    end
    always_comb if (double) fclass[0] = a[63] & a[62:52] == -11'd1 & a[51:0] == 0;
                       else fclass[0] = a[31] & a[30:23] == -8'd1 & a[22:0] == 0;
    always_comb if (double) fclass[1] = a[63] & a[62:52] != -11'd1 & a[62:52] != 0;
                       else fclass[1] = a[31] & a[30:23] != -8'd1 & a[30:23] != 0;
    always_comb if (double) fclass[2] = a[63] & a[62:52] == 0 & a[51:0] != 0;
                       else fclass[2] = a[31] & a[30:23] == 0 & a[22:0] != 0;
    always_comb if (double) fclass[3] = a[63] & a[62:0] == 0;
                       else fclass[3] = a[31] & a[30:0] == 0;
    always_comb if (double) fclass[4] = ~a[63] & a[62:0] == 0;
                       else fclass[4] = ~a[31] & a[30:0] == 0;
    always_comb if (double) fclass[5] = ~a[63] & a[62:52] == 0 & a[51:0] != 0;
                       else fclass[5] = ~a[31] & a[30:23] == 0 & a[22:0] != 0;
    always_comb if (double) fclass[6] = ~a[63] & a[62:52] != -11'd1 & a[62:52] != 0;
                       else fclass[6] = ~a[31] & a[30:23] != -8'd1 & a[30:23] != 0;
    always_comb if (double) fclass[7] = ~a[63] & a[62:52] == -11'd1 & a[51:0] == 0;
                       else fclass[7] = ~a[31] & a[30:23] == -8'd1 & a[22:0] == 0;
    always_comb if (double) fclass[8] = a[62:52] == -11'd1 & a[51] == 0;
                       else fclass[8] = a[30:23] == -8'd1 & a[22] == 0;
    always_comb if (double) fclass[9] = a[62:52] == -11'd1 & a[51] == 1;
                       else fclass[9] = a[30:23] == -8'd1 & a[22] == 1;
    always_comb begin
        res = {64{op[0]}} & $realtobits(af + bf) | // FADD
              {64{op[1]}} & $realtobits(af - bf) | // FSUB
              {64{op[2]}} & $realtobits(af * bf) | // FMUL
              {64{op[3]}} & $realtobits(-af * bf) | // FNMUL
              {64{op[4]}} & $realtobits(af / bf) | // FDIV
              {64{op[5]}} & $realtobits($sqrt(af)) | // FSQRT
              {64{op[6]}} & {bd[63], ad[62:0]} | // FSGNJ
              {64{op[7]}} & {~bd[63], ad[62:0]} | // FSGNJN
              {64{op[8]}} & {ad[63] ^ bd[63], ad[62:0]} | // FSGNJX
              {64{op[9]}} & $realtobits(af < bf ? af : bf) | // FMIN
              {64{op[10]}} & $realtobits(af > bf ? af : bf) | // FMAX
              {64{op[11]}} & {63'd0, af == bf} | // FEQ
              {64{op[12]}} & {63'd0, af < bf} | // FLT
              {64{op[13]}} & {63'd0, af <= bf} | // FLE
              {64{op[14]}} & a | //FMV.X.F
              {64{op[15]}} & {54'd0, fclass} | // FCLASS
              {64{op[16]}} & a | // FMV.F.X
              {64{op[17] & b[1:0] == 0}} & {32'd0, int'(af)} | // FCVT.W.F
              {64{op[17] & b[1:0] == 1}} & {32'd0, $signed(int'(af))} | // FCVT.WU.F
              {64{op[17] & b[1:0] == 2}} & longint'(af) | // FCVT.L.F
              {64{op[17] & b[1:0] == 3}} & $signed(longint'(af)) | // FCVT.LU.F
              {64{op[18] & b[1:0] == 0}} &
                    $realtobits(real'($signed({{32{a[31]}}, a[31:0]}))) | // FCVT.F.W
              {64{op[18] & b[1:0] == 1}} &
                    $realtobits(real'({32'd0, a[31:0]})) | // FCVT.F.WU
              {64{op[18] & b[1:0] == 2}} & $realtobits(real'($signed(a))) | // FCVT.F.L
              {64{op[18] & b[1:0] == 3}} & $realtobits(real'(a)) | // FCVT.F.LU
              {64{op[19]}} & a | // FCVT.S.D
              {64{op[20]}} & single2double(a[31:0]) | // FCVT.D.S
              0;
        if (op[17] & b[1:0] == 0) res = {{32{res[31]}}, res[31:0]};
        if (op[16]) begin
            if (~double) res = {-32'd1, res[31:0]};
        end else if (~op[11] & ~op[12] & ~op[13] & ~op[14] & ~op[15] & ~op[17]) begin
            if (res[62:52] == 11'h7ff) res = 64'h7ff8_0000_0000_0000;
            if (~double) res = {-32'd1, double2single(res, rm)};
        end
    end
    always_ff @(posedge clk)
        if (rst | flush) valid <= {`lgCQSZ+1{`fpulatency'd0}};
        else if (ena | ~valid[0][`lgCQSZ]) begin
            valid <= {cur[curi].id, valid[`fpulatency-1:1]};
            r_q <= {res, r_q[`fpulatency-1:1]};
        end
    always_comb r = {1'b0, r_q[0]};
    always_comb e = 0;
    always_comb done = valid[0];
endmodule

module regfile #(parameter dwidth = 64,
    parameter rports = 2, parameter wports = 1,
    parameter awidth = 6, parameter depth = 64) (
    input  logic clk, input logic rst,
    input  logic [rports-1:0][awidth-1:0] raddr,
    output logic [rports-1:0][dwidth-1:0] rvalue,
    input  logic [wports-1:0][awidth-1:0] waddr,
    input  logic [wports-1:0][dwidth-1:0] wvalue,
    input  logic [wports-1:0] wena
);
    int sel[depth-1:0];
    always_ff @(posedge clk)
        for (int i = 0; i < wports; i++) if (wena[i]) sel[waddr[i]] <= i;
    for (genvar i = 0; i < wports; i++) begin : dupregs
        logic [dwidth-1:0] regs[depth-1:0];
        always_ff @(posedge clk)
            if (wena[i]) regs[waddr[i]] <= wvalue[i];
    end
    for (genvar i = 0; i < rports; i++) begin
        logic [dwidth-1:0] dupval[wports-1:0];
        for (genvar j = 0; j < wports; j++)
            always_comb dupval[j] = dupregs[j].regs[raddr[i]];
        always_comb rvalue[i] = dupval[sel[raddr[i]]];
    end
endmodule

/********************************** CSR map ***********************************
User-level CSR:
    0x000 -- 0x005:
        0x000 -> ustatus    0x001 -> fflags    0x002 -> frm    0x003 -> fcsr
        0x004 -> uie        0x005 -> utvec
    0x040 -- 0x044:
        0x040 -> uscratch    0x041 -> uepc    0x042 -> ucause    0x043 -> utval
        0x044 -> uip
    0xc00 -- 0xc1f:
        0xc00 -> cycle    0xc01 -> time    0xc02 -> instret
        0xc03-0xc1f -> hpmcounter
Supervisor-level CSR:
    0x100 -- 0x106:
        0x100 -> sstatus    0x102 -> sedeleg    0x103 -> sideleg    0x104 -> sie
        0x105 -> stvec      0x106 -> scounteren
    0x140 -- 0x144:
        0x140 -> sscratch    0x141 -> sepc    0x142 -> scause    0x143 -> stval
        0x144 -> sip
    0x180 -- 0x180:
        0x180 -> satp
Machine-level CSR:
    0x300 -- 0x306:
        0x300 -> mstatus    0x301 -> misa     0x302 -> medeleg    0x303 -> mideleg
        0x304 -> mie        0x305 -> mtvec    0x306 -> mcounteren
    0x320 -- 0x33f:
        0x320 -> mcountinhibit    0x323-0x33f -> mhpmevent
    0x340 -- 0x344:
        0x340 -> mscratch    0x341 -> mepc    0x342 -> mcause    0x343 -> mtval
        0x344 -> mip
    0x3a0 -- 0x3bf:
        0x3a0,0x3a2 -> pmpcfg    0x3b0-0x3bf -> pmpaddr
    0x7a0 -- 0x7a3:
        0x7a0 -> tselect    0x7a1-0x7a3 -> tdata
    0x7b0 -- 0x7b3:
        0x7b0 -> dcsr       0x7b1 -> dpc    0x7b2-0x7b3 -> dscratch
    0xb00 -- 0xb1f:
        0xb00 -> mcycle    0xb02 -> minstret    0xb03-0xb1f -> mhpmcounter
    0xf11 -- 0xf14:
        0xf11 -> mvendorid    0xf12 -> marchid    0xf13 -> mimpid    0xf14 -> mhartid
******************************************************************************/
