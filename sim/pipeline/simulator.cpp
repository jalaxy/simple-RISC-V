#include "simulator.h"
#include <cstdio>
#include <cmath>

#define BTOD(b0, b1, b2, b3, b4, b5, b6, b7)                                                         \
    ((((uint64_t)(uint8_t)(b0)) << (uint64_t)0x00) | (((uint64_t)(uint8_t)(b1)) << (uint64_t)0x08) | \
     (((uint64_t)(uint8_t)(b2)) << (uint64_t)0x10) | (((uint64_t)(uint8_t)(b3)) << (uint64_t)0x18) | \
     (((uint64_t)(uint8_t)(b4)) << (uint64_t)0x20) | (((uint64_t)(uint8_t)(b5)) << (uint64_t)0x28) | \
     (((uint64_t)(uint8_t)(b6)) << (uint64_t)0x30) | (((uint64_t)(uint8_t)(b7)) << (uint64_t)0x38))
#define DTOB(x, i) ((uint8_t)((x) >> (8 * (uint64_t)(i))))
#define DLE(pb, addr) (BTOD((pb)[(addr) + 0], (pb)[(addr) + 1], (pb)[(addr) + 2], (pb)[(addr) + 3], \
                            (pb)[(addr) + 4], (pb)[(addr) + 5], (pb)[(addr) + 6], (pb)[(addr) + 7]))
#define BITS(dw, s, e) ((uint64_t)(dw) << (63 - (e)) >> (63 - (e) + (s)))
#define BIT(dw, i) BITS(dw, i, i)
#define SEXT(dw, width) ((BIT(dw, width - 1) ? (uint64_t)(-1) << (width) : 0) | (dw) & ~((uint64_t)(-1) << (width)))
#define CNAN64(x) (BITS(x, 52, 62) == 0x7ff && BITS(x, 0, 51) ? 0x7ff8'0000'0000'0000ul : (x))
#define CNAN32(x) (BITS(x, 23, 30) == 0x0ff && BITS(x, 0, 22) ? 0xffff'ffff'7fc0'0000ul : (x))
#define BNAN(p) *((uint32_t *)(p) + 1) = 0xffff'ffffu

/**
 * @brief simulator constructor
 * @param initpc initial pc
 * @param initmem initial memory
 */
simulator::simulator(const uint64_t &initpc, const std::map<uint64_t, uint8_t> &initmem)
{
    npc = initpc;
    memory = initmem;
    instfreq.clear();
    csr[0x000] = {"ustatus", 0};
    csr[0x001] = {"fflags", 0};
    csr[0x002] = {"frm", 0};
    csr[0x003] = {"fcsr", 0};
    csr[0x004] = {"uie", 0};
    csr[0x005] = {"utvec", 0};
    csr[0x040] = {"uscratch", 0};
    csr[0x041] = {"uepc", 0};
    csr[0x042] = {"ucause", 0};
    csr[0x043] = {"utval", 0};
    csr[0x044] = {"uip", 0};
    csr[0xc00] = {"cycle", 0};
    csr[0xc01] = {"time", 0};
    csr[0xc02] = {"instret", 0};
    for (int i = 0xc03; i < 0xc20; i++)
    {
        static char hpmcounter[32][32];
        sprintf(hpmcounter[i % 32], "hpmcounter%d", i % 32);
        csr[i] = {hpmcounter[i % 32], 0};
    }
    csr[0x100] = {"sstatus", 0};
    csr[0x102] = {"sedeleg", 0};
    csr[0x103] = {"sideleg", 0};
    csr[0x104] = {"sie", 0};
    csr[0x105] = {"stvec", 0};
    csr[0x106] = {"scounteren", 0};
    csr[0x140] = {"sscrach", 0};
    csr[0x141] = {"sepc", 0};
    csr[0x142] = {"scause", 0};
    csr[0x143] = {"stval", 0};
    csr[0x144] = {"sip", 0};
    csr[0x180] = {"satp", 0};
    csr[0x300] = {"mstatus", 0};
    csr[0x301] = {"misa", 0x800000000004112d};
    csr[0x302] = {"medeleg", 0};
    csr[0x303] = {"mideleg", 0};
    csr[0x304] = {"mie", 0};
    csr[0x305] = {"mtvec", 0};
    csr[0x306] = {"mcounteren", 0};
    csr[0x320] = {"mcounterinhibit", 0};
    for (int i = 0x323; i < 0x340; i++)
    {
        static char mhpmevent[32][32];
        sprintf(mhpmevent[i % 32], "mhpmevent%d", i % 32);
        csr[i] = {mhpmevent[i % 32], 0};
    }
    csr[0x340] = {"mscratch", 0};
    csr[0x341] = {"mepc", 0};
    csr[0x342] = {"mcause", 0};
    csr[0x343] = {"mtval", 0};
    csr[0x344] = {"mip", 0};
    csr[0x3a0] = {"pmpcfg0", 0};
    csr[0x3a2] = {"pmpcfg2", 0};
    for (int i = 0x3b0; i < 0x3c0; i++)
    {
        static char pmpaddr[32][32];
        sprintf(pmpaddr[i % 16], "pmpaddr%d", i % 16);
        csr[i] = {pmpaddr[i % 16], 0};
    }
    csr[0x7a0] = {"tselect", 0};
    csr[0x7a1] = {"tdata1", 0};
    csr[0x7a2] = {"tdata2", 0};
    csr[0x7a3] = {"tdata3", 0};
    csr[0x7b0] = {"dcsr", 0};
    csr[0x7b1] = {"dpc", 0};
    csr[0x7b2] = {"dscratch0", 0};
    csr[0x7b3] = {"dscratch1", 0};
    csr[0xb00] = {"mcycle", 0};
    csr[0xb02] = {"minstret", 0};
    for (int i = 0xb03; i < 0xb20; i++)
    {
        static char mhpmcounter[32][32];
        sprintf(mhpmcounter[i % 32], "mhpmcounter%d", i % 32);
        csr[i] = {mhpmcounter[i % 32], 0};
    }
    csr[0xf11] = {"mvendorid", 0};
    csr[0xf12] = {"marchid", 0};
    csr[0xf13] = {"mimpid", 0};
    csr[0xf14] = {"mhartid", 0};
}

/**
 * @brief step with an instruction
 */
void simulator::step(int nojump)
{
    csr[0xb00].val++;
    csr[0xb02].val++;

    // instruction fetch
    pc = npc;
    uint32_t idata = DLE(memory, pc);

    // compressed instruction extension
    ir = 0;
    switch ((BITS(idata, 13, 15) << 2) | BITS(idata, 0, 1))
    {
    case 0b00000:
        if (BITS(idata, 5, 12)) // C.ADDI4SPN
        {
            ir |= BITS(idata, 7, 10) << 26;
            ir |= BITS(idata, 11, 12) << 24;
            ir |= BIT(idata, 5) << 23;
            ir |= BIT(idata, 6) << 22;
            ir |= 2 << 15;
            ir |= (BITS(idata, 2, 4) + 8) << 7;
            ir |= 0x13;
        }
        break;
    case 0b00100: // C.FLD
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BITS(idata, 10, 12) << 23;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= (BITS(idata, 2, 4) + 8) << 7;
        ir |= 7;
        break;
    case 0b01000: // C.LW
        ir |= BIT(idata, 5) << 26;
        ir |= BITS(idata, 10, 12) << 23;
        ir |= BIT(idata, 6) << 22;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 2 << 12;
        ir |= (BITS(idata, 2, 4) + 8) << 7;
        ir |= 3;
        break;
    case 0b01100: // C.LD
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BITS(idata, 10, 12) << 23;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= (BITS(idata, 2, 4) + 8) << 7;
        ir |= 3;
        break;
    case 0b10100: // C.FSD
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= (BITS(idata, 2, 4) + 8) << 20;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= 0x27;
        break;
    case 0b11000: // C.SW
        ir |= BIT(idata, 5) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= (BITS(idata, 2, 4) + 8) << 20;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 2 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= BIT(idata, 6) << 9;
        ir |= 0x23;
        break;
    case 0b11100: // C.SD
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= (BITS(idata, 2, 4) + 8) << 20;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= 0x23;
        break;
    case 0b00001: // C.ADDI / C.NOP
        ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= BITS(idata, 7, 11) << 15;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 0x13;
        break;
    case 0b00101: // ADDIW
        ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= BITS(idata, 7, 11) << 15;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 0x1b;
        break;
    case 0b01001: // C.LI
        ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 0x13;
        break;
    case 0b01101:
        if (BITS(idata, 7, 11) == 2) // C.ADDI16SP
        {
            ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 29;
            ir |= BITS(idata, 3, 4) << 27;
            ir |= BIT(idata, 5) << 26;
            ir |= BIT(idata, 2) << 25;
            ir |= BIT(idata, 6) << 24;
            ir |= 2 << 15;
            ir |= 2 << 7;
            ir |= 0x13;
        }
        else if (BITS(idata, 2, 6) || BIT(idata, 12)) // C.LUI
        {
            ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 17;
            ir |= BITS(idata, 2, 6) << 12;
            ir |= BITS(idata, 7, 11) << 7;
            ir |= 0x37;
        }
        break;
    case 0b10001:
        if (BIT(idata, 11) == 0) // C.SRLI / C.SRAI
        {
            ir |= BITS(idata, 10, 11) << 30;
            ir |= BIT(idata, 12) << 25;
            ir |= BITS(idata, 2, 6) << 20;
            ir |= (BITS(idata, 7, 9) + 8) << 15;
            ir |= 5 << 12;
            ir |= (BITS(idata, 7, 9) + 8) << 7;
            ir |= 0x13;
        }
        else if (BIT(idata, 10) == 0) // C.ANDI
        {
            ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 25;
            ir |= BITS(idata, 2, 6) << 20;
            ir |= (BITS(idata, 7, 9) + 8) << 15;
            ir |= 7 << 12;
            ir |= (BITS(idata, 7, 9) + 8) << 7;
            ir |= 0x13;
        }
        else
            switch ((BIT(idata, 12) << 2) | BITS(idata, 5, 6))
            {
            case 0b000: // C.SUB
                ir |= 0b0100000 << 25;
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b001: // C.XOR
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= 4 << 12;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b010: // C.OR
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= 6 << 12;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b011: // C.AND
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= 7 << 12;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b100: // C.SUBW
                ir |= 0b0100000 << 25;
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x3b;
                break;
            case 0b101: // C.ADDW
                ir |= (BITS(idata, 2, 4) + 8) << 20;
                ir |= (BITS(idata, 7, 9) + 8) << 15;
                ir |= (BITS(idata, 7, 9) + 8) << 7;
                ir |= 0x3b;
                break;
            }
        break;
    case 0b10101: // C.J
        ir |= BIT(idata, 12) << 31;
        ir |= BIT(idata, 8) << 30;
        ir |= BITS(idata, 9, 10) << 28;
        ir |= BIT(idata, 6) << 27;
        ir |= BIT(idata, 7) << 26;
        ir |= BIT(idata, 2) << 25;
        ir |= BIT(idata, 11) << 24;
        ir |= BITS(idata, 3, 5) << 21;
        ir |= (BIT(idata, 12) ? (1 << 9) - 1 : 0) << 12;
        ir |= 0x6f;
        break;
    case 0b11001: // C.BEQZ
        ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 28;
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BIT(idata, 2) << 25;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= BITS(idata, 3, 4) << 8;
        ir |= BIT(idata, 12) << 7;
        ir |= 0x63;
        break;
    case 0b11101: // C.BNEZ
        ir |= (BIT(idata, 12) ? (uint64_t)(-1) : 0) << 28;
        ir |= BITS(idata, 5, 6) << 26;
        ir |= BIT(idata, 2) << 25;
        ir |= (BITS(idata, 7, 9) + 8) << 15;
        ir |= 1 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= BITS(idata, 3, 4) << 8;
        ir |= BIT(idata, 12) << 7;
        ir |= 0x63;
        break;
    case 0b00010:
        if (BIT(idata, 12) || BITS(idata, 2, 6)) // C.SLLI
        {
            ir |= BIT(idata, 12) << 25;
            ir |= BITS(idata, 2, 6) << 20;
            ir |= BITS(idata, 7, 11) << 15;
            ir |= 1 << 12;
            ir |= BITS(idata, 7, 11) << 7;
            ir |= 0x13;
        }
        break;
    case 0b00110: // C.FLDSP
        ir |= BITS(idata, 2, 4) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 5, 6) << 23;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 7;
        break;
    case 0b01010: // C.LWSP
        ir |= BITS(idata, 2, 3) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 4, 6) << 22;
        ir |= 2 << 15;
        ir |= 2 << 12;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 3;
        break;
    case 0b01110: // C.LDSP
        ir |= BITS(idata, 2, 4) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 5, 6) << 23;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 7, 11) << 7;
        ir |= 3;
        break;
    case 0b10010:
        if (!BIT(idata, 12) && !BITS(idata, 2, 6) && BITS(idata, 7, 11)) // C.JR
            ir |= (BITS(idata, 7, 11) << 15) | 0x67;
        if (!BIT(idata, 12) && BITS(idata, 2, 6)) // C.MV
        {
            ir |= BITS(idata, 2, 6) << 20;
            ir |= BITS(idata, 7, 11) << 7;
            ir |= 0x33;
        }
        if (BIT(idata, 12) && !BITS(idata, 2, 6) && BITS(idata, 7, 11) == 0) // C.EBREAK
            ir = (1 << 20) | 0x73;
        if (BIT(idata, 12) && !BITS(idata, 2, 6) && BITS(idata, 7, 11)) // C.JALR
        {
            ir |= BITS(idata, 7, 11) << 15;
            ir |= 1 << 7;
            ir |= 0x67;
        }
        if (BIT(idata, 12) && BITS(idata, 2, 6))
        {
            ir |= BITS(idata, 2, 6) << 20;
            ir |= BITS(idata, 7, 11) << 15;
            ir |= BITS(idata, 7, 11) << 7;
            ir |= 0x33;
        }
        break;
    case 0b10110: // C.FSDSP
        ir |= BITS(idata, 7, 9) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= 0x27;
        break;
    case 0b11010: // C.SWSP
        ir |= BITS(idata, 7, 8) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= 2 << 15;
        ir |= 2 << 12;
        ir |= BITS(idata, 9, 11) << 9;
        ir |= 0x23;
        break;
    case 0b11110: // C.SDSP
        ir |= BITS(idata, 7, 9) << 26;
        ir |= BIT(idata, 12) << 25;
        ir |= BITS(idata, 2, 6) << 20;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= BITS(idata, 10, 11) << 10;
        ir |= 0x23;
        break;
    default:
        ir = idata;
        break;
    }

    // decode and execution
    if (BITS(idata, 0, 1) == 3)
        sprintf(asmcode, "unimp 0x%08x", idata);
    else
        sprintf(asmcode, "unimp 0x%04x", idata & 0xffff);
    mwwidth = 0;
    csraddr = -1;
    uint8_t funct3 = BITS(ir, 12, 14), jump = 0, excp = 0;
    uint8_t rda = BITS(ir, 7, 11), rs1a = BITS(ir, 15, 19),
            rs2a = BITS(ir, 20, 24), rs3a = BITS(ir, 27, 31);
    uint64_t &rd = arregs[rda], rs1 = arregs[rs1a],
             rs2 = arregs[rs2a], rs3 = arregs[rs3a], addr;
    double &dd = *(double *)&arregs[rda + 32], ds1 = *(double *)&arregs[rs1a + 32],
           ds2 = *(double *)&arregs[rs2a + 32], ds3 = *(double *)&arregs[rs3a + 32];
    float &sd = *(float *)&arregs[rda + 32], ss1 = *(float *)&arregs[rs1a + 32],
          ss2 = *(float *)&arregs[rs2a + 32], ss3 = *(float *)&arregs[rs3a + 32];
    if (BITS(ds1, 32, 63) != 0xffff'ffffu) // NaN-boxing
        ss1 = CNAN32(ss1);
    if (BITS(ds2, 32, 63) != 0xffff'ffffu)
        ss2 = CNAN32(ss2);
    int64_t imm;
    switch (BITS(ir, 0, 6)) // opcode
    {
    case 0b0000011: // LOAD
        imm = SEXT(BITS(ir, 20, 31), 12);
        addr = rs1 + imm;
        rd = DLE(memory, addr);
        if (funct3 == 0b000) // LB
            rd = (int64_t)(int8_t)rd;
        else if (funct3 == 0b100) // LBU
            rd = (uint8_t)rd;
        else if (funct3 == 0b001) // LH
            rd = (int64_t)(int16_t)rd;
        else if (funct3 == 0b101) // LHU
            rd = (uint16_t)rd;
        else if (funct3 == 0b010) // LW
            rd = (int64_t)(int32_t)rd;
        else if (funct3 == 0b110) // LWU
            rd = (uint32_t)rd;
        static const char *lnames[] = {"lb", "lh", "lw", "ld", "lbu", "lhu", "lwu", ""};
        if (funct3 != 7)
            sprintf(asmcode, "%s x%d, %ld(x%d)", lnames[funct3], rda, imm, rs1a);
        break;
    case 0b0000111: // LOAD-FP
        imm = SEXT(BITS(ir, 20, 31), 12);
        addr = rs1 + imm;
        if (funct3 == 0b010) // FLW
            *(uint32_t *)&sd = DLE(memory, addr), BNAN(&dd);
        else if (funct3 == 0b011) // FLD
            *(uint64_t *)&dd = DLE(memory, addr);
        if (funct3 == 2 || funct3 == 3)
            sprintf(asmcode, "f%s f%d, %ld(x%d)", lnames[funct3], rda, imm, rs1a);
        break;
    case 0b0001111: // MISC-MEM
        switch (funct3)
        {
        case 0b000: // FENCE
            if (BIT(ir, 31))
                sprintf(asmcode, "fence.tso");
            else if ((BIT(ir, 27) | BIT(ir, 26) | BIT(ir, 25) | BIT(ir, 24)) &
                     (BIT(ir, 23) | BIT(ir, 22) | BIT(ir, 21) | BIT(ir, 20)))
                sprintf(asmcode, "fence.%s%s%s%s.%s%s%s%s",
                        BIT(ir, 27) ? "i" : "", BIT(ir, 26) ? "o" : "",
                        BIT(ir, 25) ? "r" : "", BIT(ir, 24) ? "w" : "",
                        BIT(ir, 23) ? "i" : "", BIT(ir, 22) ? "o" : "",
                        BIT(ir, 21) ? "r" : "", BIT(ir, 20) ? "w" : "");
            break;
        case 0b001: // FENCE.I
            sprintf(asmcode, "fence.i");
            break;
        }
        break;
    case 0b0010011: // OP-IMM
        imm = SEXT(BITS(ir, 20, 31), 12);
        if (funct3 == 0b000) // ADDI
            rd = rs1 + imm;
        else if (funct3 == 0b010) // SLTI
            rd = (int64_t)rs1 < imm;
        else if (funct3 == 0b011) // SLTIU
            rd = rs1 < (uint64_t)imm;
        else if (funct3 == 0b100) // XORI
            rd = rs1 ^ imm;
        else if (funct3 == 0b110) // ORI
            rd = rs1 | imm;
        else if (funct3 == 0b111) // ANDI
            rd = rs1 & imm;
        else if (funct3 == 0b001) // SLLI
            rd = rs1 << (imm &= 0x3f);
        else if (funct3 == 0b101)
            if (BIT(ir, 30)) // SRAI
                rd = (int64_t)rs1 >> (int64_t)(imm &= 0x3f);
            else // SRLI
                rd = rs1 >> (imm &= 0x3f);
        static const char *iname[] = {
            "addi", "slli", "slti", "sltiu", "xori", "srli", "ori", "andi"};
        if (ir == 0x13)
            sprintf(asmcode, "nop");
        else
            sprintf(asmcode, "%s x%d, x%d, %ld",
                    funct3 == 5 && BIT(ir, 30) ? "srai" : iname[funct3], rda, rs1a, imm);
        break;
    case 0b0010111: // AUIPC
        rd = pc + SEXT(BITS(ir, 12, 31) << 12, 32);
        sprintf(asmcode, "auipc x%d, 0x%lx", rda, BITS(ir, 12, 31));
        break;
    case 0b0011011: // OP-IMM-32
        imm = SEXT(BITS(ir, 20, 31), 12);
        if (funct3 == 0b000) // ADDIW
            rd = (int32_t)rs1 + (int32_t)imm;
        else if (funct3 == 0b001) // SLLIW
            rd = (int32_t)rs1 << (int32_t)(imm &= 0x3f);
        else if (funct3 == 0b101)
            if (BIT(ir, 30)) // SRAIW
                rd = (int64_t)((int32_t)rs1 >> (int32_t)(imm &= 0x3f));
            else // SRLIW
                rd = (int64_t)(int32_t)((uint32_t)rs1 >> (imm &= 0x3f));
        if (funct3 == 0 || funct3 == 1 || funct3 == 5)
            sprintf(asmcode, "%sw x%d, x%d, %ld",
                    funct3 == 5 && BIT(ir, 30) ? "srai" : iname[funct3], rda, rs1a, imm);
        break;
    case 0b0100011: // STORE
    case 0b0100111: // STORE-FP
        imm = SEXT(BITS(ir, 25, 31) << 5 | BITS(ir, 7, 11), 12);
        mwaddr = rs1 + imm;
        mwdata = BIT(ir, 2) ? *(uint64_t *)&ds2 : rs2;
        mwwidth = 1 << BITS(ir, 12, 13);
        if (mwwidth < 8)
            mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
        for (int i = 0; i < mwwidth; i++)
            memory[mwaddr + i] = DTOB(mwdata, i);
        static const char *snames[] = {"sb", "sh", "sw", "sd"};
        if (funct3 < 4)
            sprintf(asmcode, "%s%s %c%d, %ld(x%d)",
                    BIT(ir, 2) ? "f" : "", snames[funct3],
                    BIT(ir, 2) ? 'f' : 'x', rs2a, imm, rs1a);
        break;
    case 0b0101111: // AMO
        switch (BITS(ir, 27, 31))
        {
        case 0b00010: // LR
            rd = DLE(memory, rs1);
            reserved[rs1] = reserved[rs1 + 1] = reserved[rs1 + 2] = reserved[rs1 + 3] = 1;
            if (funct3 == 0b010)
                rd = (int64_t)(int32_t)rd;
            else // LR.D
                reserved[rs1 + 4] = reserved[rs1 + 5] = reserved[rs1 + 6] = reserved[rs1 + 7] = 1;
            sprintf(asmcode, "lr.%c%s%s x%d, (x%d)", funct3 == 0b010 ? 'w' : 'd',
                    BIT(ir, 26) ? ".aq" : "", BIT(ir, 25) ? ".rl" : "", rda, rs1a);
            break;
        case 0b00011: // SC
            if (reserved[rs1] & reserved[rs1 + 1] & reserved[rs1 + 2] & reserved[rs1 + 3] &
                (funct3 == 0b010 |
                 reserved[rs1 + 4] & reserved[rs1 + 5] & reserved[rs1 + 6] & reserved[rs1 + 7]))
            {
                reserved[rs1] = reserved[rs1 + 1] = reserved[rs1 + 2] = reserved[rs1 + 3] = 0;
                if (funct3 == 0b011)
                    reserved[rs1 + 4] = reserved[rs1 + 5] = reserved[rs1 + 6] = reserved[rs1 + 7] = 0;
                rd = 0;
                mwaddr = rs1, mwdata = rs2;
                mwwidth = 1 << BITS(ir, 12, 13);
                if (mwwidth < 8)
                    mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
                for (int i = 0; i < mwwidth; i++)
                    memory[mwaddr + i] = DTOB(mwdata, i);
            }
            else
                rd = 1;
            sprintf(asmcode, "sc.%c%s%s x%d, x%d, (x%d)", funct3 == 0b010 ? 'w' : 'd',
                    BIT(ir, 26) ? ".aq" : "", BIT(ir, 25) ? ".rl" : "", rda, rs2a, rs1a);
            break;
        default:
            static const char *aname;
            static uint64_t rdata;
            rdata = DLE(memory, rs1);
            if (funct3 == 0b010) // AMO*.W
                rdata = (int64_t)(int32_t)rdata, rs2 = (int64_t)(int32_t)rs2;
            aname = NULL;
            if (BITS(ir, 27, 31) == 0b00001) // AMOSWAP
                aname = "swap", mwdata = rs2;
            if (BITS(ir, 27, 31) == 0b00000) // AMOADD
                aname = "add", mwdata = rs2 + rdata;
            if (BITS(ir, 27, 31) == 0b00100) // AMOXOR
                aname = "xor", mwdata = rs2 ^ rdata;
            if (BITS(ir, 27, 31) == 0b01100) // AMOAND
                aname = "and", mwdata = rs2 & rdata;
            if (BITS(ir, 27, 31) == 0b01000) // AMOOR
                aname = "or", mwdata = rs2 | rdata;
            if (BITS(ir, 27, 31) == 0b10000) // AMOMIN
                aname = "min", mwdata = (int64_t)rs2 < (int64_t)rdata ? rs2 : rdata;
            if (BITS(ir, 27, 31) == 0b10100) // AMOMAX
                aname = "max", mwdata = (int64_t)rs2 > (int64_t)rdata ? rs2 : rdata;
            if (BITS(ir, 27, 31) == 0b11000) // AMOMINU
                aname = "minu", mwdata = rs2 < rdata ? rs2 : rdata;
            if (BITS(ir, 27, 31) == 0b11100) // AMOMAXU
                aname = "maxu", mwdata = rs2 > rdata ? rs2 : rdata;
            if (aname == NULL)
                break;
            mwaddr = rs1;
            mwwidth = 1 << BITS(ir, 12, 13);
            rd = rdata;
            if (funct3 == 0b010) // AMO*.W
                rd = (int64_t)(int32_t)rd;
            if (mwwidth < 8)
                mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
            for (int i = 0; i < mwwidth; i++)
                memory[mwaddr + i] = DTOB(mwdata, i);
            sprintf(asmcode, "amo%s.%c%s%s x%d, x%d, (x%d)", aname, funct3 == 0b010 ? 'w' : 'd',
                    BIT(ir, 26) ? ".aq" : "", BIT(ir, 25) ? ".rl" : "", rda, rs2a, rs1a);
            break;
        }
        break;
    case 0b0110011: // OP
        static const char *oname[] = {
            "add", "sll", "slt", "sltu", "xor", "srl", "or", "and"};
        if (funct3 == 0 && BIT(ir, 30))
            sprintf(asmcode, "sub x%d, x%d, x%d", rda, rs1a, rs2a);
        else if (funct3 == 5 && BIT(ir, 30))
            sprintf(asmcode, "sra x%d, x%d, x%d", rda, rs1a, rs2a);
        else
            sprintf(asmcode, "%s x%d, x%d, x%d", oname[funct3], rda, rs1a, rs2a);
        if (BIT(ir, 25))
        {
            static const char *mname[] = {
                "mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"};
            sprintf(asmcode, "%s x%d, x%d, x%d", mname[funct3], rda, rs1a, rs2a);
            uint64_t a[4], b[4];
            a[0] = BITS(rs1, 0, 31), a[1] = BITS(rs1, 32, 63);
            a[2] = a[3] = BIT(rs1, 63) && funct3 <= 2 ? 0xffffffffull : 0;
            b[0] = BITS(rs2, 0, 31), b[1] = BITS(rs2, 32, 63);
            b[2] = b[3] = BIT(rs2, 63) && funct3 <= 1 ? 0xffffffffull : 0;
            uint64_t l = a[0] * b[0] + ((a[0] * b[1] + a[1] * b[0]) << 32);
            uint64_t h =
                ((a[0] * b[0] >> 32) + (a[0] * b[1] << 32 >> 32) + (a[1] * b[0] << 32 >> 32) >> 32) +
                (a[0] * b[1] >> 32) + (a[1] * b[0] >> 32) +
                (a[0] * b[2]) + (a[1] * b[1]) + (a[2] * b[0]) +
                (a[0] * b[3] + a[1] * b[2] + a[2] * b[1] + a[3] * b[0] << 32);
            if (funct3 == 0b000) // MUL
                rd = l;
            else if (funct3 <= 0b011) // MULH[[S]U]
                rd = h;
            else if (funct3 == 0b100) // DIV
                if (rs1 == 0x8000000000000000 && rs2 == -1)
                    rd = rs1;
                else
                    rd = rs2 == 0 ? -1 : (int64_t)rs1 / (int64_t)rs2;
            else if (funct3 == 0b101) // DIVU
                rd = rs2 == 0 ? (uint64_t)-1 : rs1 / rs2;
            else if (funct3 == 0b110) // REM
                if (rs1 == 0x8000000000000000 && rs2 == -1)
                    rd = 0;
                else
                    rd = rs2 == 0 ? (int64_t)rs1 : (int64_t)rs1 % (int64_t)rs2;
            else if (funct3 == 0b111) // REMU
                rd = rs2 == 0 ? rs1 : rs1 % rs2;
        }
        else if (funct3 == 0b000)
            if (BIT(ir, 30)) // SUB
                rd = rs1 - rs2;
            else // ADD
                rd = rs1 + rs2;
        else if (funct3 == 0b001) // SLL
            rd = rs1 << rs2;
        else if (funct3 == 0b010) // SLT
            rd = (int64_t)rs1 < (int64_t)rs2;
        else if (funct3 == 0b011) // SLTU
            rd = rs1 < rs2;
        else if (funct3 == 0b100) // XOR
            rd = rs1 ^ rs2;
        else if (funct3 == 0b101)
            if (BIT(ir, 30)) // SRA
                rd = (int64_t)rs1 >> (int64_t)rs2;
            else // SRL
                rd = rs1 >> rs2;
        else if (funct3 == 0b110) // OR
            rd = rs1 | rs2;
        else if (funct3 == 0b111) // AND
            rd = rs1 & rs2;
        break;
    case 0b0110111: // LUI
        rd = SEXT(BITS(ir, 12, 31) << 12, 32);
        sprintf(asmcode, "lui x%d, 0x%lx", rda, BITS(ir, 12, 31));
        break;
    case 0b0111011: // OP-32
        if (funct3 == 0 && BIT(ir, 30))
            sprintf(asmcode, "subw x%d, x%d, x%d", rda, rs1a, rs2a);
        else if (funct3 == 5 && BIT(ir, 30))
            sprintf(asmcode, "sraw x%d, x%d, x%d", rda, rs1a, rs2a);
        else if (funct3 == 0 || funct3 == 1 || funct3 == 5)
            sprintf(asmcode, "%sw x%d, x%d, x%d", oname[funct3], rda, rs1a, rs2a);
        if (BIT(ir, 25))
        {
            static const char *mwname[] = {
                "mulw", 0, 0, 0, "divw", "divuw", "remw", "remuw"};
            if (funct3 != 1 && funct3 != 2 && funct3 != 3)
                sprintf(asmcode, "%s x%d, x%d, x%d", mwname[funct3], rda, rs1a, rs2a);
            if (funct3 == 0b000) // MULW
                rd = SEXT(rs1 * rs2, 32);
            else if (funct3 == 0b100) // DIVW
                if ((int32_t)rs1 == 0x80000000 && (int32_t)rs2 == -1)
                    rd = rs1;
                else
                    rd = SEXT(rs2 == 0 ? -1 : (int32_t)rs1 / (int32_t)rs2, 32);
            else if (funct3 == 0b101) // DIVUW
                rd = SEXT(rs2 == 0 ? -1 : (uint32_t)rs1 / (uint32_t)rs2, 32);
            else if (funct3 == 0b110) // REMW
                if ((int32_t)rs1 == 0x80000000 && (int32_t)rs2 == -1)
                    rd = 0;
                else
                    rd = SEXT(rs2 == 0 ? (int32_t)rs1 : (int32_t)rs1 % (int32_t)rs2, 32);
            else if (funct3 == 0b111) // REMUW
                rd = SEXT(rs2 == 0 ? (uint32_t)rs1 : (uint32_t)rs1 % (uint32_t)rs2, 32);
        }
        else if (funct3 == 0b000)
            if (BIT(ir, 30)) // SUBW
                rd = (int32_t)rs1 - (int32_t)rs2;
            else // ADDW
                rd = (int32_t)rs1 + (int32_t)rs2;
        else if (funct3 == 0b001) // SLLW
            rd = (int32_t)rs1 << (int32_t)rs2;
        else if (funct3 == 0b101)
            if (BIT(ir, 30)) // SRAW
                rd = (int64_t)((int32_t)rs1 >> (int32_t)rs2);
            else // SRLW
                rd = (int64_t)(int32_t)((uint32_t)rs1 >> (uint32_t)rs2);
        break;
    case 0b1000011: // MADD
        BIT(ir, 25) ? (dd = ds1 * ds2 + ds3) : (sd = ss1 * ss2 + ss3, BNAN(&dd));
        sprintf(asmcode, "fmadd.%c f%d, f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's',
                rda, rs1a, rs2a, rs3a);
        break;
    case 0b1000111: // MSUB
        BIT(ir, 25) ? (dd = ds1 * ds2 - ds3) : (sd = ss1 * ss2 - ss3, BNAN(&dd));
        sprintf(asmcode, "fmsub.%c f%d, f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's',
                rda, rs1a, rs2a, rs3a);
        break;
    case 0b1001011: // NMSUB
        BIT(ir, 25) ? (dd = -ds1 * ds2 + ds3) : (sd = -ss1 * ss2 + ss3, BNAN(&dd));
        sprintf(asmcode, "fnmsub.%c f%d, f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's',
                rda, rs1a, rs2a, rs3a);
        break;
    case 0b1001111: // NMADD
        BIT(ir, 25) ? (dd = -ds1 * ds2 - ds3) : (sd = -ss1 * ss2 - ss3, BNAN(&dd));
        sprintf(asmcode, "fnmadd.%c f%d, f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's',
                rda, rs1a, rs2a, rs3a);
        break;
    case 0b1010011: // OP-FP
        switch (BITS(ir, 27, 31))
        {
        case 0b00000: // FADD
            BIT(ir, 25) ? (dd = ds1 + ds2) : (sd = ss1 + ss2, BNAN(&dd));
            sprintf(asmcode, "fadd.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
            break;
        case 0b00001: // FSUB
            BIT(ir, 25) ? (dd = ds1 - ds2) : (sd = ss1 - ss2, BNAN(&dd));
            sprintf(asmcode, "fsub.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
            break;
        case 0b00010: // FMUL
            BIT(ir, 25) ? (dd = ds1 * ds2) : (sd = ss1 * ss2, BNAN(&dd));
            sprintf(asmcode, "fmul.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
            break;
        case 0b00011: // FDIV
            BIT(ir, 25) ? (dd = ds1 / ds2) : (sd = ss1 / ss2, BNAN(&dd));
            sprintf(asmcode, "fdiv.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
            break;
        case 0b01011:
            if (BIT(ir, 25)) // FSQRT.D
                dd = sqrt(ds1), *(uint64_t *)&dd = CNAN64(*(uint64_t *)&dd);
            else // FSQRT.S
                sd = sqrtf(ss1), *(uint32_t *)&sd = CNAN32(*(uint32_t *)&sd), BNAN(&dd);
            sprintf(asmcode, "fsqrt.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
            break;
        case 0b00100:
            switch (funct3)
            {
            case 0:
                if (BIT(ir, 25)) // FSGNJ.D
                    dd = ds2 < 0 ? -ds1 : ds1;
                else
                    sd = ss2 < 0 ? -ss1 : ss1, BNAN(&dd);
                sprintf(asmcode, "fsgnj.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            case 1:
                if (BIT(ir, 25)) // FSGNJN.D
                    dd = ds2 < 0 ? ds1 : -ds1;
                else
                    sd = ss2 < 0 ? ss1 : -ss1, BNAN(&dd);
                sprintf(asmcode, "fsgnjn.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            case 2:
                if (BIT(ir, 25)) // FSGNJX.D
                    dd = ds1 < 0 ^ ds2 < 0 ? -ds1 : ds1;
                else
                    sd = ss1 < 0 ^ ss2 < 0 ? -ss1 : ss1, BNAN(&dd);
                sprintf(asmcode, "fsgnjx.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            }
            break;
        case 0b00101:
            switch (funct3)
            {
            case 0: // FMIN
                BIT(ir, 25) ? (dd = fmin(ds1, ds2)) : (sd = fminf(ss1, ss2), BNAN(&dd));
                sprintf(asmcode, "fmin.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            case 1: // FMAX
                BIT(ir, 25) ? (dd = fmax(ds1, ds2)) : (sd = fmaxf(ss1, ss2), BNAN(&dd));
                sprintf(asmcode, "fmax.%c f%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            }
            break;
        case 0b10100:
            switch (funct3)
            {
            case 0: // FLE
                rd = BIT(ir, 25) ? ds1 <= ds2 : ss1 <= ss2;
                sprintf(asmcode, "fle.%c x%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            case 1: // FLT
                rd = BIT(ir, 25) ? ds1 < ds2 : ss1 < ss2;
                sprintf(asmcode, "flt.%c x%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            case 2: // FEQ
                rd = BIT(ir, 25) ? ds1 == ds2 : ss1 == ss2;
                sprintf(asmcode, "feq.%c x%d, f%d, f%d", BIT(ir, 25) ? 'd' : 's', rda, rs1a, rs2a);
                break;
            }
            break;
        case 0b01000:
            if (BIT(ir, 25)) // FCVT.D.S
                dd = (double)ss1;
            else // FCVT.S.D
                sd = (float)ds1, BNAN(&dd);
            sprintf(asmcode, "fcvt.%s f%d, x%d", BIT(ir, 25) ? "d.s" : "s.d", rda, rs1a);
            break;
        case 0b11000: // FCVT.I.F
            switch ((BIT(ir, 25) << 2) | BITS(ir, 20, 21))
            {
            case 0b000: // FCVT.W.S
                rd = (int32_t)round(ss1);
                break;
            case 0b001: // FCVT.WU.S
                rd = (uint32_t)round(ss1);
                break;
            case 0b010: // FCVT.L.S
                rd = (int64_t)round(ss1);
                break;
            case 0b011: // FCVT.LU.S
                rd = round(ss1);
                break;
            case 0b100: // FCVT.W.D
                rd = (int32_t)round(ds1);
                break;
            case 0b101: // FCVT.WU.D
                rd = (uint32_t)round(ds1);
                break;
            case 0b110: // FCVT.L.D
                rd = (int64_t)round(ds1);
                break;
            case 0b111: // FCVT.LU.D
                rd = round(ds1);
                break;
            }
            sprintf(asmcode, "fcvt.%c.%c%s x%d, f%d",
                    BIT(ir, 21) ? 'l' : 'w', BIT(ir, 25) ? 'd' : 's',
                    BIT(ir, 20) ? "u" : "", rda, rs1a);
            break;
        case 0b11010: // FCVT.F.I
            switch ((BIT(ir, 25) << 2) | BITS(ir, 20, 21))
            {
            case 0b000: // FCVT.S.W
                sd = (float)(int32_t)BITS(rs1, 0, 31), BNAN(&dd);
                break;
            case 0b001: // FCVT.S.WU
                sd = (float)BITS(rs1, 0, 31), BNAN(&dd);
                break;
            case 0b010: // FCVT.S.L
                sd = (float)(int64_t)rs1, BNAN(&dd);
                break;
            case 0b011: // FCVT.S.LU
                sd = (float)rs1, BNAN(&dd);
                break;
            case 0b100: // FCVT.D.W
                dd = (double)(int32_t)BITS(rs1, 0, 31);
                break;
            case 0b101: // FCVT.D.W
                dd = (double)BITS(rs1, 0, 31);
                break;
            case 0b110: // FCVT.D.L
                dd = (double)(int64_t)rs1;
                break;
            case 0b111: // FCVT.D.LU
                dd = (double)rs1;
                break;
            }
            sprintf(asmcode, "fcvt.%c.%c%s f%d, x%d",
                    BIT(ir, 25) ? 'd' : 's', BIT(ir, 21) ? 'l' : 'w',
                    BIT(ir, 20) ? "u" : "", rda, rs1a);
            break;
        case 0b11100: // FMV.X.F
            if (funct3 == 0)
            {
                BIT(ir, 25) ? (rd = *(uint64_t *)&ds1) : (rd = *(uint32_t *)&ds1, BNAN(&rd));
                sprintf(asmcode, "fmv.x.%c x%d, f%d", BIT(ir, 25) ? 'd' : 'w', rda, rs1a);
            }
            else if (funct3 == 1) // FCLASS
            {
                using namespace std;
                if (BIT(ir, 25) ? isinf(ds1) && ds1 < 0 : isinf(ss1) && ss1 < 0)
                    rd = 1;
                else if (BIT(ir, 25) ? isnormal(ds1) && ds1 < 0 : isnormal(ss1) && ss1 < 0)
                    rd = 2;
                else if (BIT(ir, 25) ? issubnormal(ds1) && ds1 < 0 : issubnormal(ss1) && ss1 < 0)
                    rd = 4;
                else if (BIT(ir, 25) ? iszero(ds1) && ds1 < 0 : iszero(ss1) && ss1 < 0)
                    rd = 8;
                else if (BIT(ir, 25) ? iszero(ds1) && ds1 > 0 : iszero(ss1) && ss1 > 0)
                    rd = 16;
                else if (BIT(ir, 25) ? issubnormal(ds1) && ds1 > 0 : issubnormal(ss1) && ss1 > 0)
                    rd = 32;
                else if (BIT(ir, 25) ? isnormal(ds1) && ds1 > 0 : isnormal(ss1) && ss1 > 0)
                    rd = 64;
                else if (BIT(ir, 25) ? isinf(ds1) && ds1 > 0 : isinf(ss1) && ss1 > 0)
                    rd = 128;
                else if (BIT(ir, 25) ? isnan(ds1) && issignaling(ds1) : isnan(ss1) && issignaling(ss1))
                    rd = 256;
                else if (BIT(ir, 25) ? isnan(ds1) && !issignaling(ds1) : isnan(ss1) && !issignaling(ss1))
                    rd = 512;
                sprintf(asmcode, "fclass.%c x%d, f%d", BIT(ir, 25) ? 'd' : 'w', rda, rs1a);
            }
            break;
        case 0b11110: // FMV.F.X
            BIT(ir, 25) ? (*(uint64_t *)&dd = rs1) : (*(uint32_t *)&sd = rs1, BNAN(&dd));
            sprintf(asmcode, "fmv.%c.x f%d, x%d", BIT(ir, 25) ? 'd' : 'w', rda, rs1a);
            break;
        }
        break;
    case 0b1100011: // BRANCH
        imm = (BIT(ir, 31) << 12) | (BIT(ir, 7) << 11) |
              (BITS(ir, 25, 30) << 5) | (BITS(ir, 8, 11) << 1);
        imm = SEXT(imm, 13);
        static const char *bnames[] = {"beq", "bne", "", "", "blt", "bge", "bltu", "bgeu"};
        if (funct3 == 0b000 && rs1 == rs2 ||                   // BEQ
            funct3 == 0b001 && rs1 != rs2 ||                   // BNE
            funct3 == 0b100 && (int64_t)rs1 < (int64_t)rs2 ||  // BLT
            funct3 == 0b101 && (int64_t)rs1 >= (int64_t)rs2 || // BGE
            funct3 == 0b110 && rs1 < rs2 ||                    // BLTU
            funct3 == 0b111 && rs1 >= rs2)                     // BGEU
            jump = 1, npc = pc + imm;
        if (funct3 != 2 && funct3 != 3)
            sprintf(asmcode, "%s x%d, x%d, %ld(pc)", bnames[funct3], rs1a, rs2a, imm);
        break;
    case 0b1100111: // JALR
        imm = SEXT(BITS(ir, 20, 31), 12);
        rd = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
        npc = rs1 + imm;
        jump = 1;
        sprintf(asmcode, "jalr x%d, %ld(x%d)", rda, imm, rs1a);
        break;
    case 0b1101111: // JAL
        imm = (BIT(ir, 31) << 20) | (BITS(ir, 12, 19) << 12) |
              (BIT(ir, 20) << 11) | (BITS(ir, 21, 30) << 1);
        imm = SEXT(imm, 21);
        rd = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
        npc = pc + imm;
        jump = 1;
        sprintf(asmcode, "jal x%d, %ld(pc)", rda, imm);
        break;
    case 0b1110011: // SYSTEM
        static char csrname[4] = {0, 'w', 's', 'c'};
        if (BITS(ir, 12, 13))
        {
            csraddr = BITS(ir, 20, 31);
            uint64_t rs1val = BIT(ir, 14) ? rs1a : rs1;
            if (BITS(ir, 12, 13) == 1)
                rd = csr[csraddr].val, csr[csraddr].val = rs1val;
            else if (BITS(ir, 12, 13) == 2)
                rd = csr[csraddr].val, csr[csraddr].val |= rs1val;
            else if (BITS(ir, 12, 13) == 3)
                rd = csr[csraddr].val, csr[csraddr].val &= ~rs1val;
            csrdata = csr[csraddr].val;
            sprintf(asmcode, "csrr%c%s x%d, %s, %s%d",
                    csrname[BITS(ir, 12, 13)], BIT(ir, 14) ? "i" : "",
                    rda, get_csrname(csraddr), BIT(ir, 14) ? "" : "x", rs1a);
        }
        else if ((ir & ~(1 << 20)) == 0x73)
        {
            csr[0x341].val = pc;
            csr[0x342].val = BIT(ir, 20) ? 3 : 11;
            excp = 1;
            sprintf(asmcode, BIT(ir, 20) ? "ebreak" : "ecall");
        }
        break;
    }
    if (!jump | nojump)
        npc = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
    if (excp)
        npc = csr[0x305].val, step();
    arregs[0] = 0;

    // statistics
    if (instfreq.find(pc) == instfreq.end())
        instfreq[pc] = 0;
    instfreq[pc]++;
}

/**
 * @brief check values
 * @param pc program counter
 * @param rda destination register address
 * @param rd destination register value
 * @param mwaddr memory write address
 * @param mwdata memory write data
 * @param mwwidth memory write width
 * @return if consistent with above arguments
 */
int simulator::check(uint64_t pc, uint64_t rda, uint64_t rd,
                     uint64_t mwaddr, uint64_t mwdata, uint8_t mwwidth,
                     uint64_t csraddr, uint64_t csrdata)
{
    if (mwwidth < 8)
        mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
    return this->pc == pc && (rda == 0 || this->arregs[rda] == rd) &&
           this->csr[csraddr].val == csrdata &&
           (!this->mwwidth || this->mwaddr == mwaddr && this->mwdata == mwdata &&
                                  this->mwwidth == mwwidth);
}

const uint64_t *simulator::get_arreg() { return arregs; }
uint64_t simulator::get_pc() { return pc; }
uint64_t simulator::get_ir() { return ir; }
std::map<uint64_t, uint8_t> &simulator::get_mem() { return memory; }
uint64_t simulator::get_mwaddr() { return mwaddr; }
uint64_t simulator::get_mwdata() { return mwdata; }
uint8_t simulator::get_mwwidth() { return mwwidth; }
const char *simulator::get_asmcode() { return asmcode; }
const char *simulator::get_csrname(uint64_t addr)
{
    static char defname[32] = {0};
    sprintf(defname, "(csr0x%lx)", addr);
    if (csr[addr].name)
        return csr[addr].name;
    return defname;
}
const std::map<uint64_t, int> &simulator::get_freq() { return instfreq; }
uint64_t simulator::get_csraddr() { return csraddr; }
uint64_t simulator::get_csrdata() { return csrdata; }
