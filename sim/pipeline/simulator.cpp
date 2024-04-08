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
#define SEXT(dw, width) ((BIT(dw, width - 1) ? (uint64_t)-1 << (width) : 0) | (dw) & ~((uint64_t)-1 << (width)))
#define CNAN64(x) (BITS(x, 52, 62) == 0x7ff && BITS(x, 0, 51) ? 0x7ff8'0000'0000'0000ul : (x))
#define CNAN32(x) (BITS(x, 23, 30) == 0x0ff && BITS(x, 0, 22) ? 0xffff'ffff'7fc0'0000ul : (x))
#define BNAN(p) *((uint32_t *)(p) + 1) = 0xffff'ffffu

/**
 * @brief simulator constructor
 * @param initmem initial memory
 */
simulator::simulator(const uint64_t &initpc, const std::map<uint64_t, uint8_t> &initmem)
{
    npc = initpc;
    memory = initmem;
    simtime = 0;
}

/**
 * @brief step with an instruction
 */
void simulator::step(int nojump)
{
    simtime++;
    // instruction fetch
    pc = npc;
    uint32_t idata = DLE(memory, pc);

    // compressed instruction extension
    switch ((BITS(idata, 13, 15) << 2) | BITS(idata, 0, 1))
    {
    case 0b00000:
        break;
    case 0b00100:
        break;
    case 0b01000:
        break;
    case 0b01100:
        break;
    case 0b10100:
        break;
    case 0b11000:
        break;
    case 0b11100:
        break;
    case 0b00001:
        break;
    case 0b00101:
        break;
    case 0b01001:
        break;
    case 0b01101:
        break;
    case 0b10001:
        break;
    case 0b10101:
        break;
    case 0b11001:
        break;
    case 0b11101:
        break;
    case 0b00010:
        break;
    case 0b00110:
        break;
    case 0b01010:
        break;
    case 0b01110:
        break;
    case 0b10010:
        break;
    case 0b10110:
        break;
    case 0b11010:
        break;
    case 0b11110:
        break;
    default:
        ir = idata;
        break;
    }

    // decode and execution
    asmcode[0] = 0;
    mwwidth = 0;
    uint8_t funct3 = BITS(ir, 12, 14), jump = 0;
    uint8_t rda = BITS(ir, 7, 11), rs1a = BITS(ir, 15, 19),
            rs2a = BITS(ir, 20, 24), rs3a = BITS(ir, 27, 31);
    uint64_t &rd = arregs[rda], rs1 = arregs[rs1a],
             rs2 = arregs[rs2a], rs3 = arregs[rs3a];
    double &dd = *(double *)&arregs[rda + 32], ds1 = *(double *)&arregs[rs1a + 32],
           ds2 = *(double *)&arregs[rs2a + 32], ds3 = *(double *)&arregs[rs3a + 32];
    float &sd = *(float *)&arregs[rda + 32], ss1 = *(float *)&arregs[rs1a + 32],
          ss2 = *(float *)&arregs[rs2a + 32], ss3 = *(float *)&arregs[rs3a + 32];
    if (BITS(ds1, 32, 63) != 0xffff'ffffu) // NaN-boxing
        ss1 = CNAN32(ss1);
    if (BITS(ds2, 32, 63) != 0xffff'ffffu)
        ss2 = CNAN32(ss2);
    int64_t imm;
    switch (BITS(ir, 2, 6)) // opcode
    {
    case 0b00000: // LOAD
        imm = SEXT(BITS(ir, 20, 31), 12);
        rd = DLE(memory, rs1 + imm);
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
    case 0b00001: // LOAD-FP
        break;
    case 0b00011: // MISC-MEM
        break;
    case 0b00100: // OP-IMM
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
    case 0b00101: // AUIPC
        rd = pc + SEXT(BITS(ir, 12, 31) << 12, 32);
        sprintf(asmcode, "auipc x%d, 0x%lx", rda, BITS(ir, 12, 31));
        break;
    case 0b00110: // OP-IMM-32
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
    case 0b01000: // STORE
        imm = SEXT(BITS(ir, 25, 31) << 5 | BITS(ir, 7, 11), 12);
        mwaddr = rs1 + imm;
        mwdata = rs2;
        mwwidth = 1 << BITS(ir, 12, 13);
        if (mwwidth < 8)
            mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
        for (int i = 0; i < mwwidth; i++)
            memory[mwaddr + i] = DTOB(mwdata, i);
        static const char *snames[] = {"sb", "sh", "sw", "sd"};
        if (funct3 < 4)
            sprintf(asmcode, "%s x%d, %ld(x%d)", snames[funct3], rs2a, imm, rs1a);
        break;
    case 0b01001: // STORE-FP
        break;
    case 0b01011: // AMO
        break;
    case 0b01100: // OP
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
            uint64_t h = ((a[0] * b[1] + a[1] * b[0]) >> 32) +
                         (a[0] * b[2] + a[1] * b[1] + a[2] * b[0]) +
                         ((a[0] * b[3] + a[1] * b[2] + a[2] * b[1] + a[3] * b[0]) << 32);
            if (funct3 == 0b000) // MUL
                rd = l;
            else if (funct3 <= 0b011) // MULH[[S]U]
                rd = h;
            else if (funct3 == 0b100) // DIV
                rd = (int64_t)rs1 / (int64_t)rs2;
            else if (funct3 == 0b101) // DIVU
                rd = rs1 / rs2;
            else if (funct3 == 0b110) // REM
                rd = (int64_t)rs1 % (int64_t)rs2;
            else if (funct3 == 0b111) // REMU
                rd = rs1 % rs2;
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
    case 0b01101: // LUI
        rd = SEXT(BITS(ir, 12, 31) << 12, 32);
        sprintf(asmcode, "lui x%d, 0x%lx", rda, BITS(ir, 12, 31));
        break;
    case 0b01110: // OP-32
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
                rd = SEXT((int32_t)rs1 / (int32_t)rs2, 32);
            else if (funct3 == 0b101) // DIVUW
                rd = SEXT((uint32_t)rs1 / (uint32_t)rs2, 32);
            else if (funct3 == 0b110) // REMW
                rd = SEXT((int32_t)rs1 % (int32_t)rs2, 32);
            else if (funct3 == 0b111) // REMUW
                rd = SEXT((uint32_t)rs1 % (uint32_t)rs2, 32);
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
    case 0b10000: // MADD
        break;
    case 0b10001: // MSUB
        break;
    case 0b10010: // NMSUB
        break;
    case 0b10011: // NMADD
        break;
    case 0b10100: // OP-FP
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
    case 0b11000: // BRANCH
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
    case 0b11001: // JALR
        imm = SEXT(BITS(ir, 20, 31), 12);
        rd = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
        npc = rs1 + imm;
        jump = 1;
        sprintf(asmcode, "jalr x%d, %ld(x%d)", rda, imm, rs1a);
        break;
    case 0b11011: // JAL
        imm = (BIT(ir, 31) << 20) | (BITS(ir, 12, 19) << 12) |
              (BIT(ir, 20) << 11) | (BITS(ir, 21, 30) << 1);
        imm = SEXT(imm, 21);
        rd = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
        npc = pc + imm;
        jump = 1;
        sprintf(asmcode, "jal x%d, %ld(pc)", rda, imm);
        break;
    case 0b11100: // SYSTEM
        break;
    }
    if (!jump | nojump)
        npc = pc + (BITS(idata, 0, 1) == 3 ? 4 : 2);
    arregs[0] = 0;
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
                     uint64_t mwaddr, uint64_t mwdata, uint8_t mwwidth)
{
    if (mwwidth < 8)
        mwdata &= ~((uint64_t)-1 << (8 * mwwidth));
    return this->pc == pc && (rda == 0 || this->arregs[rda] == rd) &&
           (!this->mwwidth || this->mwaddr == mwaddr && this->mwdata == mwdata &&
                                  this->mwwidth == mwwidth);
}

/**
 * @brief get architectural register values
 */
const uint64_t *simulator::get_arreg() { return arregs; }

/**
 * @brief get program counter
 */
uint64_t simulator::get_pc() { return pc; }

/**
 * @brief get instruction register value
 */
uint64_t simulator::get_ir() { return ir; }

/**
 * @brief get reference of memory
 */
std::map<uint64_t, uint8_t> &simulator::get_mem() { return memory; }

/**
 * @brief get memory write address
 */
uint64_t simulator::get_mwaddr() { return mwaddr; }

/**
 * @brief get memory write data
 */
uint64_t simulator::get_mwdata() { return mwdata; }

/**
 * @brief get memory write width
 */
uint8_t simulator::get_mwwidth() { return mwwidth; }

/**
 * @brief get assembly code
 */
char *simulator::get_asmcode() { return asmcode; }
