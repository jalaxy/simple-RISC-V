#include "state.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <fenv.h>

bits::bits() { this->data = 0; }
bits::bits(uint64_t ui) { this->data = ui; }
bits::bits(double d)
{
    double data = d;
    this->data = *(uint64_t *)&data;
    if (range(52, 62) == 0x7ff && range(0, 51))
        this->data = 0x7ff8'0000'0000'0000ul;
}
bits::bits(float f)
{
    float data = f;
    this->data = *(uint64_t *)&data | 0xffff'ffff'0000'0000ull;
    if (range(23, 30) == 0x0ff && range(0, 22))
        this->data = 0xffff'ffff'7fc0'0000ul;
}
bits::operator uint64_t() const { return data; }
bits::operator double() const { return *(double *)&data; }
bits::operator float() const { return *(float *)&data; }
uint8_t bits::operator[](uint8_t i) const { return (data >> i) & 1; }
uint64_t bits::operator++(int) { return data++; }
bits &bits::operator>>=(int x) { return this->data >>= x, *this; }
bits &bits::operator|=(uint64_t x) { return this->data |= x, *this; }
bits bits::range(uint8_t s, uint8_t e) const { return data << 63 - e >> 63 - e + s; }
int64_t bits::sext(int w) const { return ((data >> w - 1) & 1 ? -1ull << w : 0) | data & ~(-1ull << w); }
void bits::write(uint8_t s, uint8_t e, uint64_t x) { (data &= ~((1ull << e - s + 1ull) - 1ull << s)) |= x << s; }
void bits::write(uint8_t i, uint64_t x) { (data &= ~(1 << i)) |= x << i; }

memory::memory() {}
memory::memory(const memory &b) { *this = b; }
memory::~memory()
{
    for (auto p : ptr)
        delete[] p;
}
bool memory::add(uint64_t size, uint64_t base)
{
    size = (size + base % 4096 + 4095) / 4096 * 4096;
    base = base / 4096 * 4096;
    for (int i = 0; i < this->size.size(); i++)
        if (this->base[i] <= base && this->base[i] + this->size[i] >= base + size)
            return true;
        else if (this->base[i] <= base && this->base[i] + this->size[i] >= base)
            size += base - this->base[i], base = this->base[i];
        else if (this->base[i] <= base + size && this->base[i] + this->size[i] >= base + size)
            size += this->base[i] + this->size[i] - base - size;
    this->base.push_back(base);
    this->size.push_back(size);
    this->ptr.push_back(new (std::nothrow) uint8_t[size]);
    if (this->ptr.back() == 0)
        return this->size.back() = 0, false;
    for (int i = 0; i < this->size.size() - 1; i++)
        if (this->base[i] >= base && this->base[i] + this->size[i] <= base + size)
        {
            memcpy(this->ptr.back() + this->base[i] - base, this->ptr[i], this->size[i]);
            delete[] this->ptr[i];
            this->base.erase(this->base.begin() + i);
            this->size.erase(this->size.begin() + i);
            this->ptr.erase(this->ptr.begin() + i);
            i--;
        }
    return true;
}
bool memory::copy(uint8_t *ptr, uint64_t size, uint64_t base)
{
    if (!add(size, base))
        return false;
    memcpy(&(*this)[base], ptr, size);
    return true;
}
bool memory::read(FILE *fp, uint64_t size, uint64_t base)
{
    if (!add(size, base))
        return false;
    if (fread(&(*this)[base], size, 1, fp) < 0)
        return false;
    return true;
}
uint8_t &memory::operator[](uint64_t addr)
{
    for (int i = 0; i < size.size(); i++)
        if (base[i] <= addr && addr < base[i] + size[i])
            return *(ptr[i] + addr - base[i]);
    return segfault;
}
memory &memory::operator=(const memory &b)
{
    for (int i = 0; i < b.size.size(); i++)
        this->copy(b.ptr[i], b.size[i], b.base[i]);
    return *this;
}
uint8_t &memory::ui8(uint64_t addr) { return (*this)[addr]; }
uint16_t &memory::ui16(uint64_t addr) { return *(uint16_t *)&(*this)[addr]; }
uint32_t &memory::ui32(uint64_t addr) { return *(uint32_t *)&(*this)[addr]; }
uint64_t &memory::ui64(uint64_t addr) { return *(uint64_t *)&(*this)[addr]; }
bool memory::issegfault(const uint8_t &x) { return &x == &segfault; }

/**
 * @brief address translation
 * @param mem   pointer of memory
 * @param satp  CSR satp
 * @param vaddr virtual address
 * @param perm  the permission required ([7..0]: ----XWR-, [9,8]: SU)
 * @param adpf  raise page fault when violate access/dirty bits
 * @retval physical adderss
 */
inline uint64_t paddr(memory &mem, bits satp, bits vaddr, bits perm, bool adpf)
{
    uint64_t ppn, vpn[5], offset;
    int start = -1;
    ppn = satp.range(0, 43);
    offset = vaddr & 0xfffllu;
    if (satp >> 60 == 0) // bare
        return vaddr;
    offset = vaddr.range(0, 11);
    vpn[0] = vaddr.range(12, 20);
    vpn[1] = vaddr.range(21, 29);
    vpn[2] = vaddr.range(30, 38);
    vpn[3] = vaddr.range(39, 47);
    vpn[4] = vaddr.range(48, 56);
    if (satp >> 60 == 8) // sv39
        start = 2;
    else if (satp >> 60 == 9) // sv48
        start = 3;
    else if (satp >> 60 == 10) // sv57
        start = 4;
    for (int i = start; i >= 0; i--)
    {
        bits &pte = *(bits *)&mem.ui64((ppn << 12) + (vpn[i] << 3));
        ppn = pte.range(10, 53);
        if (pte[0] == 0) // invalid entry
            return -1;
        if (pte[1] || pte[3]) // pte.r = 1 or pte.x = 1 -> leaf node
        {
            if ((pte.range(0, 7) | perm.range(0, 7)) != pte.range(0, 7)) // check permission
                return -1;
            if (perm[8] && !pte[4] || perm[9] && pte[4]) // U bit
                return -1;
            if (!adpf) // A/D bit
            {
                pte.write(6, 1); // A bit
                if (perm[2])
                    pte.write(7, 1); // D bit
            }
            else if (!pte[6] || perm[2] && !pte[7])
                return -1;
            if (i > 0 && bits(ppn).range(0, i * 9 - 1)) // misaligned super page
                return -1;
            for (int j = 0; j < i; j++)
                ppn |= vpn[j] << (j * 9); // super page
            break;
        }
    }
    return (ppn << 12) | offset;
}

/**
 * @brief disassemble an instruction
 * @param ir instruction register value
 * @retval disassembled string
 */
std::string disas(uint32_t ir)
{
    char ret[128] = {0};
    bits inst = ir;
    sprintf(ret, "unimp 0x%04x", ir);
    switch ((inst.range(13, 15) << 2) | inst.range(0, 1))
    {
    case 0b00000:
        if (inst.range(5, 12)) // C.ADDI4SPN
            sprintf(ret, "c.addi4spn %s, %lu", gprname[inst.range(2, 4) + 8],
                    (inst.range(7, 10) << 6) | (inst.range(11, 12) << 4) | (inst[5] << 3) | (inst[6] << 2));
        return ret;
    case 0b00100: // C.FLD
        sprintf(ret, "c.fld %s, %lu(%s)", gprname[inst.range(2, 4) + 8 + 32],
                (inst.range(5, 6) << 6) | (inst.range(10, 12) << 3), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b01000: // C.LW
        sprintf(ret, "c.lw %s, %lu(%s)", gprname[inst.range(2, 4) + 8],
                (inst[5] << 6) | (inst.range(10, 12) << 3) | (inst[6] << 2), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b01100: // C.LD
        sprintf(ret, "c.ld %s, %lu(%s)", gprname[inst.range(2, 4) + 8],
                (inst.range(5, 6) << 6) | (inst.range(10, 12) << 3), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b10100: // C.FSD
        sprintf(ret, "c.fsd %s, %lu(%s)", gprname[inst.range(2, 4) + 8 + 32],
                (inst.range(5, 6) << 6) | (inst.range(10, 12) << 3), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b11000: // C.SW
        sprintf(ret, "c.sw %s, %lu(%s)", gprname[inst.range(2, 4) + 8],
                (inst[5] << 6) | (inst.range(10, 12) << 3) | (inst[6] << 2), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b11100: // C.SD
        sprintf(ret, "c.sd %s, %lu(%s)", gprname[inst.range(2, 4) + 8],
                (inst.range(5, 6) << 6) | (inst.range(10, 12) << 3), gprname[inst.range(7, 9) + 8]);
        return ret;
    case 0b00001: // C.ADDI / C.NOP
        sprintf(ret, "c.nop");
        if ((inst[12] << 5) || inst.range(2, 6))
            sprintf(ret, "c.addi %s, %ld", gprname[inst.range(7, 11)],
                    bits((inst[12] << 5) | inst.range(2, 6)).sext(6));
        return ret;
    case 0b00101: // ADDIW
        sprintf(ret, "c.addiw %s, %ld", gprname[inst.range(7, 11)],
                bits((inst[12] << 5) | inst.range(2, 6)).sext(6));
        return ret;
    case 0b01001: // C.LI
        sprintf(ret, "c.li %s, %ld", gprname[inst.range(7, 11)],
                bits((inst[12] << 5) | inst.range(2, 6)).sext(6));
        return ret;
    case 0b01101:
        if (inst.range(7, 11) == 2) // C.ADDI16SP
            sprintf(ret, "c.addi16sp sp, %ld",
                    bits((inst[12] << 9) | (inst.range(3, 4) << 7) |
                         (inst[5] << 6) | (inst[2] << 5) | (inst[6] << 4))
                        .sext(10));
        else if (inst.range(2, 6) || inst[12]) // C.LUI
            sprintf(ret, "c.lui %s, %ld", gprname[inst.range(7, 11)],
                    bits((inst[12] << 5) | inst.range(2, 6)).sext(6));
        return ret;
    case 0b10001:
        static const char *coname[] = {"sub", "xor", "or", "and", "subw", "addw"};
        if (inst[11] == 0) // C.SRLI / C.SRAI
            sprintf(ret, "c.sr%ci %s, %lu", inst[10] ? 'a' : 'l',
                    gprname[inst.range(7, 9) + 8], (inst[12] << 5) | inst.range(2, 6));
        else if (inst[10] == 0) // C.ANDI
            sprintf(ret, "c.andi %s, %ld", gprname[inst.range(7, 9) + 8],
                    bits((inst[12] << 5) | inst.range(2, 6)).sext(6));
        else if ((inst[12] << 2) | inst.range(5, 6) < 6)
            sprintf(ret, "c.%s %s, %s", coname[(inst[12] << 2) | inst.range(5, 6)],
                    gprname[inst.range(7, 9) + 8], gprname[inst.range(2, 4) + 8]);
        return ret;
    case 0b10101: // C.J
        sprintf(ret, "c.j %ld(pc)",
                bits((inst[12] << 11) | (inst[8] << 10) | (inst.range(9, 10) << 8) | (inst[6] << 7) |
                     (inst[7] << 6) | (inst[2] << 5) | (inst[11] << 4) | inst.range(3, 5) << 1)
                    .sext(12));
        return ret;
    case 0b11001: // C.BEQZ
    case 0b11101: // C.BNEZ
        sprintf(ret, "c.b%sz %s, %ld(pc)", inst[13] ? "ne" : "eq", gprname[inst.range(7, 9) + 8],
                bits((inst[12] << 8) | (inst.range(5, 6) << 6) | (inst[2] << 5) |
                     (inst.range(10, 11) << 3) | (inst.range(3, 4) << 1))
                    .sext(9));
        return ret;
    case 0b00010:
        if (inst[12] || inst.range(2, 6)) // C.SLLI
            sprintf(ret, "c.slli %s, %lu", gprname[inst.range(7, 11)], (inst[12] << 5) | inst.range(2, 6));
        return ret;
    case 0b00110: // C.FLDSP
        sprintf(ret, "c.fldsp %s, %lu(sp)", gprname[inst.range(7, 11) + 32],
                (inst.range(2, 4) << 6) | (inst[12] << 5) | (inst.range(5, 6) << 3));
        return ret;
    case 0b01010: // C.LWSP
        sprintf(ret, "c.lwsp %s, %lu(sp)", gprname[inst.range(7, 11)],
                (inst.range(2, 3) << 6) | (inst[12] << 5) | (inst.range(4, 6) << 2));
        return ret;
    case 0b01110: // C.LDSP
        sprintf(ret, "c.ldsp %s, %lu(sp)", gprname[inst.range(7, 11)],
                (inst.range(2, 4) << 6) | (inst[12] << 5) | (inst.range(5, 6) << 3));
        return ret;
    case 0b10010:
        if (!inst[12] && !inst.range(2, 6) && inst.range(7, 11)) // C.JR
            sprintf(ret, "c.jr %s", gprname[inst.range(7, 11)]);
        if (!inst[12] && inst.range(2, 6)) // C.MV
            sprintf(ret, "c.mv %s, %s", gprname[inst.range(7, 11)], gprname[inst.range(2, 6)]);
        if (inst[12] && !inst.range(2, 6) && inst.range(7, 11) == 0) // C.EBREAK
            sprintf(ret, "c.ebreak");
        if (inst[12] && !inst.range(2, 6) && inst.range(7, 11)) // C.JALR
            sprintf(ret, "c.jalr %s", gprname[inst.range(7, 11)]);
        if (inst[12] && inst.range(2, 6)) // C.ADD
            sprintf(ret, "c.add %s, %s", gprname[inst.range(7, 11)], gprname[inst.range(2, 6)]);
        return ret;
    case 0b10110: // C.FSDSP
        sprintf(ret, "c.fsdsp %s, %lu(sp)", gprname[inst.range(2, 6) + 32],
                (inst.range(7, 9) << 6) | (inst.range(10, 12) << 3));
        return ret;
    case 0b11010: // C.SWSP
        sprintf(ret, "c.swsp %s, %lu(sp)", gprname[inst.range(2, 6)],
                (inst.range(7, 8) << 6) | (inst.range(9, 12) << 2));
        return ret;
    case 0b11110: // C.SDSP
        sprintf(ret, "c.sdsp %s, %lu(sp)", gprname[inst.range(2, 6)],
                (inst.range(7, 9) << 6) | (inst.range(10, 12) << 3));
        return ret;
    }
    sprintf(ret, "unimp 0x%08x", ir);
    uint8_t funct3 = inst.range(12, 14);
    const char *xd = gprname[inst.range(7, 11)], *xs1 = gprname[inst.range(15, 19)],
               *xs2 = gprname[inst.range(20, 24)];
    const char *fd = gprname[inst.range(7, 11) + 32], *fs1 = gprname[inst.range(15, 19) + 32],
               *fs2 = gprname[inst.range(20, 24) + 32], *fs3 = gprname[inst.range(27, 31) + 32];
    int64_t imm;
    switch (inst.range(0, 6)) // opcode
    {
    case 0b0000011: // LOAD
        imm = inst.range(20, 31).sext(12);
        static const char *lnames[] = {"lb", "lh", "lw", "ld", "lbu", "lhu", "lwu", ""};
        if (funct3 != 7)
            sprintf(ret, "%s %s, %ld(%s)", lnames[funct3], xd, imm, xs1);
        break;
    case 0b0000111: // LOAD-FP
        imm = inst.range(20, 31).sext(12);
        if (funct3 == 2 || funct3 == 3)
            sprintf(ret, "f%s %s, %ld(%s)", lnames[funct3], fd, imm, xs1);
        break;
    case 0b0001111: // MISC-MEM
        switch (funct3)
        {
        case 0b000: // FENCE
            if (inst[31])
                sprintf(ret, "fence.tso");
            else if ((inst[27] | inst[26] | inst[25] | inst[24]) &
                     (inst[23] | inst[22] | inst[21] | inst[20]))
                sprintf(ret, "fence.%s%s%s%s.%s%s%s%s",
                        inst[27] ? "i" : "", inst[26] ? "o" : "",
                        inst[25] ? "r" : "", inst[24] ? "w" : "",
                        inst[23] ? "i" : "", inst[22] ? "o" : "",
                        inst[21] ? "r" : "", inst[20] ? "w" : "");
            break;
        case 0b001: // FENCE.I
            sprintf(ret, "fence.i");
            break;
        }
        break;
    case 0b0010011: // OP-IMM
        imm = inst.range(20, 31).sext(12);
        static const char *iname[] = {
            "addi", "slli", "slti", "sltiu", "xori", "srli", "ori", "andi"};
        if (ir == 0x13)
            sprintf(ret, "nop");
        else
            sprintf(ret, "%s %s, %s, %ld",
                    funct3 == 5 && inst[30] ? "srai" : iname[funct3], xd, xs1, imm);
        break;
    case 0b0010111: // AUIPC
        sprintf(ret, "auipc %s, 0x%lx", xd, (int64_t)inst.range(12, 31));
        break;
    case 0b0011011: // OP-IMM-32
        imm = inst.range(20, 31).sext(12);
        if (funct3 == 0 || funct3 == 1 || funct3 == 5)
            sprintf(ret, "%sw %s, %s, %ld",
                    funct3 == 5 && inst[30] ? "srai" : iname[funct3], xd, xs1, imm);
        break;
    case 0b0100011: // STORE
    case 0b0100111: // STORE-FP
        imm = bits(inst.range(25, 31) << 5 | inst.range(7, 11)).sext(12);
        static const char *snames[] = {"sb", "sh", "sw", "sd"};
        if (funct3 < 4)
            sprintf(ret, "%s%s %s, %ld(%s)",
                    inst[2] ? "f" : "", snames[funct3], inst[2] ? fs2 : xs2, imm, xs1);
        break;
    case 0b0101111: // AMO
        switch (inst.range(27, 31))
        {
        case 0b00010:
            sprintf(ret, "lr.%c%s%s %s, (%s)", funct3 == 0b010 ? 'w' : 'd',
                    inst[26] ? ".aq" : "", inst[25] ? ".rl" : "", xd, xs1);
            break;
        case 0b00011:
            sprintf(ret, "sc.%c%s%s %s, %s, (%s)", funct3 == 0b010 ? 'w' : 'd',
                    inst[26] ? ".aq" : "", inst[25] ? ".rl" : "", xd, xs2, xs1);
            break;
        default:
            static const char *aname;
            aname = NULL;
            if (inst.range(27, 31) == 0b00001)
                aname = "swap";
            if (inst.range(27, 31) == 0b00000)
                aname = "add";
            if (inst.range(27, 31) == 0b00100)
                aname = "xor";
            if (inst.range(27, 31) == 0b01100)
                aname = "and";
            if (inst.range(27, 31) == 0b01000)
                aname = "or";
            if (inst.range(27, 31) == 0b10000)
                aname = "min";
            if (inst.range(27, 31) == 0b10100)
                aname = "max";
            if (inst.range(27, 31) == 0b11000)
                aname = "minu";
            if (inst.range(27, 31) == 0b11100)
                aname = "maxu";
            if (aname == NULL)
                break;
            sprintf(ret, "amo%s.%c%s%s %s, %s, (%s)", aname, funct3 == 0b010 ? 'w' : 'd',
                    inst[26] ? ".aq" : "", inst[25] ? ".rl" : "", xd, xs2, xs1);
            break;
        }
        break;
    case 0b0110011: // OP
        static const char *oname[] = {
            "add", "sll", "slt", "sltu", "xor", "srl", "or", "and"};
        static const char *mname[] = {
            "mul", "mulh", "mulhsu", "mulhu", "div", "divu", "rem", "remu"};
        if (funct3 == 0 && inst[30])
            sprintf(ret, "sub %s, %s, %s", xd, xs1, xs2);
        else if (funct3 == 5 && inst[30])
            sprintf(ret, "sra %s, %s, %s", xd, xs1, xs2);
        else
            sprintf(ret, "%s %s, %s, %s", oname[funct3], xd, xs1, xs2);
        if (inst[25])
            sprintf(ret, "%s %s, %s, %s", mname[funct3], xd, xs1, xs2);
        break;
    case 0b0110111: // LUI
        sprintf(ret, "lui %s, 0x%lx", xd, (int64_t)inst.range(12, 31));
        break;
    case 0b0111011: // OP-32
        static const char *mwname[] = {
            "mulw", 0, 0, 0, "divw", "divuw", "remw", "remuw"};
        if (funct3 == 0 && inst[30])
            sprintf(ret, "subw %s, %s, %s", xd, xs1, xs2);
        else if (funct3 == 5 && inst[30])
            sprintf(ret, "sraw %s, %s, %s", xd, xs1, xs2);
        else if (funct3 == 0 || funct3 == 1 || funct3 == 5)
            sprintf(ret, "%sw %s, %s, %s", oname[funct3], xd, xs1, xs2);
        if (inst[25])
            if (funct3 != 1 && funct3 != 2 && funct3 != 3)
                sprintf(ret, "%s %s, %s, %s", mwname[funct3], xd, xs1, xs2);
        break;
    case 0b1000011: // MADD
        sprintf(ret, "fmadd.%c %s, %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2, fs3);
        break;
    case 0b1000111: // MSUB
        sprintf(ret, "fmsub.%c %s, %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2, fs3);
        break;
    case 0b1001011: // NMSUB
        sprintf(ret, "fnmsub.%c %s, %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2, fs3);
        break;
    case 0b1001111: // NMADD
        sprintf(ret, "fnmadd.%c %s, %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2, fs3);
        break;
    case 0b1010011: // OP-FP
        switch (inst.range(27, 31))
        {
        case 0b00000:
            sprintf(ret, "fadd.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b00001:
            sprintf(ret, "fsub.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b00010:
            sprintf(ret, "fmul.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b00011:
            sprintf(ret, "fdiv.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b01011:
            sprintf(ret, "fsqrt.%c %s, %s", inst[25] ? 'd' : 's', fd, fs1);
            break;
        case 0b00100:
            if (funct3 == 0)
                sprintf(ret, "fsgnj.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            else if (funct3 == 1)
                sprintf(ret, "fsgnjn.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            else if (funct3 == 2)
                sprintf(ret, "fsgnjx.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b00101:
            if (funct3 == 0)
                sprintf(ret, "fmin.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            else if (funct3 == 1)
                sprintf(ret, "fmax.%c %s, %s, %s", inst[25] ? 'd' : 's', fd, fs1, fs2);
            break;
        case 0b10100:
            if (funct3 == 0)
                sprintf(ret, "fle.%c %s, %s, %s", inst[25] ? 'd' : 's', xd, fs1, fs2);
            else if (funct3 == 1)
                sprintf(ret, "flt.%c %s, %s, %s", inst[25] ? 'd' : 's', xd, fs1, fs2);
            else if (funct3 == 2)
                sprintf(ret, "feq.%c %s, %s, %s", inst[25] ? 'd' : 's', xd, fs1, fs2);
            break;
        case 0b01000:
            sprintf(ret, "fcvt.%s %s, %s", inst[25] ? "d.s" : "s.d", fd, fs1);
            break;
        case 0b11000: // FCVT.I.F
            sprintf(ret, "fcvt.%c%s.%c %s, %s",
                    inst[21] ? 'l' : 'w', inst[20] ? "u" : "", inst[25] ? 'd' : 's', xd, fs1);
            break;
        case 0b11010: // FCVT.F.I
            sprintf(ret, "fcvt.%c.%c%s %s, %s",
                    inst[25] ? 'd' : 's', inst[21] ? 'l' : 'w', inst[20] ? "u" : "", fd, xs1);
            break;
        case 0b11100: // FMV.X.F
            if (funct3 == 0)
                sprintf(ret, "fmv.x.%c %s, %s", inst[25] ? 'd' : 'w', xd, fs1);
            else if (funct3 == 1) // FCLASS
                sprintf(ret, "fclass.%c %s, %s", inst[25] ? 'd' : 'w', xd, fs1);
            break;
        case 0b11110: // FMV.F.X
            sprintf(ret, "fmv.%c.x %s, %s", inst[25] ? 'd' : 'w', fd, xs1);
            break;
        }
        break;
    case 0b1100011: // BRANCH
        imm = bits((inst[31] << 12) | (inst[7] << 11) |
                   (inst.range(25, 30) << 5) | (inst.range(8, 11) << 1))
                  .sext(13);
        static const char *bnames[] = {"beq", "bne", "", "", "blt", "bge", "bltu", "bgeu"};
        if (funct3 != 2 && funct3 != 3)
            sprintf(ret, "%s %s, %s, %ld(pc)", bnames[funct3], xs1, xs2, imm);
        break;
    case 0b1100111: // JALR
        imm = inst.range(20, 31).sext(12);
        sprintf(ret, "jalr %s, %ld(%s)", xd, imm, xs1);
        break;
    case 0b1101111: // JAL
        imm = bits((inst[31] << 20) | (inst.range(12, 19) << 12) |
                   (inst[20] << 11) | (inst.range(21, 30) << 1))
                  .sext(21);
        sprintf(ret, "jal %s, %ld(pc)", xd, imm);
        break;
    case 0b1110011: // SYSTEM
        static char cname[4] = {0, 'w', 's', 'c'}, immname[16];
        static std::map<uint16_t, const char *> csrnamefull = {
            {0x004, "uie"},
            {0x005, "utvec"},
            {0x040, "uscratch"},
            {0x041, "uepc"},
            {0x042, "ucause"},
            {0x043, "utval"},
            {0x044, "uip"},
            {0xc00, "cycle"},
            {0xc01, "time"},
            {0xc02, "instret"},
            {0xc03, "hpmcounter3"},
            {0xc04, "hpmcounter4"},
            {0xc05, "hpmcounter5"},
            {0xc06, "hpmcounter6"},
            {0xc07, "hpmcounter7"},
            {0xc08, "hpmcounter8"},
            {0xc09, "hpmcounter9"},
            {0xc0a, "hpmcounter10"},
            {0xc0b, "hpmcounter11"},
            {0xc0c, "hpmcounter12"},
            {0xc0d, "hpmcounter13"},
            {0xc0e, "hpmcounter14"},
            {0xc0f, "hpmcounter15"},
            {0xc10, "hpmcounter16"},
            {0xc11, "hpmcounter17"},
            {0xc12, "hpmcounter18"},
            {0xc13, "hpmcounter19"},
            {0xc14, "hpmcounter20"},
            {0xc15, "hpmcounter21"},
            {0xc16, "hpmcounter22"},
            {0xc17, "hpmcounter23"},
            {0xc18, "hpmcounter24"},
            {0xc19, "hpmcounter25"},
            {0xc1a, "hpmcounter26"},
            {0xc1b, "hpmcounter27"},
            {0xc1c, "hpmcounter28"},
            {0xc1d, "hpmcounter29"},
            {0xc1e, "hpmcounter30"},
            {0xc1f, "hpmcounter31"},
            {0x100, "sstatus"},
            {0x102, "sedeleg"},
            {0x103, "sideleg"},
            {0x104, "sie"},
            {0x144, "sip"}};
        std::map<uint16_t, const char *> *pnames = &csrname;
        if (pnames->find(inst.range(20, 31)) == pnames->end())
            pnames = &csrnamefull;
        sprintf(immname, "%lu", (uint64_t)inst.range(15, 19));
        if (inst.range(12, 13))
            sprintf(ret, "csrr%c%s %s, %s, %s",
                    cname[inst.range(12, 13)], inst[14] ? "i" : "",
                    xd, (*pnames)[inst.range(20, 31)], inst[14] ? immname : xs1);
        else if ((ir & ~(1 << 20)) == 0x73)
            sprintf(ret, inst[20] ? "ebreak" : "ecall");
        else if (ir == 0x30200073)
            sprintf(ret, "mret");
        else if (ir == 0x10200073)
            sprintf(ret, "sret");
        else if (ir == 0x12000073)
            sprintf(ret, "sfence.vma");
        else if (ir == 0x10500073)
            sprintf(ret, "wfi");
        break;
    }
    if (ir == 0x00c02013)
        sprintf(ret, "[instruction page fault]");
    if (ir == 0x00002013)
        sprintf(ret, "[instruction address misaligned]");
    return ret;
}

/**
 * @brief generate exception delta
 * @param state current state
 * @param del    current delta
 * @param level  exception level
 * @param cause  exception cause
 * @retval newly generated delta on current state and delta
 */
inline delta_t &genx(state_t &state, delta_t &del, uint8_t level, uint64_t cause, uint64_t val = 0)
{
    delta_t &ret = del;
    ret.gprw = ret.memw = 0;
    if (level == 1) // trap into S mode
    {
        ret.level = 1;
        ret.pc = state.csr["stvec"];
        ret.csr["sepc"] = state.pc;
        ret.csr["scause"] = cause;
        ret.csr["stval"] = val;
        ret.csr["mstatus"] = state.csr["mstatus"];
        ret.csr["mstatus"].write(8, state.level & 1);
        ret.csr["mstatus"].write(5, state.csr["mstatus"][1]);
        ret.csr["mstatus"].write(1, 0);
    }
    else if (level == 3) // trap into M mode
    {
        ret.level = 3;
        ret.pc = state.csr["mtvec"];
        ret.csr["mepc"] = state.pc;
        ret.csr["mcause"] = cause;
        ret.csr["mtval"] = val;
        ret.csr["mstatus"] = state.csr["mstatus"];
        ret.csr["mstatus"].write(11, 12, state.level);
        ret.csr["mstatus"].write(7, state.csr["mstatus"][3]);
        ret.csr["mstatus"].write(3, 0);
    }
    if ((ret.pc & 3) == 1 && (int64_t)cause < 0) // vectored for async exception
        ret.pc += 4 * (cause & 0xf);
    return ret;
}

/**
 * @brief calculate a next state
 * @param s the current state
 * @retval a possible delta for next state
 */
delta_t next(state_t &s)
{
    /* initialize */
    delta_t ret;
    ret.level = s.level;
    ret.gprw = ret.memw = 0;
    s.pc &= ~1ull;

    /* interrupt */
    bool mintena = s.level < 3 || s.level == 3 && s.csr["mstatus"][3];
    bool sintena = s.level < 1 || s.level == 1 && s.csr["mstatus"][1];
    if (s.csr["mie"] & s.csr["mip"])
        for (int i = 0; i < 64; i++)
            if (s.csr["mie"][i] && s.csr["mip"][i])
                if (s.csr["mideleg"][i] && sintena)
                    return genx(s, ret, 1, (1ull << 63) | i);
                else if (!s.csr["mideleg"][i] && mintena)
                    return genx(s, ret, 3, (1ull << 63) | i);

    /* fetch instruction and decode compressed instruction */
    bits medeleg = s.level <= 1 ? s.csr["medeleg"] : bits();
    uint64_t su = s.level == 0 ? 0x100 : (s.level == 1 ? 0x200 : 0);
    uint64_t satp = s.level == 3 ? 0 : (uint64_t)s.csr["satp"];
    uint64_t ppc = paddr(s.mem, satp, s.pc, su | 8); // require X permission
    bits idata = s.mem.ui32(ppc), ir;
    if (ppc == -1)
    {
        s.ir = 0x00c02013; // HINT for instruction page fault(SLTI zero, zero, 12)
        return genx(s, ret, medeleg[12] ? 1 : 3, 12, s.pc);
    }
    if ((s.pc & 0xfff) == 0xffe) // beyond page
        idata = idata & 0xffff | (s.mem.ui32(ppc = paddr(s.mem, satp, s.pc + 2, su | 8)) << 16);
    if (ppc == -1)
    {
        s.ir = 0x00c02013; // HINT for instruction page fault(SLTI zero, zero, 12)
        return genx(s, ret, medeleg[12] ? 1 : 3, 12, s.pc + 2);
    }
    if (s.csr["mstatus"][17] && s.csr["mstatus"].range(11, 12) < 3) // MPRV bit in mstatus
    {
        satp = (uint64_t)s.csr["satp"];
        su = s.csr["mstatus"].range(11, 12) == 1 ? 0x200 : 0x100;
    }
    if (s.csr["mstatus"][18] && su == 0x200) // SUM bit in mstatus
        su = 0;
    s.ir = idata[0] && idata[1] ? idata : idata.range(0, 15);
    ret.pc = s.pc + (idata.range(0, 1) == 3 ? 4 : 2);
    switch ((idata.range(13, 15) << 2) | idata.range(0, 1))
    {
    case 0b00000:
        if (idata.range(5, 12)) // C.ADDI4SPN
        {
            ir |= idata.range(7, 10) << 26;
            ir |= idata.range(11, 12) << 24;
            ir |= idata[5] << 23;
            ir |= idata[6] << 22;
            ir |= 2 << 15;
            ir |= (idata.range(2, 4) + 8) << 7;
            ir |= 0x13;
        }
        break;
    case 0b00100: // C.FLD
        ir |= idata.range(5, 6) << 26;
        ir |= idata.range(10, 12) << 23;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= (idata.range(2, 4) + 8) << 7;
        ir |= 7;
        break;
    case 0b01000: // C.LW
        ir |= idata[5] << 26;
        ir |= idata.range(10, 12) << 23;
        ir |= idata[6] << 22;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 2 << 12;
        ir |= (idata.range(2, 4) + 8) << 7;
        ir |= 3;
        break;
    case 0b01100: // C.LD
        ir |= idata.range(5, 6) << 26;
        ir |= idata.range(10, 12) << 23;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= (idata.range(2, 4) + 8) << 7;
        ir |= 3;
        break;
    case 0b10100: // C.FSD
        ir |= idata.range(5, 6) << 26;
        ir |= idata[12] << 25;
        ir |= (idata.range(2, 4) + 8) << 20;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= 0x27;
        break;
    case 0b11000: // C.SW
        ir |= idata[5] << 26;
        ir |= idata[12] << 25;
        ir |= (idata.range(2, 4) + 8) << 20;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 2 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= idata[6] << 9;
        ir |= 0x23;
        break;
    case 0b11100: // C.SD
        ir |= idata.range(5, 6) << 26;
        ir |= idata[12] << 25;
        ir |= (idata.range(2, 4) + 8) << 20;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 3 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= 0x23;
        break;
    case 0b00001: // C.ADDI / C.NOP
        ir |= (idata[12] ? (uint64_t)(-1) : 0) << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= idata.range(7, 11) << 15;
        ir |= idata.range(7, 11) << 7;
        ir |= 0x13;
        break;
    case 0b00101: // ADDIW
        ir |= (idata[12] ? (uint64_t)(-1) : 0) << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= idata.range(7, 11) << 15;
        ir |= idata.range(7, 11) << 7;
        ir |= 0x1b;
        break;
    case 0b01001: // C.LI
        ir |= (idata[12] ? (uint64_t)(-1) : 0) << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= idata.range(7, 11) << 7;
        ir |= 0x13;
        break;
    case 0b01101:
        if (idata.range(7, 11) == 2) // C.ADDI16SP
        {
            ir |= (idata[12] ? (uint64_t)(-1) : 0) << 29;
            ir |= idata.range(3, 4) << 27;
            ir |= idata[5] << 26;
            ir |= idata[2] << 25;
            ir |= idata[6] << 24;
            ir |= 2 << 15;
            ir |= 2 << 7;
            ir |= 0x13;
        }
        else if (idata.range(2, 6) || idata[12]) // C.LUI
        {
            ir |= (idata[12] ? (uint64_t)(-1) : 0) << 17;
            ir |= idata.range(2, 6) << 12;
            ir |= idata.range(7, 11) << 7;
            ir |= 0x37;
        }
        break;
    case 0b10001:
        if (idata[11] == 0) // C.SRLI / C.SRAI
        {
            ir |= idata.range(10, 11) << 30;
            ir |= idata[12] << 25;
            ir |= idata.range(2, 6) << 20;
            ir |= (idata.range(7, 9) + 8) << 15;
            ir |= 5 << 12;
            ir |= (idata.range(7, 9) + 8) << 7;
            ir |= 0x13;
        }
        else if (idata[10] == 0) // C.ANDI
        {
            ir |= (idata[12] ? (uint64_t)(-1) : 0) << 25;
            ir |= idata.range(2, 6) << 20;
            ir |= (idata.range(7, 9) + 8) << 15;
            ir |= 7 << 12;
            ir |= (idata.range(7, 9) + 8) << 7;
            ir |= 0x13;
        }
        else
            switch ((idata[12] << 2) | idata.range(5, 6))
            {
            case 0b000: // C.SUB
                ir |= 0b0100000 << 25;
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b001: // C.XOR
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= 4 << 12;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b010: // C.OR
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= 6 << 12;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b011: // C.AND
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= 7 << 12;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x33;
                break;
            case 0b100: // C.SUBW
                ir |= 0b0100000 << 25;
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x3b;
                break;
            case 0b101: // C.ADDW
                ir |= (idata.range(2, 4) + 8) << 20;
                ir |= (idata.range(7, 9) + 8) << 15;
                ir |= (idata.range(7, 9) + 8) << 7;
                ir |= 0x3b;
                break;
            }
        break;
    case 0b10101: // C.J
        ir |= idata[12] << 31;
        ir |= idata[8] << 30;
        ir |= idata.range(9, 10) << 28;
        ir |= idata[6] << 27;
        ir |= idata[7] << 26;
        ir |= idata[2] << 25;
        ir |= idata[11] << 24;
        ir |= idata.range(3, 5) << 21;
        ir |= (idata[12] ? (1 << 9) - 1 : 0) << 12;
        ir |= 0x6f;
        break;
    case 0b11001: // C.BEQZ
        ir |= (idata[12] ? (uint64_t)(-1) : 0) << 28;
        ir |= idata.range(5, 6) << 26;
        ir |= idata[2] << 25;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= idata.range(10, 11) << 10;
        ir |= idata.range(3, 4) << 8;
        ir |= idata[12] << 7;
        ir |= 0x63;
        break;
    case 0b11101: // C.BNEZ
        ir |= (idata[12] ? (uint64_t)(-1) : 0) << 28;
        ir |= idata.range(5, 6) << 26;
        ir |= idata[2] << 25;
        ir |= (idata.range(7, 9) + 8) << 15;
        ir |= 1 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= idata.range(3, 4) << 8;
        ir |= idata[12] << 7;
        ir |= 0x63;
        break;
    case 0b00010:
        if (idata[12] || idata.range(2, 6)) // C.SLLI
        {
            ir |= idata[12] << 25;
            ir |= idata.range(2, 6) << 20;
            ir |= idata.range(7, 11) << 15;
            ir |= 1 << 12;
            ir |= idata.range(7, 11) << 7;
            ir |= 0x13;
        }
        break;
    case 0b00110: // C.FLDSP
        ir |= idata.range(2, 4) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(5, 6) << 23;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= idata.range(7, 11) << 7;
        ir |= 7;
        break;
    case 0b01010: // C.LWSP
        ir |= idata.range(2, 3) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(4, 6) << 22;
        ir |= 2 << 15;
        ir |= 2 << 12;
        ir |= idata.range(7, 11) << 7;
        ir |= 3;
        break;
    case 0b01110: // C.LDSP
        ir |= idata.range(2, 4) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(5, 6) << 23;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= idata.range(7, 11) << 7;
        ir |= 3;
        break;
    case 0b10010:
        if (!idata[12] && !idata.range(2, 6) && idata.range(7, 11)) // C.JR
            ir |= (idata.range(7, 11) << 15) | 0x67;
        if (!idata[12] && idata.range(2, 6)) // C.MV
        {
            ir |= idata.range(2, 6) << 20;
            ir |= idata.range(7, 11) << 7;
            ir |= 0x33;
        }
        if (idata[12] && !idata.range(2, 6) && idata.range(7, 11) == 0) // C.EBREAK
            ir = (1 << 20) | 0x73;
        if (idata[12] && !idata.range(2, 6) && idata.range(7, 11)) // C.JALR
        {
            ir |= idata.range(7, 11) << 15;
            ir |= 1 << 7;
            ir |= 0x67;
        }
        if (idata[12] && idata.range(2, 6))
        {
            ir |= idata.range(2, 6) << 20;
            ir |= idata.range(7, 11) << 15;
            ir |= idata.range(7, 11) << 7;
            ir |= 0x33;
        }
        break;
    case 0b10110: // C.FSDSP
        ir |= idata.range(7, 9) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= 0x27;
        break;
    case 0b11010: // C.SWSP
        ir |= idata.range(7, 8) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= 2 << 15;
        ir |= 2 << 12;
        ir |= idata.range(9, 11) << 9;
        ir |= 0x23;
        break;
    case 0b11110: // C.SDSP
        ir |= idata.range(7, 9) << 26;
        ir |= idata[12] << 25;
        ir |= idata.range(2, 6) << 20;
        ir |= 2 << 15;
        ir |= 3 << 12;
        ir |= idata.range(10, 11) << 10;
        ir |= 0x23;
        break;
    default:
        ir = idata.range(0, 31);
        break;
    }

    /* execute */
    uint8_t funct3 = ir.range(12, 14), fs = s.csr["mstatus"].range(13, 14);
    uint64_t rs1 = s.gpr[ir.range(15, 19)], rs2 = s.gpr[ir.range(20, 24)];
    double ds1 = (double)s.gpr[ir.range(15, 19) + 32], ds2 = (double)s.gpr[ir.range(20, 24) + 32],
           ds3 = (double)s.gpr[ir.range(27, 31) + 32];
    float ss1 = (float)s.gpr[ir.range(15, 19) + 32], ss2 = (float)s.gpr[ir.range(20, 24) + 32],
          ss3 = (float)s.gpr[ir.range(27, 31) + 32];
    int64_t imm;
    uint64_t va, pa;
    switch (ir.range(0, 6)) // opcode
    {
    case 0b0000011: // LOAD
        va = rs1 + ir.range(20, 31).sext(12);
        if ((pa = paddr(s.mem, satp, va, su | 2)) == -1)      // require R permission
            return genx(s, ret, medeleg[13] ? 1 : 3, 13, va); // load PF
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        ret.gprv = s.mem.ui64(pa);
        if (funct3 == 0b000) // LB
            ret.gprv = (int64_t)(int8_t)ret.gprv;
        else if (funct3 == 0b100) // LBU
            ret.gprv = (uint8_t)ret.gprv;
        else if (funct3 == 0b001) // LH
            ret.gprv = (int64_t)(int16_t)ret.gprv;
        else if (funct3 == 0b101) // LHU
            ret.gprv = (uint16_t)ret.gprv;
        else if (funct3 == 0b010) // LW
            ret.gprv = (int64_t)(int32_t)ret.gprv;
        else if (funct3 == 0b110) // LWU
            ret.gprv = (uint32_t)ret.gprv;
        break;
    case 0b0000111: // LOAD-FP
        if (!fs)
            return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
        va = rs1 + ir.range(20, 31).sext(12);
        if ((pa = paddr(s.mem, satp, va, su | 2)) == -1)
            return genx(s, ret, medeleg[13] ? 1 : 3, 13, va); // load PF
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11) + 32;
        if (funct3 == 0b010) // FLW
            ret.gprv = s.mem.ui64(pa) | (-1llu << 32);
        else if (funct3 == 0b011) // FLD
            ret.gprv = s.mem.ui64(pa);
        break;
    case 0b0001111: // MISC-MEM
        break;
    case 0b0010011: // OP-IMM
        imm = ir.range(20, 31).sext(12);
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        if (funct3 == 0b000) // ADDI
            ret.gprv = rs1 + imm;
        else if (funct3 == 0b010) // SLTI
            ret.gprv = (int64_t)rs1 < imm;
        else if (funct3 == 0b011) // SLTIU
            ret.gprv = rs1 < (uint64_t)imm;
        else if (funct3 == 0b100) // XORI
            ret.gprv = rs1 ^ imm;
        else if (funct3 == 0b110) // ORI
            ret.gprv = rs1 | imm;
        else if (funct3 == 0b111) // ANDI
            ret.gprv = rs1 & imm;
        else if (funct3 == 0b001) // SLLI
            ret.gprv = rs1 << (imm &= 0x3f);
        else if (funct3 == 0b101)
            if (ir[30]) // SRAI
                ret.gprv = (int64_t)rs1 >> (int64_t)(imm &= 0x3f);
            else // SRLI
                ret.gprv = rs1 >> (imm &= 0x3f);
        break;
    case 0b0010111: // AUIPC
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        ret.gprv = s.pc + bits(ir.range(12, 31) << 12).sext(32);
        break;
    case 0b0011011: // OP-IMM-32
        imm = ir.range(20, 31).sext(12);
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        if (funct3 == 0b000) // ADDIW
            ret.gprv = (int32_t)rs1 + (int32_t)imm;
        else if (funct3 == 0b001) // SLLIW
            ret.gprv = (int32_t)rs1 << (int32_t)(imm &= 0x3f);
        else if (funct3 == 0b101)
            if (ir[30]) // SRAIW
                ret.gprv = (int64_t)((int32_t)rs1 >> (int32_t)(imm &= 0x3f));
            else // SRLIW
                ret.gprv = (int64_t)(int32_t)((uint32_t)rs1 >> (imm &= 0x3f));
        break;
    case 0b0100111: // STORE-FP
        if (!fs)
            return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
    case 0b0100011: // STORE
        va = rs1 + bits(ir.range(25, 31) << 5 | ir.range(7, 11)).sext(12);
        if ((pa = paddr(s.mem, satp, va, su | 4)) == -1)      // require W permission
            return genx(s, ret, medeleg[15] ? 1 : 3, 15, va); // store PF
        ret.memw = 1 << ir.range(12, 13);
        ret.mema = pa;
        ret.memv = s.gpr[ir.range(20, 24) + (ir[2] ? 32 : 0)];
        if (ret.memw < 8)
            ret.memv &= ~(-1ull << 8 * ret.memw);
        break;
    case 0b0101111: // AMO
        switch (ir.range(27, 31))
        {
        case 0b00010: // LR
            if ((pa = paddr(s.mem, satp, rs1, su | 2)) == -1)
                return genx(s, ret, medeleg[13] ? 1 : 3, 13, rs1); // load PF
            ret.gprw = 1;
            ret.gpra = ir.range(7, 11);
            ret.gprv = s.mem.ui64(pa);
            ret.memw = 0x80 | (1 << ir.range(12, 13));
            ret.mema = pa;
            if (funct3 == 0b010) // LR.W
                ret.gprv = (int64_t)(int32_t)ret.gprv;
            break;
        case 0b00011: // SC
            if ((pa = paddr(s.mem, satp, rs1, su | 4)) == -1)
                return genx(s, ret, medeleg[15] ? 1 : 3, 15, rs1); // store PF
            ret.gprw = 1;
            ret.gpra = ir.range(7, 11);
            if (s.rsrv[pa + 0] && s.rsrv[pa + 1] &&
                s.rsrv[pa + 2] && s.rsrv[pa + 3] &&
                (funct3 == 0b010 || s.rsrv[pa + 4] && s.rsrv[pa + 5] &&
                                        s.rsrv[pa + 6] && s.rsrv[pa + 7]))
            {
                ret.gprv = 0;
                ret.memw = 0xc0 | (1 << ir.range(12, 13));
                ret.mema = pa;
                ret.memv = rs2;
                if (ret.memw & 0xf < 8)
                    ret.memv &= ~(-1ull << 8 * ret.memw);
            }
            else
                ret.gprv = 1;
            break;
        default:
            if ((pa = paddr(s.mem, satp, rs1, su | 4)) == -1)
                return genx(s, ret, medeleg[15] ? 1 : 3, 15, rs1); // AMO PF
            ret.gprw = 1;
            ret.gpra = ir.range(7, 11);
            ret.gprv = s.mem.ui64(pa);
            ret.memw = 1 << ir.range(12, 13);
            ret.mema = pa;
            if (funct3 == 0b010) // AMO*.W
                ret.gprv = (int64_t)(int32_t)ret.gprv, rs2 = (int64_t)(int32_t)rs2;
            ret.memv = ret.gprv;
            if (ir.range(27, 31) == 0b00001) // AMOSWAP
                ret.memv = rs2;
            if (ir.range(27, 31) == 0b00000) // AMOADD
                ret.memv = rs2 + ret.gprv;
            if (ir.range(27, 31) == 0b00100) // AMOXOR
                ret.memv = rs2 ^ ret.gprv;
            if (ir.range(27, 31) == 0b01100) // AMOAND
                ret.memv = rs2 & ret.gprv;
            if (ir.range(27, 31) == 0b01000) // AMOOR
                ret.memv = rs2 | ret.gprv;
            if (ir.range(27, 31) == 0b10000) // AMOMIN
                ret.memv = (int64_t)rs2 < (int64_t)ret.gprv ? (uint64_t)rs2 : ret.gprv;
            if (ir.range(27, 31) == 0b10100) // AMOMAX
                ret.memv = (int64_t)rs2 > (int64_t)ret.gprv ? (uint64_t)rs2 : ret.gprv;
            if (ir.range(27, 31) == 0b11000) // AMOMINU
                ret.memv = rs2 < ret.gprv ? (uint64_t)rs2 : ret.gprv;
            if (ir.range(27, 31) == 0b11100) // AMOMAXU
                ret.memv = rs2 > ret.gprv ? (uint64_t)rs2 : ret.gprv;
            if (ret.memw < 8)
                ret.memv &= ~((uint64_t)-1 << (8 * ret.memw));
            break;
        }
        break;
    case 0b0110011: // OP
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        if (ir[25])
        {
            uint64_t a[4], b[4];
            a[0] = bits(rs1).range(0, 31), a[1] = bits(rs1).range(32, 63);
            a[2] = a[3] = bits(rs1)[63] && funct3 <= 2 ? 0xffffffffull : 0;
            b[0] = bits(rs2).range(0, 31), b[1] = bits(rs2).range(32, 63);
            b[2] = b[3] = bits(rs2)[63] && funct3 <= 1 ? 0xffffffffull : 0;
            uint64_t l = a[0] * b[0] + ((a[0] * b[1] + a[1] * b[0]) << 32);
            uint64_t h =
                ((a[0] * b[0] >> 32) + (a[0] * b[1] << 32 >> 32) + (a[1] * b[0] << 32 >> 32) >> 32) +
                (a[0] * b[1] >> 32) + (a[1] * b[0] >> 32) +
                (a[0] * b[2]) + (a[1] * b[1]) + (a[2] * b[0]) +
                (a[0] * b[3] + a[1] * b[2] + a[2] * b[1] + a[3] * b[0] << 32);
            if (funct3 == 0b000) // MUL
                ret.gprv = l;
            else if (funct3 <= 0b011) // MULH[[S]U]
                ret.gprv = h;
            else if (funct3 == 0b100) // DIV
                if (rs1 == 0x8000000000000000 && rs2 == -1)
                    ret.gprv = rs1;
                else
                    ret.gprv = rs2 == 0 ? -1 : (int64_t)rs1 / (int64_t)rs2;
            else if (funct3 == 0b101) // DIVU
                ret.gprv = rs2 == 0 ? (uint64_t)-1 : rs1 / rs2;
            else if (funct3 == 0b110) // REM
                if (rs1 == 0x8000000000000000 && rs2 == -1)
                    ret.gprv = 0;
                else
                    ret.gprv = rs2 == 0 ? (int64_t)rs1 : (int64_t)rs1 % (int64_t)rs2;
            else if (funct3 == 0b111) // REMU
                ret.gprv = rs2 == 0 ? (uint64_t)rs1 : rs1 % rs2;
        }
        else if (funct3 == 0b000)
            if (ir[30]) // SUB
                ret.gprv = rs1 - rs2;
            else // ADD
                ret.gprv = rs1 + rs2;
        else if (funct3 == 0b001) // SLL
            ret.gprv = rs1 << rs2;
        else if (funct3 == 0b010) // SLT
            ret.gprv = (int64_t)rs1 < (int64_t)rs2;
        else if (funct3 == 0b011) // SLTU
            ret.gprv = rs1 < rs2;
        else if (funct3 == 0b100) // XOR
            ret.gprv = rs1 ^ rs2;
        else if (funct3 == 0b101)
            if (ir[30]) // SRA
                ret.gprv = (int64_t)rs1 >> (int64_t)rs2;
            else // SRL
                ret.gprv = rs1 >> rs2;
        else if (funct3 == 0b110) // OR
            ret.gprv = rs1 | rs2;
        else if (funct3 == 0b111) // AND
            ret.gprv = rs1 & rs2;
        break;
    case 0b0110111: // LUI
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        ret.gprv = bits(ir.range(12, 31) << 12).sext(32);
        break;
    case 0b0111011: // OP-32
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        if (ir[25])
        {
            if (funct3 == 0b000) // MULW
                ret.gprv = bits(rs1 * rs2).sext(32);
            else if (funct3 == 0b100) // DIVW
                if ((int32_t)rs1 == 0x80000000 && (int32_t)rs2 == -1)
                    ret.gprv = rs1;
                else
                    ret.gprv = bits(uint64_t(rs2 == 0 ? -1 : (int32_t)rs1 / (int32_t)rs2)).sext(32);
            else if (funct3 == 0b101) // DIVUW
                ret.gprv = bits(uint64_t(rs2 == 0 ? -1 : (uint32_t)rs1 / (uint32_t)rs2)).sext(32);
            else if (funct3 == 0b110) // REMW
                if ((int32_t)rs1 == 0x80000000 && (int32_t)rs2 == -1)
                    ret.gprv = 0;
                else
                    ret.gprv = bits(uint64_t(rs2 == 0 ? (int32_t)rs1 : (int32_t)rs1 % (int32_t)rs2)).sext(32);
            else if (funct3 == 0b111) // REMUW
                ret.gprv = bits(uint64_t(rs2 == 0 ? (uint32_t)rs1 : (uint32_t)rs1 % (uint32_t)rs2)).sext(32);
        }
        else if (funct3 == 0b000)
            if (ir[30]) // SUBW
                ret.gprv = (int32_t)rs1 - (int32_t)rs2;
            else // ADDW
                ret.gprv = (int32_t)rs1 + (int32_t)rs2;
        else if (funct3 == 0b001) // SLLW
            ret.gprv = (int32_t)rs1 << (int32_t)rs2;
        else if (funct3 == 0b101)
            if (ir[30]) // SRAW
                ret.gprv = (int64_t)((int32_t)rs1 >> (int32_t)rs2);
            else // SRLW
                ret.gprv = (int64_t)(int32_t)((uint32_t)rs1 >> (uint32_t)rs2);
        break;
    case 0b1000011: // MADD
    case 0b1000111: // MSUB
    case 0b1001011: // NMSUB
    case 0b1001111: // NMADD
        if (!fs)
            return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11) + 32;
        if (ir.range(2, 3) == 0)
            ret.gprv = ir[25] ? bits(ds1 * ds2 + ds3) : bits(ss1 * ss2 + ss3);
        else if (ir.range(2, 3) == 1)
            ret.gprv = ir[25] ? bits(ds1 * ds2 - ds3) : bits(ss1 * ss2 - ss3);
        else if (ir.range(2, 3) == 2)
            ret.gprv = ir[25] ? bits(-ds1 * ds2 + ds3) : bits(-ss1 * ss2 + ss3);
        else if (ir.range(2, 3) == 3)
            ret.gprv = ir[25] ? bits(-ds1 * ds2 - ds3) : bits(-ss1 * ss2 - ss3);
        break;
    case 0b1010011: // OP-FP
        if (!fs)
            return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11) + 32;
        switch (ir.range(27, 31))
        {
        case 0b00000: // FADD
            ret.gprv = ir[25] ? bits(ds1 + ds2) : bits(ss1 + ss2);
            break;
        case 0b00001: // FSUB
            ret.gprv = ir[25] ? bits(ds1 - ds2) : bits(ss1 - ss2);
            break;
        case 0b00010: // FMUL
            ret.gprv = ir[25] ? bits(ds1 * ds2) : bits(ss1 * ss2);
            break;
        case 0b00011: // FDIV
            ret.gprv = ir[25] ? bits(ds1 / ds2) : bits(ss1 / ss2);
            break;
        case 0b01011: // FSQRT
            ret.gprv = ir[25] ? bits(sqrt(ds1)) : bits(sqrtf(ss1));
            break;
        case 0b00100:
            if (funct3 == 0) // FSGNJ
                ret.gprv = ir[25] ? bits(ds2 < 0 ? -ds1 : ds1) : bits(ss2 < 0 ? -ss1 : ss1);
            else if (funct3 == 1) // FSGNJN
                ret.gprv = ir[25] ? bits(ds2 < 0 ? ds1 : -ds1) : bits(ss2 < 0 ? ss1 : -ss1);
            else if (funct3 == 2) // FSGNJX
                ret.gprv = ir[25] ? bits(ds1 < 0 ^ ds2 < 0 ? -ds1 : ds1)
                                  : bits(ss1 < 0 ^ ss2 < 0 ? -ss1 : ss1);
            break;
        case 0b00101:
            if (funct3 == 0) // FMIN
                ret.gprv = ir[25] ? bits(fmin(ds1, ds2)) : bits(fminf(ss1, ss2));
            else if (funct3 == 1) // FMAX
                ret.gprv = ir[25] ? bits(fmax(ds1, ds2)) : bits(fmaxf(ss1, ss2));
            break;
        case 0b10100:
            ret.gpra = ir.range(7, 11);
            if (funct3 == 0) // FLE
                ret.gprv = ir[25] ? ds1 <= ds2 : ss1 <= ss2;
            else if (funct3 == 1) // FLT
                ret.gprv = ir[25] ? ds1 < ds2 : ss1 < ss2;
            else if (funct3 == 2) // FEQ
                ret.gprv = ir[25] ? ds1 == ds2 : ss1 == ss2;
            break;
        case 0b01000: // FCVT.D.S / FCVT.S.D
            ret.gprv = ir[25] ? bits(double(ss1)) : bits(float(ds1));
            break;
        case 0b11000: // FCVT.I.F
            ret.gpra = ir.range(7, 11);
            switch ((ir[25] << 2) | ir.range(20, 21))
            {
            case 0b000: // FCVT.W.S
                ret.gprv = (int32_t)round(ss1);
                break;
            case 0b001: // FCVT.WU.S
                ret.gprv = (uint32_t)round(ss1);
                break;
            case 0b010: // FCVT.L.S
                ret.gprv = (int64_t)round(ss1);
                break;
            case 0b011: // FCVT.LU.S
                ret.gprv = round(ss1);
                break;
            case 0b100: // FCVT.W.D
                ret.gprv = (int32_t)round(ds1);
                break;
            case 0b101: // FCVT.WU.D
                ret.gprv = (uint32_t)round(ds1);
                break;
            case 0b110: // FCVT.L.D
                ret.gprv = (int64_t)round(ds1);
                break;
            case 0b111: // FCVT.LU.D
                ret.gprv = round(ds1);
                break;
            }
            break;
        case 0b11010: // FCVT.F.I
            switch ((ir[25] << 2) | ir.range(20, 21))
            {
            case 0b000: // FCVT.S.W
                ret.gprv = (bits)(float)(int32_t)rs1;
                break;
            case 0b001: // FCVT.S.WU
                ret.gprv = (bits)(float)(uint32_t)rs1;
                break;
            case 0b010: // FCVT.S.L
                ret.gprv = (bits)(float)(int64_t)rs1;
                break;
            case 0b011: // FCVT.S.LU
                ret.gprv = (bits)(float)rs1;
                break;
            case 0b100: // FCVT.D.W
                ret.gprv = (bits)(double)(int32_t)rs1;
                break;
            case 0b101: // FCVT.D.W
                ret.gprv = (bits)(double)(uint32_t)rs1;
                break;
            case 0b110: // FCVT.D.L
                ret.gprv = (bits)(double)(int64_t)rs1;
                break;
            case 0b111: // FCVT.D.LU
                ret.gprv = (bits)(double)rs1;
                break;
            }
            break;
        case 0b11100:
            ret.gpra = ir.range(7, 11);
            if (funct3 == 0) // FMV.X.F
                ret.gprv = ir[25] ? *(uint64_t *)&ds1 : *(uint32_t *)&ss1 | 0xffff'ffff'0000'0000;
            else if (funct3 == 1) // FCLASS
            {
                using namespace std;
                if (ir[25] ? isinf(ds1) && ds1 < 0 : isinf(ss1) && ss1 < 0)
                    ret.gprv = 1;
                else if (ir[25] ? isnormal(ds1) && ds1 < 0 : isnormal(ss1) && ss1 < 0)
                    ret.gprv = 2;
                else if (ir[25] ? issubnormal(ds1) && ds1 < 0 : issubnormal(ss1) && ss1 < 0)
                    ret.gprv = 4;
                else if (ir[25] ? iszero(ds1) && ds1 < 0 : iszero(ss1) && ss1 < 0)
                    ret.gprv = 8;
                else if (ir[25] ? iszero(ds1) && ds1 > 0 : iszero(ss1) && ss1 > 0)
                    ret.gprv = 16;
                else if (ir[25] ? issubnormal(ds1) && ds1 > 0 : issubnormal(ss1) && ss1 > 0)
                    ret.gprv = 32;
                else if (ir[25] ? isnormal(ds1) && ds1 > 0 : isnormal(ss1) && ss1 > 0)
                    ret.gprv = 64;
                else if (ir[25] ? isinf(ds1) && ds1 > 0 : isinf(ss1) && ss1 > 0)
                    ret.gprv = 128;
                else if (ir[25] ? isnan(ds1) && issignaling(ds1) : isnan(ss1) && issignaling(ss1))
                    ret.gprv = 256;
                else if (ir[25] ? isnan(ds1) && !issignaling(ds1) : isnan(ss1) && !issignaling(ss1))
                    ret.gprv = 512;
            }
            break;
        case 0b11110: // FMV.F.X
            ret.gprv = ir[25] ? bits(rs1) : bits((float)(bits)rs1);
            break;
        }
        break;
    case 0b1100011: // BRANCH
        imm = bits((ir[31] << 12) | (ir[7] << 11) | (ir.range(25, 30) << 5) | (ir.range(8, 11) << 1)).sext(13);
        if (funct3 == 0b000 && rs1 == rs2 ||                   // BEQ
            funct3 == 0b001 && rs1 != rs2 ||                   // BNE
            funct3 == 0b100 && (int64_t)rs1 < (int64_t)rs2 ||  // BLT
            funct3 == 0b101 && (int64_t)rs1 >= (int64_t)rs2 || // BGE
            funct3 == 0b110 && rs1 < rs2 ||                    // BLTU
            funct3 == 0b111 && rs1 >= rs2)                     // BGEU
            ret.pc = s.pc + imm;
        break;
    case 0b1100111: // JALR
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        ret.gprv = s.pc + (idata.range(0, 1) == 3 ? 4 : 2);
        imm = ir.range(20, 31).sext(12);
        ret.pc = rs1 + imm;
        break;
    case 0b1101111: // JAL
        ret.gprw = 1;
        ret.gpra = ir.range(7, 11);
        ret.gprv = s.pc + (idata.range(0, 1) == 3 ? 4 : 2);
        imm = bits((ir[31] << 20) | (ir.range(12, 19) << 12) | (ir[20] << 11) | (ir.range(21, 30) << 1)).sext(21);
        ret.pc = s.pc + imm;
        break;
    case 0b1110011: // SYSTEM
        if (ir.range(12, 13))
        { // CSRR[WSC]
            uint16_t addr = ir.range(20, 31);
            if (addr >= 0xc00 && addr < 0xc20) // shadowed counters
                if (s.level == 3 ||
                    s.csr["mcounteren"][addr & 0x1f] &&
                        (s.level == 2 || s.csr["scounteren"][addr & 0x1f]))
                    addr -= 0x100;
            if (addr == 0x100 || addr == 0x144 || addr == 0x104) // sstatus/sip/sie
                addr += 0x200;
            if (csrname.find(addr) == csrname.end())
                return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
            ret.gprw = 1;
            ret.gpra = ir.range(7, 11);
            ret.gprv = s.csr[csrname[addr]];
            rs1 = ir[14] ? (uint64_t)ir.range(15, 19) : rs1;
            uint64_t wvalue;
            if (ir.range(12, 13) == 1) // CSRRW
                wvalue = rs1;
            else if (ir.range(12, 13) == 2) // CSRRS
                wvalue = ret.gprv | rs1;
            else if (ir.range(12, 13) == 3) // CSRRC
                wvalue = ret.gprv & ~rs1;
            if (ir.range(20, 31).range(8, 9) > s.level) // check level
                return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
            if (ir.range(20, 31) >= 0xc00 && ir.range(20, 31) < 0xc20) // write read-only counters
                if (wvalue != ret.gprv)
                    return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
            ret.csr[csrname[addr]] = wvalue;
        }
        else if ((ir & ~(1 << 20)) == 0x73) // ECALL / EBREAK
            return genx(s, ret, medeleg[ir[20] ? 3 : s.level + 8] ? 1 : 3,
                        ir[20] ? 3 : s.level + 8);
        else if (ir == 0x30200073) // MRET
        {
            ret.pc = s.csr["mepc"];
            ret.level = s.csr["mstatus"].range(11, 12);
            ret.csr["mstatus"] = s.csr["mstatus"];
            ret.csr["mstatus"].write(11, 12, 0);
            ret.csr["mstatus"].write(3, s.csr["mstatus"][7]);
            ret.csr["mstatus"].write(7, 1);
        }
        else if (ir == 0x10200073) // SRET
        {
            ret.pc = s.csr["sepc"];
            ret.level = s.csr["mstatus"][8];
            ret.csr["mstatus"] = s.csr["mstatus"];
            ret.csr["mstatus"].write(8, 0);
            ret.csr["mstatus"].write(1, s.csr["mstatus"][5]);
            ret.csr["mstatus"].write(5, 1);
        }
        break;
    default:
        return genx(s, ret, medeleg[2] ? 1 : 3, 2, idata);
    }

    /* return state delta */
    if (ret.gpra == 0)
        ret.gprw = 0;
    ret.csr["mcycle"] = s.csr["mcycle"] + 1;
    ret.csr["minstret"] = s.csr["minstret"] + 1;
    if (ret.csr.find("misa") != ret.csr.end()) // some WARL csr fields
        ret.csr.at("misa") = 0x14112d | (1ull << 63);
    if (ret.csr.find("mstatus") != ret.csr.end()) // some WARL csr fields
        ret.csr.at("mstatus").write(32, 35, 0xa);
    return ret;
}

/**
 * @brief apply delta on state
 * @param s the state
 * @param d the delta to apply
 */
void apply(state_t &s, delta_t d)
{
    s.pc = d.pc;
    s.level = d.level;
    if (d.gprw)
        s.gpr[d.gpra] = d.gprv;
    if (d.memw >> 4 == 0x8) // LR
    {
        d.memw &= 0xf;
        for (int i = 0; i < d.memw; i++)
            s.rsrv[d.mema + i] = 1;
        d.memw = 0;
    }
    else if (d.memw >> 4 == 0xc) // SC
    {
        d.memw &= 0xf;
        for (int i = 0; i < d.memw; i++)
            s.rsrv[d.mema + i] = 0;
    }
    if (d.memw && s.mem.issegfault(s.mem[d.mema]))
        s.mem.add(1, d.mema);
    if (d.memw == 1)
        s.mem.ui8(d.mema) = d.memv;
    else if (d.memw == 2)
        s.mem.ui16(d.mema) = d.memv;
    else if (d.memw == 4)
        s.mem.ui32(d.mema) = d.memv;
    else if (d.memw == 8)
        s.mem.ui64(d.mema) = d.memv;
    for (auto i : d.csr)
        s.csr[i.first] = i.second;
}

/**
 * @brief handle HTIF requests
 * @param mem memory working on
 * @param addr HTIF addressed
 * @retval tohost exit call value ((code << 1) | 1)
 */
uint64_t htif(memory &mem, htifaddr_t &addr, std::vector<const char *> &pkargs)
{
    /* handle HTIF requests */
    uint64_t tohost_dev = mem[addr.tohost + 7];
    uint64_t tohost_cmd = mem[addr.tohost + 6];
    uint64_t tohost_dat = mem.ui64(addr.tohost) & 0xffff'ffff'ffff;
    if (tohost_dev == 0 && tohost_cmd == 0)
    {
        if (tohost_dat & 1) // exit
            return mem.ui64(addr.tohost);
        else if (tohost_dat != 0) // proxied ststem call
        {
            uint64_t magic_mem = tohost_dat, which = mem.ui64(magic_mem);
            uint64_t retval = 0;
            if (which == 0x38) // sysopenat
            {
                uint64_t arg0, arg1, arg2, arg3, arg4;
                arg0 = mem.ui64(magic_mem + 8);  // directory file descriptor
                arg1 = mem.ui64(magic_mem + 16); // filename
                arg2 = mem.ui64(magic_mem + 24); // filename size
                arg3 = mem.ui64(magic_mem + 32); // flags
                arg4 = mem.ui64(magic_mem + 40); // mode
                mem[arg1 + arg2] = 0;
                retval = openat(arg0, (char *)&mem[arg1], arg3, arg4);
            }
            else if (which == 0x39) // sysclose
            {
                uint64_t arg0;
                arg0 = mem.ui64(magic_mem + 8); // file descriptor
                retval = close(arg0);
            }
            else if (which == 0x3e) // syslseek
            {
                uint64_t arg0, arg1, arg2;
                arg0 = mem.ui64(magic_mem + 8);  // file descriptor
                arg1 = mem.ui64(magic_mem + 16); // pointer
                arg2 = mem.ui64(magic_mem + 24); // directive
                retval = lseek(arg0, arg1, arg2);
            }
            else if (which == 0x3f) // sysread
            {
                uint64_t arg0, arg1, arg2;
                arg0 = mem.ui64(magic_mem + 8);  // file descriptor
                arg1 = mem.ui64(magic_mem + 16); // memory address
                arg2 = mem.ui64(magic_mem + 24); // max read size
                retval = read(arg0, &mem[arg1], arg2);
            }
            else if (which == 0x40) // syswrite
            {
                uint64_t arg0, arg1, arg2;
                arg0 = mem.ui64(magic_mem + 8);  // file descriptor
                arg1 = mem.ui64(magic_mem + 16); // memory address
                arg2 = mem.ui64(magic_mem + 24); // write size
                fflush(NULL);
                if (arg0 = 2)
                    arg0 = 1; // redirect stderr of program to stdout for debugging
                retval = write(arg0, &mem[arg1], arg2);
            }
            else if (which == 0x43) // syspread
            {
                uint64_t arg0, arg1, arg2, arg3;
                arg0 = mem.ui64(magic_mem + 8);  // file descriptor
                arg1 = mem.ui64(magic_mem + 16); // memory address
                arg2 = mem.ui64(magic_mem + 24); // read size
                arg3 = mem.ui64(magic_mem + 32); // read offset
                retval = pread(arg0, &mem[arg1], arg2, arg3);
            }
            else if (which == 0x50) // sysfstat
            {
                uint64_t arg0, arg1;
                arg0 = mem.ui64(magic_mem + 8);  // file descriptor
                arg1 = mem.ui64(magic_mem + 16); // memory address
                retval = fstat(arg0, (struct stat *)&mem[arg1]);
            }
            else if (which == 0x5d) // exit
                return (mem.ui64(magic_mem + 8) << 1) | 1;
            else if (which == 0x7db) // pk-sysgetmainvars
            {
                // buffer format: argc(64) argv[0](64) argv[1](64) ...
                uint64_t arg0, arg1;
                arg0 = mem.ui64(magic_mem + 8);  // argument buffer address
                arg1 = mem.ui64(magic_mem + 16); // argument buffer size
                mem.ui64(arg0) = pkargs.size();
                uint64_t addr = arg0 + (pkargs.size() + 1) * 8;
                for (int i = 0; i < pkargs.size(); i++)
                {
                    mem.ui64(arg0 + (i + 1) * 8) = addr;
                    if (addr - arg0 + strlen(pkargs[i]) + 1 <= arg1)
                        memcpy(&mem[addr], pkargs[i], strlen(pkargs[i]) + 1);
                    addr += strlen(pkargs[i]) + 1;
                }
                if (addr - arg0 >= arg1)
                    retval = -1;
            }
            else
                fprintf(stderr, "[Info] Unhandled proxied system call 0x%lx\n", which);
            mem.ui64(magic_mem) = retval;
            mem.ui64(addr.fromhost) = 1;
        }
    }
    else if (tohost_dev == 1 && tohost_cmd == 1) // console write
        putchar(tohost_dat), fflush(stdout);
    else if (tohost_dev == 1 && tohost_cmd == 0) // console_read
        ;
    else
        fprintf(stderr, "[Info] Unrecognized HTIF command:\n  dev: 0x%lx  cmd: 0x%lx  data: 0x%lx\n",
                tohost_dev, tohost_cmd, tohost_dat);
    mem.ui64(addr.tohost) = 0;
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    char ch; // receive character from stdin
    if (mem.ui64(addr.fromhost) == 0 && (ch = getchar()) != EOF)
        mem.ui64(addr.fromhost) = (1ull << 56) | ch;
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) & ~O_NONBLOCK);
    return 0;
}

/**
 * @brief print delta at state
 * @param cycle the current cycle
 * @param state the currrent state
 * @param delta the delta to print
 */
void print(uint64_t cycle, state_t &state, const delta_t &delta)
{
    char s[256], lch[4] = {'U', 'S', 'H', 'M'};
    sprintf(s, "[Debug] cycle %ld: %c@%lx: %8x %s", cycle,
            lch[state.level & 3], state.pc, state.ir, disas(state.ir).c_str());
    if (strlen(s) < 63)
    {
        for (int i = strlen(s); i < 63; i++)
            s[i] = ' ';
        s[63] = 0;
    }
    fputs(s, stderr);
    if (delta.gprw)
        fprintf(stderr, " %s: %lx", gprname[delta.gpra], delta.gprv);
    if (delta.memw && delta.memw >> 4 != 0x8)
        fprintf(stderr, " d%d@%lx: %lx", delta.memw & 0xf, delta.mema,
                delta.memv & (delta.memw == 8 ? -1ul : (1ul << (delta.memw & 0xf) * 8) - 1));
    fprintf(stderr, "\n");
}

/**
 * @brief print a state
 * @param s the state to print
 * @param addr memory base address to dump
 * @param size memory size to dump
 */
void print(state_t &s, uint64_t addr, uint64_t size)
{
    fprintf(stderr, "[Debug] General-purpose registers:\n");
    for (int i = 0; i < 16; i++)
    {
        fprintf(stderr, "[Debug]");
        for (int j = 0; j < 4; j++)
            fprintf(stderr, " %8s: %016lx", gprname[i * 4 + j], (uint64_t)s.gpr[i * 4 + j]);
        fprintf(stderr, "\n");
    }
    if (size)
        dumpmem(&s.mem[addr], addr, size);
}

/**
 * @brief print memory to stderr
 * @param mem pointer to working memory
 * @param base memory base (only to print)
 * @param size memory size
 */
void dumpmem(const uint8_t *mem, uint64_t base, uint64_t size)
{
    fprintf(stderr, "[Debug] Memory@%016lx:", base);
    for (int i = 0; i < size; i++)
    {
        i % 16 ? fprintf(stderr, i % 2 ? "" : " ") : fprintf(stderr, "\n[Debug]     %08x: ", i);
        fprintf(stderr, "%02x", mem[i]);
        if ((i + 1) % 16 == 0 || i == size - 1)
        {
            if (i == size - 1)
                for (int j = i + 1; j < i / 16 * 16 + 16; j++)
                    fprintf(stderr, j % 2 ? "  " : "   ");
            fprintf(stderr, "  ");
            for (int j = i / 16 * 16; j <= i; j++)
                if (mem[j] >= 0x20 && mem[j] <= 0x7e)
                    fprintf(stderr, "%c", mem[j]);
                else
                    fprintf(stderr, " ");
            if (i == size - 1)
                fprintf(stderr, "\n");
        }
    }
}

/**
 * @brief disassemble memory
 * @param mem pointer to working memory
 * @param size memory size
 */
void disasmem(const uint8_t *mem, uint64_t size)
{
    fprintf(stderr, "[Debug] Memory@%016lx:\n", (uint64_t)mem);
    const uint8_t *p = mem;
    while (p < mem + size)
        if (*p & 3 == 3)
            fprintf(stderr, "[Debug]     %08x: %08x %s\n",
                    uint32_t(p - mem), *(uint32_t *)p, disas(*(uint32_t *)p).c_str()),
                p += 4;
        else
            fprintf(stderr, "[Debug]     %08x:    %04hx %s\n",
                    uint32_t(p - mem), *(uint16_t *)p, disas(*(uint16_t *)p).c_str()),
                p += 2;
}

const char *gprname[64] = {
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6",
    "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7",
    "fs0", "fs1", "fa0", "fa1", "fa2", "fa3", "fa4", "fa5",
    "fa6", "fa7", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7",
    "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"};
std::map<uint16_t, const char *> csrname = {
    {0x000, "ustatus"},
    {0x001, "fflags"},
    {0x002, "frm"},
    {0x003, "fcsr"},
    {0x105, "stvec"},
    {0x106, "scounteren"},
    {0x140, "sscratch"},
    {0x141, "sepc"},
    {0x142, "scause"},
    {0x143, "stval"},
    {0x180, "satp"},
    {0x300, "mstatus"},
    {0x301, "misa"},
    {0x302, "medeleg"},
    {0x303, "mideleg"},
    {0x304, "mie"},
    {0x305, "mtvec"},
    {0x306, "mcounteren"},
    {0x320, "mcountinhibit"},
    {0x323, "mhpmevent3"},
    {0x324, "mhpmevent4"},
    {0x325, "mhpmevent5"},
    {0x326, "mhpmevent6"},
    {0x327, "mhpmevent7"},
    {0x328, "mhpmevent8"},
    {0x329, "mhpmevent9"},
    {0x32a, "mhpmevent10"},
    {0x32b, "mhpmevent11"},
    {0x32c, "mhpmevent12"},
    {0x32d, "mhpmevent13"},
    {0x32e, "mhpmevent14"},
    {0x32f, "mhpmevent15"},
    {0x330, "mhpmevent16"},
    {0x331, "mhpmevent17"},
    {0x332, "mhpmevent18"},
    {0x333, "mhpmevent19"},
    {0x334, "mhpmevent20"},
    {0x335, "mhpmevent21"},
    {0x336, "mhpmevent22"},
    {0x337, "mhpmevent23"},
    {0x338, "mhpmevent24"},
    {0x339, "mhpmevent25"},
    {0x33a, "mhpmevent26"},
    {0x33b, "mhpmevent27"},
    {0x33c, "mhpmevent28"},
    {0x33d, "mhpmevent29"},
    {0x33e, "mhpmevent30"},
    {0x33f, "mhpmevent31"},
    {0x340, "mscratch"},
    {0x341, "mepc"},
    {0x342, "mcause"},
    {0x343, "mtval"},
    {0x344, "mip"},
    {0x3a0, "pmpcfg0"},
    {0x3a1, "pmpcfg1"},
    {0x3a2, "pmpcfg2"},
    {0x3b0, "pmpaddr0"},
    {0x3b1, "pmpaddr1"},
    {0x3b2, "pmpaddr2"},
    {0x3b3, "pmpaddr3"},
    {0x3b4, "pmpaddr4"},
    {0x3b5, "pmpaddr5"},
    {0x3b6, "pmpaddr6"},
    {0x3b7, "pmpaddr7"},
    {0x3b8, "pmpaddr8"},
    {0x3b9, "pmpaddr9"},
    {0x3ba, "pmpaddr10"},
    {0x3bb, "pmpaddr11"},
    {0x3bc, "pmpaddr12"},
    {0x3bd, "pmpaddr13"},
    {0x3be, "pmpaddr14"},
    {0x3bf, "pmpaddr15"},
    {0x7a0, "tselect"},
    {0x7a1, "tdata1"},
    {0x7a2, "tdata2"},
    {0x7a3, "tdata3"},
    {0x7b0, "dcsr"},
    {0x7b1, "dpc"},
    {0x7b2, "dscratch0"},
    {0x7b3, "dscratch1"},
    {0xb00, "mcycle"},
    {0xb02, "minstret"},
    {0xb03, "mhpmcounter3"},
    {0xb04, "mhpmcounter4"},
    {0xb05, "mhpmcounter5"},
    {0xb06, "mhpmcounter6"},
    {0xb07, "mhpmcounter7"},
    {0xb08, "mhpmcounter8"},
    {0xb09, "mhpmcounter9"},
    {0xb0a, "mhpmcounter10"},
    {0xb0b, "mhpmcounter11"},
    {0xb0c, "mhpmcounter12"},
    {0xb0d, "mhpmcounter13"},
    {0xb0e, "mhpmcounter14"},
    {0xb0f, "mhpmcounter15"},
    {0xb10, "mhpmcounter16"},
    {0xb11, "mhpmcounter17"},
    {0xb12, "mhpmcounter18"},
    {0xb13, "mhpmcounter19"},
    {0xb14, "mhpmcounter20"},
    {0xb15, "mhpmcounter21"},
    {0xb16, "mhpmcounter22"},
    {0xb17, "mhpmcounter23"},
    {0xb18, "mhpmcounter24"},
    {0xb19, "mhpmcounter25"},
    {0xb1a, "mhpmcounter26"},
    {0xb1b, "mhpmcounter27"},
    {0xb1c, "mhpmcounter28"},
    {0xb1d, "mhpmcounter29"},
    {0xb1e, "mhpmcounter30"},
    {0xb1f, "mhpmcounter31"},
    {0xf11, "mvendorid"},
    {0xf12, "marchid"},
    {0xf13, "mimpid"},
    {0xf14, "mhartid"}};
