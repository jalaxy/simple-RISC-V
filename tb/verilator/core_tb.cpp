#include <cstdio>
#include <verilated.h>
#include "elf.h"
#include "Vcore_tb.h"
typedef struct memrec
{
    uint64_t addr, size;
    uint8_t *data, rw; // R/W -> 1,  RO -> 0
} memrec_t;
int main(int argc, char **argv)
{
    // First argument is the test ELF filename
    if (argc != 2)
        return printf("Not enough arguments.\n"), 0;

    // Read ELF file
    FILE *fp = fopen(argv[1], "r");
    if (!fp)
        return printf("File does not exist.\n"), 0;
    // Read and check ELF header, and print info
    Elf64_Ehdr elf_h; // ELF header
    int res = fread(&elf_h, sizeof(elf_h), 1, fp);
    if (!res)
        return printf("Unable to read file.\n"), 0;
    if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)))
        return printf("Wrong ELF format.\n"), 0;
    if (elf_h.e_type != ET_EXEC)
        return printf("Not an executable file.\n"), 0;
    if (elf_h.e_machine != EM_RISCV)
        return printf("Not RISC-V architecture.\n"), 0;
    printf("Version: 0x%x, Header size: %d, Entry: 0x%lx, Flags: 0x%x\n",
           elf_h.e_version, elf_h.e_ehsize, elf_h.e_entry, elf_h.e_flags);
    printf("Program header: offset %lu, size %hu, num %hu\n",
           elf_h.e_phoff, elf_h.e_phentsize, elf_h.e_phnum);
    printf("Section header: offset %lu, size %hu, num %hu\n",
           elf_h.e_shoff, elf_h.e_shentsize, elf_h.e_shnum);
    int mem_num = 2 + 0; // number of instruction sections
    // int mem_num = 2 + elf_h.e_shnum; // number of instruction sections
    // load and set data in memory
    memrec_t *pmem = new (std::nothrow) memrec_t[mem_num];
    if (!pmem)
        return printf("Memory allocation error.\n"), 0;
    pmem[0].addr = 0x0;
    pmem[0].data = NULL;
    pmem[0].rw = 0;
    pmem[0].size = 0;
    uint32_t reset_code[] = {
        0x00000537, // lui a0, 0x00000
        0x00050513, // addi a0, a0, 0x000
        0, 0, 0, 0, 0,
        0x00050067, // jalr zero, a0, 0
        0, 0, 0, 0, 0};
    reset_code[0] |= elf_h.e_entry & 0xfffff000;
    reset_code[1] |= (elf_h.e_entry & 0x00000fff) << 20;
    pmem[1].addr = 0x400000;
    pmem[1].data = new uint8_t[sizeof(reset_code)];
    pmem[1].size = sizeof(reset_code);
    pmem[1].rw = 0;
    memcpy(pmem[1].data, reset_code, sizeof(reset_code));
    fclose(fp);

    // Simulation
    Vcore_tb *tb = new (std::nothrow) Vcore_tb;
    if (!tb)
        return printf("Memory allocation error.\n"), 0;
    tb->rst = 0, tb->eval();
    tb->rst = 1, tb->clk = 0, tb->eval();
    for (int i = 0; i < 8; i++)
        tb->clk = !tb->clk, tb->eval();
    tb->rst = 0, tb->eval();
    unsigned char show[] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0}; // may be implemented by command arguments
    for (int i = 0; i < 16; i++)
    {
        // can be optimized with binary search
        int seg = 0;
        while (seg + 1 < mem_num && pmem[seg + 1].addr <= tb->icache_addr)
            seg++;
        int rel_addr = tb->icache_addr - pmem[seg].addr;
        if (rel_addr >= pmem[seg].size)
            return printf("Invalid instruction addr: 0x%08x\n", tb->icache_addr), 0;
        uint32_t icache_data = *(uint32_t *)(pmem[seg].data + rel_addr);
        tb->icache_valid = tb->dcache_valid = 1;
        printf("cycle %d:\n    pc: 0x%08x\n    ir: 0x%08x\n", i, tb->pc, icache_data);
        for (int i = 0; i < 32; i++)
            if (show[i])
                printf("    x%d: 0x%08x\n", i, tb->gpr[i]);
        tb->clk = 0, tb->eval();
        tb->clk = 1, tb->eval();
        if (tb->icache_ena)
            tb->icache_data = icache_data;
        // if (tb->dcache_r_ena)
        //     tb->dcache_data_out = arr[(tb->dcache_addr - 0x10010000) >> 2];
        // if (tb->dcache_w_ena)
        //     arr[(tb->dcache_addr - 0x10010000) >> 2] = tb->dcache_data_in;
    }

    // Clean
    delete tb;
    for (int i = 0; i < mem_num; i++)
        delete[] pmem[i].data;
    delete[] pmem;
    return 0;
}
