#include <cstdio>
#include <map>
#include <algorithm>
#include <verilated.h>
#include "elf.h"
#include "Vcore_tb.h"
#define BTOW(b0, b1, b2, b3) (((uint8_t)(b0)) | (((uint8_t)(b1)) << 8) | (((uint8_t)(b2)) << 16) | (((uint8_t)(b3)) << 24))
#define WTOB(x, i) ((uint8_t)(0xff & ((x) >> (8 * ((uint32_t)(i))))))
typedef struct memrec
{
    uint8_t data, rw; // R/W -> 1,  RO -> 0
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
    int ret = fread(&elf_h, sizeof(elf_h), 1, fp);
    if (!ret)
        return printf("Unable to read file.\n"), 0;
    if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)))
        return printf("Wrong ELF format.\n"), 0;
    if (elf_h.e_type != ET_EXEC)
        return printf("Not an executable file.\n"), 0;
    if (elf_h.e_machine != EM_RISCV)
        return printf("Not RISC-V architecture.\n"), 0;
    // load and set data in memory
    std::map<uint64_t, memrec_t> memory;
    // start section: jump from reset address to ELF entry
    // uint32_t rst_code[] = {
    //     0x00000537, // lui a0, 0x00000
    //     0x00050513, // addi a0, a0, 0x000
    //     0, 0, 0, 0, 0,
    //     0x00050067, // jalr zero, a0, 0
    //     0, 0, 0, 0, 0};
    // rst_code[0] |= elf_h.e_entry & 0xfffff000;
    // rst_code[1] |= (elf_h.e_entry & 0x00000fff) << 20;
    uint32_t rst_code[] = {
        0x10010137,
        0x10010113,
        0xfe010113,
        0x00812e23,
        0x02010413,
        0xfe042623,
        0x06400793,
        0xfef42223,
        0x00100793,
        0xfef42423,
        0x0380006f,
        0xfe842783,
        0x0017f793,
        0x00079863,
        0xfe842783,
        0x00179793,
        0x0080006f,
        0xfe842783,
        0xfec42703,
        0x00f707b3,
        0xfef42623,
        0xfe842783,
        0x00178793,
        0xfef42423,
        0xfe842703,
        0xfe442783,
        0xfce7d2e3,
        0xfec42783,
        0x00f00533,
        0x01c12403,
        0x02010113,
        0x0000006f,
        0x13, 0x13, 0x13, 0x13, 0x13};
    uint32_t rst_addr;
    for (int i = 0; i < sizeof(rst_code) / sizeof(uint32_t); i++)
        for (int j = 0; j < 4; j++)
            memory[0x400000 + i * 4 + j] = {.data = WTOB(rst_code[i], j), .rw = 0};
    // sections from ELF file
    Elf64_Shdr *shdr = new (std::nothrow) Elf64_Shdr[elf_h.e_shnum]; // section headers
    ret = fseek(fp, elf_h.e_shoff, SEEK_SET);
    if (ret == -1)
        return printf("Fseek error.\n"), 0;
    ret = fread(shdr, sizeof(Elf64_Shdr) * elf_h.e_shnum, 1, fp);
    if (!ret)
        return printf("Unable to read file.\n"), 0;
    for (int i = 0; i < elf_h.e_shnum; i++)
        if (shdr[i].sh_addr) // with an address to bind
        {
            ret = fseek(fp, shdr[i].sh_offset, SEEK_SET);
            if (ret == -1)
                return printf("Fseek error.\n"), 0;
            for (int j = 0; j < shdr[i].sh_size; j++)
            {
                uint64_t addr = shdr[i].sh_addr + j;
                memory[addr] = {.rw = 0};
                ret = fread(&memory[addr].data, 1, 1, fp);
                if (!ret)
                    return printf("Unable to read file.\n"), 0;
            }
        }
    delete[] shdr;
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
        0, 0, 1, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0}; // may be implemented by command arguments
    // memset(show, 0xff, sizeof(show));
    for (int i = 0; i < 1024; i++)
    {
        // print info
        printf("cycle %d:\n    pc: 0x%08x\n", i, tb->pc);
        for (int i = 0; i < 32; i++)
            if (show[i])
                printf("    x%d: 0x%08x\n", i, tb->gpr[i]);
        for (int addr = 0x100100e4; addr < 0x100100f0; addr += 4)
            printf("    %08x\n", BTOW(memory[addr + 0].data, memory[addr + 1].data,
                                      memory[addr + 2].data, memory[addr + 3].data));
        // negedge clock
        tb->clk = 0, tb->eval();
        // before posedge clock read signals
        uint32_t icache_data, dcache_data;
        if (memory.count(tb->icache_addr) && memory.count(tb->icache_addr + 1) &&
            memory.count(tb->icache_addr + 2) && memory.count(tb->icache_addr + 3))
            icache_data = BTOW(memory[tb->icache_addr + 0].data, memory[tb->icache_addr + 1].data,
                               memory[tb->icache_addr + 2].data, memory[tb->icache_addr + 3].data);
        else if (tb->icache_ena)
            return printf("Invalid instruction addr: 0x%08x\n", tb->icache_addr), 0;
        if (memory.count(tb->dcache_addr) && memory.count(tb->dcache_addr + 1) &&
            memory.count(tb->dcache_addr + 2) && memory.count(tb->dcache_addr + 3))
            dcache_data = BTOW(memory[tb->dcache_addr + 0].data, memory[tb->dcache_addr + 1].data,
                               memory[tb->dcache_addr + 2].data, memory[tb->dcache_addr + 3].data);
        else if (tb->dcache_r_ena)
            return printf("Invalid data addr: 0x%08x\n", tb->icache_addr), 0;
        tb->icache_valid = tb->dcache_valid = 1;
        printf("    dcache_addr: 0x%08x\n", tb->dcache_addr);
        if (tb->dcache_w_ena)
            for (int j = 0; j < 4; j++)
                memory[tb->dcache_addr + j] = {.data = WTOB(tb->dcache_data_in, j), .rw = 1};
        // posedge clock
        tb->clk = 1, tb->eval();
        // after posedge clock set registers
        if (tb->icache_ena)
            tb->icache_data = icache_data;
        if (tb->dcache_r_ena)
            tb->dcache_data_out = dcache_data;
    }

    // Clean
    delete tb;
    return 0;
}
