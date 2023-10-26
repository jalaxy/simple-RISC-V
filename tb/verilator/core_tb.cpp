#include <cstdio>
#include <verilated.h>
#include "elf.h"
#include "Vcore_tb.h"
extern uint32_t inst[], arr[];
typedef struct memrec
{
    uint64_t addr, size;
    uint8_t *data;
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
    Elf64_Ehdr elf_h; // ELF header
    int res = fread(&elf_h, sizeof(elf_h), 1, fp);
    if (!res)
        return printf("Unable to read file.\n"), 0;
    if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)))
        return printf("Wrong ELF format.\n"), 0;
    for (int i = 0; i < 16; i++)
        printf("%x ", elf_h.e_ident[i]);
    printf("\n");
    int mem_num;
    // load data in memory
    memrec_t *pmem = new (std::nothrow) memrec_t[mem_num];
    if (!pmem)
        return printf("Memory allocation error.\n"), 0;
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
    for (int i = 0; i < 0; i++)
    {
        unsigned int icache_data = inst[(tb->icache_addr - 0x400000) >> 2];
        tb->icache_valid = tb->dcache_valid = 1;
        printf("cycle %d:\n    pc: 0x%08x\n", i, tb->pc);
        for (int i = 0; i < 32; i++)
            printf("    x%d: 0x%08x\n", i, tb->gpr[i]);
        tb->clk = 0, tb->eval();
        tb->clk = 1, tb->eval();
        if (tb->icache_ena)
            tb->icache_data = icache_data;
        if (tb->dcache_r_ena)
            tb->dcache_data_out = arr[(tb->dcache_addr - 0x10010000) >> 2];
        if (tb->dcache_w_ena)
            arr[(tb->dcache_addr - 0x10010000) >> 2] = tb->dcache_data_in;
    }

    // Clean
    delete tb;
    for (int i = 0; i < mem_num; i++)
        delete[] pmem[i].data;
    delete[] pmem;
    return 0;
}
unsigned int arr[1024],
    inst[] = {
        0x000000b3,
        0x00000133,
        0x02000193,
        0x00108093,
        0x00210113,
        0xfe30cce3,
        0x0040006f,
        0x00408093,
        0x0040026f,
        0x0000006f,
        0x00000013};