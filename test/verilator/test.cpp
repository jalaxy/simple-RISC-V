#include <cstdio>
#include <map>
#include <algorithm>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "elf.h"
#include "Vtest.h"
#define BTOW(b0, b1, b2, b3) (((uint8_t)(b0)) | (((uint8_t)(b1)) << 8) | (((uint8_t)(b2)) << 16) | (((uint8_t)(b3)) << 24))
#define WTOB(x, i) ((uint8_t)(0xff & ((x) >> (8 * ((uint32_t)(i))))))
#define NOP ((uint32_t)0x13u)
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
    //     NOP, NOP, NOP, NOP, NOP,
    //     0x00050067, // jalr zero, a0, 0
    //     NOP, NOP, NOP, NOP, NOP};
    // rst_code[0] |= elf_h.e_entry & 0xfffff000;
    // rst_code[1] |= (elf_h.e_entry & 0x00000fff) << 20;
    uint32_t rst_code[1024];
    for (int i = 0; i < 1024; i++)
        rst_code[i] = NOP;
    FILE *fp_dump = fopen("test.dump", "r");
    int i_dump = 0;
    while (fscanf(fp_dump, "%x", &rst_code[i_dump++]) != EOF)
        ;
    for (int i = 0; i < sizeof(rst_code) / sizeof(uint32_t); i++)
        for (int j = 0; j < 4; j++) // reset address is 0x400000
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
    Vtest *dut = new (std::nothrow) Vtest;
    if (!dut)
        return printf("Memory allocation error.\n"), 0;
    Verilated::traceEverOn(true); // trace waveform
    VerilatedVcdC *trace = new VerilatedVcdC;
    dut->trace(trace, 5);
    trace->open("waveform.vcd");
    int st = 0; // simulation time
    dut->rst = 0, dut->eval(), trace->dump(st++);
    dut->rst = 1, dut->clk = 0, dut->eval(), trace->dump(st++);
    for (int i = 0; i < 8; i++)
        dut->clk = !dut->clk, dut->eval(), trace->dump(st++);
    dut->rst = 0, dut->eval(), trace->dump(st++);
    unsigned char show[] = {
        0, 0, 1, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0}; // may be implemented by command arguments
    memset(show, 0xff, sizeof(show));
    for (int i = 0; i < 4096; i++)
    {
        // print info
        printf("cycle %d:\n    pc: 0x%08x\n", i, dut->pc);
        for (int i = 0; i < 32; i++)
            if (show[i])
                printf("    x%d: 0x%08x\n", i, dut->gpr[i]);
        for (int addr = 0x100100e4; addr < 0x100100f0; addr += 4)
            printf("    0x%08x\n", BTOW(memory[addr + 0].data, memory[addr + 1].data,
                                        memory[addr + 2].data, memory[addr + 3].data));
        // before posedge clock read signals
        uint32_t icache_data, dcache_data;
        if (dut->icache_ena &&
            memory.count(dut->icache_addr + 0) && memory.count(dut->icache_addr + 1) &&
            memory.count(dut->icache_addr + 2) && memory.count(dut->icache_addr + 3))
            icache_data = BTOW(memory[dut->icache_addr + 0].data, memory[dut->icache_addr + 1].data,
                               memory[dut->icache_addr + 2].data, memory[dut->icache_addr + 3].data);
        else if (dut->icache_ena)
            return printf("Invalid instruction addr: 0x%08x\n", dut->icache_addr), 0;
        if (dut->dcache_r_ena &&
            memory.count(dut->dcache_addr + 0) && memory.count(dut->dcache_addr + 1) &&
            memory.count(dut->dcache_addr + 2) && memory.count(dut->dcache_addr + 3))
            dcache_data = BTOW(memory[dut->dcache_addr + 0].data, memory[dut->dcache_addr + 1].data,
                               memory[dut->dcache_addr + 2].data, memory[dut->dcache_addr + 3].data);
        else if (dut->dcache_r_ena)
            return printf("Invalid data addr: 0x%08x\n", dut->icache_addr), 0;
        if (dut->dcache_w_ena)
            for (int j = 0; j < 4; j++)
                memory[dut->dcache_addr + j] = {.data = WTOB(dut->dcache_data_in, j), .rw = 1};
        // posedge clock
        dut->clk = 1, dut->eval(), trace->dump(st++);
        // after posedge clock set registers
        dut->icache_valid = dut->dcache_valid = 1;
        dut->icache_data = icache_data;
        dut->dcache_data_out = dcache_data;
        // negedge clock
        dut->clk = 0, dut->eval(), trace->dump(st++);
    }

    // Clean
    delete dut;
    trace->close();
    delete trace;
    return 0;
}
