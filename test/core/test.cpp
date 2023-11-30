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
    // Read command line
    int option_src = 0, total_time = INT32_MAX;
    int show_pc = 0, show_reg[32], show_reg_flag = 0;
    memset(show_reg, 0, sizeof(show_reg));
    const char *filename = NULL;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] == '-')
        {
            int j = 1;
            while (argv[i][j] == '-')
                j++;
            if (strcmp(argv[i] + j, "dump") == 0)
                option_src = 0;
            else if (strcmp(argv[i] + j, "elf") == 0)
                option_src = 1;
            else if (strcmp(argv[i] + j, "t") == 0)
            {
                if (i + 1 < argc)
                    total_time = atoi(argv[i + 1]);
                if (total_time <= 0)
                    total_time = 1024;
                i++;
            }
            else if (strcmp(argv[i] + j, "pc") == 0)
                show_pc = 1;
            else if (strcmp(argv[i] + j, "r") == 0)
            {
                show_reg_flag = 1;
                int all = 1;
                while (i + 1 < argc && atoi(argv[i + 1]) > 0 && atoi(argv[i + 1]) < 32)
                    show_reg[atoi(argv[++i])] = 1, all = 0;
                if (all)
                    for (int i = 0; i < 32; i++)
                        show_reg[i] = 1;
            }
        }
        else if (filename == NULL)
            filename = argv[i];
    if (filename == NULL)
        return printf("Not enough arguments.\n"), 0;

    // Load and set reset code in memory
    std::map<uint64_t, memrec_t> memory;
    // start section: jump from reset address to ELF entry
    uint32_t elf_rst_code[] = {
        0x00000537, // lui a0, 0x00000
        0x00050513, // addi a0, a0, 0x000
        NOP, NOP, NOP, NOP, NOP,
        0x00050067, // jalr zero, a0, 0
        NOP, NOP, NOP, NOP, NOP};
    uint32_t *rst_code;
    int rst_code_len = 0;
    if (option_src == 0)
    {
        FILE *fp = fopen(filename, "r");
        if (!fp)
            return printf("Unable to open file.\n"), 0;
        fseek(fp, 0, SEEK_END);
        rst_code_len = ftell(fp) + 32;
        rst_code = new (std::nothrow) uint32_t[rst_code_len];
        if (rst_code == NULL)
            return printf("Memory allocation error.\n"), 0;
        rewind(fp);
        for (int i = 0; i < rst_code_len; i++)
            rst_code[i] = NOP;
        int i_dump = 0;
        while (i_dump < rst_code_len && fscanf(fp, "%x", &rst_code[i_dump++]) > 0)
            ;
    }
    else if (option_src == 1)
    {
        rst_code_len = sizeof(elf_rst_code) / sizeof(elf_rst_code[0]);
        rst_code = new (std::nothrow) uint32_t[rst_code_len];
        if (rst_code == NULL)
            return printf("Memory allocation error.\n"), 0;
        memcpy(rst_code, elf_rst_code, rst_code_len * sizeof(uint32_t));
    }

    // Read and check ELF header, and print info
    if (option_src == 1)
    {
        FILE *fp = fopen(filename, "r");
        if (!fp)
            return printf("Unable to open file.\n"), 0;
        Elf32_Ehdr elf_h; // ELF header
        int ret = fread(&elf_h, sizeof(elf_h), 1, fp);
        if (!ret)
            return printf("Unable to read file.\n"), 0;
        if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)))
            return printf("Wrong ELF format.\n"), 0;
        if (elf_h.e_type != ET_EXEC)
            return printf("Not an executable file.\n"), 0;
        if (elf_h.e_machine != EM_RISCV)
            return printf("Not RISC-V architecture.\n"), 0;
        rst_code[0] |= (elf_h.e_entry + ((elf_h.e_entry & 0x00000800) << 1)) & 0xfffff000;
        rst_code[1] |= (elf_h.e_entry & 0x00000fff) << 20;
        // sections from ELF file
        Elf32_Shdr *shdr;
        shdr = new (std::nothrow) Elf32_Shdr[elf_h.e_shnum]; // section headers
        ret = fseek(fp, elf_h.e_shoff, SEEK_SET);
        if (ret == -1)
            return printf("Fseek error.\n"), 0;
        ret = fread(shdr, sizeof(Elf32_Shdr) * elf_h.e_shnum, 1, fp);
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
                    uint32_t addr = shdr[i].sh_addr + j;
                    memory[addr] = {.rw = 0};
                    ret = fread(&memory[addr].data, 1, 1, fp);
                    if (!ret)
                        ; // exceed file size maybe and it does not matter
                }
            }
        delete[] shdr;
        fclose(fp);
    }
    for (int i = 0; i < rst_code_len; i++)
        for (int j = 0; j < 4; j++) // reset address is 0x400000
            memory[0x400000 + i * 4 + j] = {.data = WTOB(rst_code[i], j), .rw = 0};
    delete[] rst_code;

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
    for (int i = 0; i < total_time; i++)
    {
        // print info
        if (show_pc || show_reg_flag)
            printf("cycle %d:\n", i);
        if (show_pc)
            printf("    pc: 0x%08x\n", dut->pc);
        for (int i = 0; i < 32; i++)
            if (show_reg[i])
                printf("    x%d: 0x%08x\n", i, dut->gpr[i]);
        // before posedge clock read signals
        uint32_t icache_data, dcache_data;
        if (dut->icache_ena &&
            memory.count(dut->icache_addr + 0) && memory.count(dut->icache_addr + 1) &&
            memory.count(dut->icache_addr + 2) && memory.count(dut->icache_addr + 3))
            icache_data = BTOW(memory[dut->icache_addr + 0].data, memory[dut->icache_addr + 1].data,
                               memory[dut->icache_addr + 2].data, memory[dut->icache_addr + 3].data);
        else if (dut->icache_ena)
        {
            printf("Invalid instruction addr: 0x%08x\n", dut->icache_addr);
            break;
        }
        if (dut->dcache_r_ena &&
            memory.count(dut->dcache_addr + 0) && memory.count(dut->dcache_addr + 1) &&
            memory.count(dut->dcache_addr + 2) && memory.count(dut->dcache_addr + 3))
            dcache_data = BTOW(memory[dut->dcache_addr + 0].data, memory[dut->dcache_addr + 1].data,
                               memory[dut->dcache_addr + 2].data, memory[dut->dcache_addr + 3].data);
        else if (dut->dcache_r_ena)
        {
            printf("Invalid data addr: 0x%08x\n", dut->dcache_addr);
            break;
        }
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
    trace->close();
    delete trace;
    delete dut;
    return 0;
}
