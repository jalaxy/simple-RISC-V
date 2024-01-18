#include <cstdio>
#include <map>
#include <algorithm>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include <elf.h>
#include "Vtest.h"

#define BTOD(b0, b1, b2, b3, b4, b5, b6, b7)                                     \
    ((((uint8_t)(b0)) << (uint64_t)0x00) | (((uint8_t)(b1)) << (uint64_t)0x08) | \
     (((uint8_t)(b2)) << (uint64_t)0x10) | (((uint8_t)(b3)) << (uint64_t)0x18) | \
     (((uint8_t)(b4)) << (uint64_t)0x10) | (((uint8_t)(b5)) << (uint64_t)0x18) | \
     (((uint8_t)(b6)) << (uint64_t)0x10) | (((uint8_t)(b7)) << (uint64_t)0x18))
#define DTOB(x, i) ((uint8_t)(0xff & ((x) >> (8 * ((uint64_t)(i))))))
#define BTOW(b0, b1, b2, b3) ((uint32_t)BTOD(b0, b1, b2, b3, 0, 0, 0, 0))
#define NOP ((uint64_t)0x13)

typedef struct struct_cmd
{
    const char *filename = 0;
    std::vector<const char *> args;
    uint8_t filetype = 0, showpc = 0, vcd = 1;
    uint32_t showreg = 0;
    int simtime = INT32_MAX;
} cmd_t;

typedef struct struct_htif
{
    uint64_t fromhost = 0, tohost = 0, lock = 0;
} htif_t;

void dumpmem(std::map<uint64_t, uint8_t> &mem, uint64_t addr, uint64_t size)
{
    printf("Memory@%08lx:", addr);
    for (int i = 0; i < size; i++)
    {
        i % 16 ? printf(i % 2 ? "" : " ") : printf("\n%08lx: ", addr + i);
        printf("%02x", mem[addr + i]);
        if ((i + 1) % 16 == 0 || i == size - 1)
        {
            if (i == size - 1)
                for (int j = i + 1; j < i / 16 * 16 + 16; j++)
                    printf(j % 2 ? "  " : "   ");
            printf("  ");
            for (int j = i / 16 * 16; j <= i; j++)
                if (mem[addr + j] >= 0x20 && mem[addr + j] <= 0x7e)
                    printf("%c", mem[addr + j]);
                else
                    printf(" ");
            if (i == size - 1)
                printf("\n");
        }
    }
}

int main(int argc, char **argv)
{
    // Read command line
    cmd_t cmd;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] == '-' && cmd.filename == NULL)
        {
            int j = 1;
            while (argv[i][j] == '-')
                j++;
            if (strcmp(argv[i] + j, "dump") == 0)
                cmd.filetype = 0;
            else if (strcmp(argv[i] + j, "elf") == 0)
                cmd.filetype = 1;
            else if (strcmp(argv[i] + j, "no-vcd") == 0)
                cmd.vcd = 0;
            else if (strcmp(argv[i] + j, "t") == 0)
            {
                if (i + 1 < argc)
                    cmd.simtime = atoi(argv[i + 1]);
                if (cmd.simtime <= 0)
                    cmd.simtime = 1024;
                i++;
            }
            else if (strcmp(argv[i] + j, "pc") == 0)
                cmd.showpc = 1;
            else if (strcmp(argv[i] + j, "r") == 0)
            {
                int all = 1;
                while (i + 1 < argc && atoi(argv[i + 1]) > 0 && atoi(argv[i + 1]) < 32)
                    cmd.showreg |= 1u << atoi(argv[++i]), all = 0;
                if (all)
                    cmd.showreg = -1;
            }
        }
        else if (cmd.filename == NULL)
            cmd.filename = argv[i];
        else
            cmd.args.push_back(argv[i]);
    if (cmd.filename == NULL)
        return printf("Not enough arguments.\n"), 0;
    printf("Running simulation in %s mode with:\n    %s",
           cmd.filetype == 0 ? "dump" : "elf", cmd.filename);
    for (int i = 0; i < cmd.args.size(); i++)
        printf(" %s", cmd.args[i]);
    printf("\n");

    // Load and set reset code in memory
    std::map<uint64_t, uint8_t> memory;
    std::vector<uint32_t> ini_code;
    htif_t htif;
    FILE *fp = fopen(cmd.filename, "r");
    if (!fp)
        return printf("Unable to open file %s.\n", cmd.filename), 0;
    if (cmd.filetype == 0) // direct dumped hex code
    {
        int code;
        while (fscanf(fp, "%x", &code) > 0)
            ini_code.push_back(code);
    }
    else if (cmd.filetype == 1) // code in ELF file
    {
        // Read and check ELF header, and print info
        Elf64_Ehdr elf_h; // ELF header
        if (fread(&elf_h, sizeof(elf_h), 1, fp) < 0)
            exit((perror("fread"), 1));
        if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)) ||
            elf_h.e_ident[EI_CLASS] != ELFCLASS64)
            return printf("Not 64-bit ELF format.\n"), 0;
        if (elf_h.e_type != ET_EXEC && elf_h.e_type != ET_DYN)
            return printf("Not an executable file.\n"), 0;
        if (elf_h.e_machine != EM_RISCV)
            return printf("Not RISC-V architecture.\n"), 0;
        // sections from ELF file
        Elf64_Shdr *shdr = new (std::nothrow) Elf64_Shdr[elf_h.e_shnum]; // section headers
        fseek(fp, elf_h.e_shoff, SEEK_SET);
        if (fread(shdr, sizeof(Elf64_Shdr) * elf_h.e_shnum, 1, fp) < 0)
            exit((perror("fread"), 1));
        for (int i = 0; i < elf_h.e_shnum; i++)
            if (shdr[i].sh_type == SHT_PROGBITS)
            {
                fseek(fp, shdr[i].sh_offset, SEEK_SET);
                for (int j = 0; j < shdr[i].sh_size; j++)
                    if (fread(&memory[shdr[i].sh_addr + j], 1, 1, fp) < 0)
                        exit((perror("fread"), 1));
            }
            else if (shdr[i].sh_type == SHT_NOBITS)
                for (int j = 0; j < shdr[i].sh_size; j++)
                    memory[shdr[i].sh_addr + j] = 0;
            else if (shdr[i].sh_type == SHT_SYMTAB)
            {
                // check section name
                char name[1024];
                fseek(fp, shdr[elf_h.e_shstrndx].sh_offset + shdr[i].sh_name, SEEK_SET);
                if (fscanf(fp, "%1023s", name) <= 0)
                    exit((perror("fscanf"), 1));
                if (strcmp(name, ".symtab") != 0)
                    continue;
                // read symbol table and search for fromhost and tohost
                int sym_sz = shdr[i].sh_size / shdr[i].sh_entsize;
                for (int j = 0; j < sym_sz; j++)
                {
                    Elf64_Sym sym;
                    fseek(fp, shdr[i].sh_offset + j * shdr[i].sh_entsize, SEEK_SET);
                    if (fread(&sym, sizeof(sym), 1, fp) < 0)
                        exit((perror("fread"), 1));
                    fseek(fp, shdr[shdr[i].sh_link].sh_offset + sym.st_name, SEEK_SET);
                    if (fscanf(fp, "%1023s", name) <= 0)
                        exit((perror("fscanf"), 1));
                    if (strcmp(name, "fromhost") == 0)
                        htif.fromhost = sym.st_value;
                    else if (strcmp(name, "tohost") == 0)
                        htif.tohost = sym.st_value;
                    else if (strcmp(name, "htif_lock") == 0)
                        htif.lock = sym.st_value;
                }
            }
        if (htif.fromhost == 0 || htif.tohost == 0 || htif.lock == 0)
        {
            htif = {0x100000, 0x100008, 0x100010}; // default htif addresses
            printf("HTIF address not specified, set to default.\n");
        }
        delete[] shdr;
        fclose(fp);
        // start section: jump from reset address to ELF entry
        ini_code.push_back(0x93); // addi ra, zero, 0
        for (int i = 0; i < 8; i++)
        {
            ini_code.push_back(0x8093 | (DTOB(elf_h.e_entry, 7 - i) << 20));
            ini_code.push_back(i == 7 ? 0x8067 : 0x809093); // ret : slli ra, ra, 8
        }
    }
    for (int i = 0; i < 32; i++)
        ini_code.push_back(NOP);
    for (int i = 0; i < ini_code.size(); i++)
        for (int j = 0; j < 4; j++) // reset address is 0x400000
            memory[0x400000 + i * 4 + j] = DTOB(ini_code[i], j);
    dumpmem(memory, 0x400000, 128);

    // Simulation
    Vtest *dut = new (std::nothrow) Vtest;
    VerilatedVcdC *trace = NULL;
    if (!dut)
        return printf("Memory allocation error.\n"), 0;
    if (cmd.vcd)
    {
        Verilated::traceEverOn(true); // trace waveform
        dut->trace(trace = new (std::nothrow) VerilatedVcdC, 5);
        trace->open("waveform.vcd");
    }
    int st = 0; // simulation time
    dut->rst = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    dut->rst = 1, dut->clk = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    for (int i = 0; i < 8; i++)
        dut->clk = !dut->clk, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    dut->rst = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    for (int i = 0; i < cmd.simtime; i++)
    {
        // print info
        if (cmd.showpc || cmd.showreg)
            printf("cycle %d:\n", i);
        if (cmd.showpc)
            printf("    pc: 0x%08x\n", dut->pc);
        for (int i = 0; i < 32; i++)
            if (cmd.showreg & (1u << i))
                printf("    x%d: 0x%08x\n", i, dut->gpr[i]);
        // before posedge clock read signals
        uint32_t icache_data, dcache_data;
        if (dut->icache_ena)
            icache_data = BTOW(memory[dut->icache_addr + 0], memory[dut->icache_addr + 1],
                               memory[dut->icache_addr + 2], memory[dut->icache_addr + 3]);
        if (dut->dcache_r_ena)
            dcache_data = BTOW(memory[dut->dcache_addr + 0], memory[dut->dcache_addr + 1],
                               memory[dut->dcache_addr + 2], memory[dut->dcache_addr + 3]);
        if (dut->dcache_w_ena)
            for (int j = 0; j < 4; j++)
                memory[dut->dcache_addr + j] = DTOB(dut->dcache_data_in, j);
        // posedge clock
        dut->clk = 1, dut->eval(), trace ? trace->dump(st++), 0 : 0;
        // after posedge clock set registers
        dut->icache_valid = dut->dcache_valid = 1;
        dut->icache_data = icache_data;
        dut->dcache_data_out = dcache_data;
        // negedge clock
        dut->clk = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    }

    // Clean
    trace ? trace->close(), 0 : 0;
    delete trace;
    delete dut;
    return 0;
}
