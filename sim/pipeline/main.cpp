#include <cstdio>
#include <map>
#include <queue>
#include <algorithm>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include <elf.h>
#include "Vstats.h"
#include "simulator.h"

#define BTOD(b0, b1, b2, b3, b4, b5, b6, b7)                                                         \
    ((((uint64_t)(uint8_t)(b0)) << (uint64_t)0x00) | (((uint64_t)(uint8_t)(b1)) << (uint64_t)0x08) | \
     (((uint64_t)(uint8_t)(b2)) << (uint64_t)0x10) | (((uint64_t)(uint8_t)(b3)) << (uint64_t)0x18) | \
     (((uint64_t)(uint8_t)(b4)) << (uint64_t)0x20) | (((uint64_t)(uint8_t)(b5)) << (uint64_t)0x28) | \
     (((uint64_t)(uint8_t)(b6)) << (uint64_t)0x30) | (((uint64_t)(uint8_t)(b7)) << (uint64_t)0x38))
#define DTOB(x, i) ((uint8_t)((x) >> (8 * (uint64_t)(i))))
#define DLE(pb, addr) (BTOD((pb)[(addr) + 0], (pb)[(addr) + 1], (pb)[(addr) + 2], (pb)[(addr) + 3], \
                            (pb)[(addr) + 4], (pb)[(addr) + 5], (pb)[(addr) + 6], (pb)[(addr) + 7]))
#define NOP ((uint64_t)0x13)

typedef struct struct_cmd
{
    const char *filename = 0, *vcd = 0;
    std::vector<const char *> args;
    uint8_t help = 0, filetype = 0, debug = 0, step = 0;
    int simtime = INT32_MAX;
} cmd_t;

typedef struct struct_htif
{
    uint64_t fromhost = 0, tohost = 0, lock = 0;
} htif_t;

typedef struct struct_icache_req
{
    uint8_t rqst = 0;
    uint64_t addr = 0;
} icache_req_t;

typedef struct struct_dcache_req
{
    uint8_t rqst = 0, bits = 0, wena = 0;
    uint64_t addr = 0, wdata = 0;
} dcache_req_t;

typedef struct struct_store
{
    uint64_t addr, data;
    uint8_t width;
} store_t;

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
            else if (strcmp(argv[i] + j, "w") == 0)
                cmd.vcd = argv[++i];
            else if (strcmp(argv[i] + j, "t") == 0)
            {
                if (i + 1 < argc)
                    cmd.simtime = atoi(argv[i + 1]);
                if (cmd.simtime <= 0)
                    cmd.simtime = 1024;
                i++;
            }
            else if (strcmp(argv[i] + j, "d") == 0)
                cmd.debug = 1;
            else if (strcmp(argv[i] + j, "s") == 0)
                cmd.step = 1;
            else if (strcmp(argv[i] + j, "h") == 0)
                cmd.help = 1;
        }
        else if (cmd.filename == NULL)
            cmd.filename = argv[i];
        else
            cmd.args.push_back(argv[i]);
    if (!cmd.help && cmd.filename == NULL)
        printf("Not enough arguments.\n"), cmd.help = 1;
    if (cmd.help)
    {
        printf("Usage: exec [options] file\n");
        printf("Available options:\n");
        printf("    -dump: (default) input file as hex hump\n");
        printf("    -elf: (force) input file as RISC-V ELF executable\n");
        printf("    -w `waveform`: output waveform to `waveform`\n");
        printf("    -t `time`: maximum simulation time of `time`\n");
        printf("    -d: debug mode\n");
        printf("    -d -s: debug mode with step\n");
        return 0;
    }
    if (cmd.debug)
    {
        printf("Running simulation in %s mode with:\n    %s",
               cmd.filetype == 0 ? "dump" : "elf", cmd.filename);
        for (int i = 0; i < cmd.args.size(); i++)
            printf(" %s", cmd.args[i]);
        printf("\n");
        if (cmd.vcd)
            printf("Recording waveform in file: %s\n", cmd.vcd);
    }

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

    // Simulation
    simulator *sim = cmd.debug ? new (std::nothrow) simulator(0x400000, memory) : 0;
    Vstats *dut = new (std::nothrow) Vstats;
    VerilatedVcdC *trace = NULL;
    if (cmd.vcd)
    {
        Verilated::traceEverOn(true); // trace waveform
        dut->trace(trace = new (std::nothrow) VerilatedVcdC, 5);
        trace->open(cmd.vcd);
    }
    int st = 0; // simulation time
    // reset
    dut->rst = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    dut->rst = 1, dut->clk = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    for (int i = 0; i < 8; i++)
        dut->clk = !dut->clk, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    dut->rst = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
    // clock and memory loop
    std::queue<icache_req_t> i_delay;
    std::queue<dcache_req_t> d_delay;
    std::queue<store_t> stores;
    for (int i = 0; i < cmd.simtime; i++)
    {
        // negedge clock
        dut->clk = 0, dut->eval(), trace ? trace->dump(st++), 0 : 0;
        // posedge clock
        if (dut->icache_rqst) // record stats before posedge
        {
            // if (i == 0004) // delay for some cycles in some conditions
            //     for (int i = 0; i < 0004; i++)
            //         i_delay.push({0, 0});
            i_delay.push({1, dut->icache_addr});
        }
        if (dut->dcache_rqst)
            d_delay.push({dut->dcache_rqst, dut->dcache_bits, dut->dcache_wena,
                          dut->dcache_addr, dut->dcache_wdat});
        dut->clk = 1, dut->eval(); // clock changes first
        i_delay.empty() ? i_delay.push({0, 0}), 0 : 0;
        d_delay.empty() ? d_delay.push({0, 0}), 0 : 0;
        dut->icache_done = i_delay.front().rqst; // other signals change after clk
        if (i_delay.front().rqst)
            dut->icache_data = DLE(memory, i_delay.front().addr);
        dut->dcache_done = d_delay.front().rqst;
        if (d_delay.front().rqst && !d_delay.front().wena)
            dut->dcache_rdat = DLE(memory, d_delay.front().addr);
        // bits width (funct3) decode: 00b -> 8  01b -> 16  10b -> 32  11b -> 64
        uint64_t bitwidth = 8 * (1 << (d_delay.front().bits & 3));
        uint64_t mask = bitwidth < 64 ? (1llu << bitwidth) - 1 : ~0llu;
        dut->dcache_rdat &= mask;
        if (((1 << bitwidth - 1) & dut->dcache_rdat) && !(d_delay.front().bits >> 2))
            dut->dcache_rdat |= ~mask; // msb = 1 and sign extended
        if (d_delay.front().rqst && d_delay.front().wena)
        {
            uint64_t addr = d_delay.front().addr, data = d_delay.front().wdata;
            uint8_t width = 1 << (d_delay.front().bits & 3);
            for (int j = 0; j < width; j++)
                memory[addr + j] = DTOB(data, j);
            stores.push({addr, data, width});
        }
        i_delay.pop(), d_delay.pop();
        dut->eval(), trace ? trace->dump(st++), 0 : 0; // evaluate again
        // simulator checker
        if (!cmd.debug)
            continue;
        for (int j = 0; j < sizeof(dut->cmtpc) / sizeof(dut->cmtpc[0]); j++)
            if (dut->cmtpc[j])
            {
                sim->step();
                store_t curstore;
                if (sim->get_mwwidth() && !stores.empty())
                    curstore = stores.front(), stores.pop();
                else
                    curstore = {0, 0, 0};
                int check = sim->check(dut->cmtpc[j], dut->cmtaddr[j], dut->cmtdata[j],
                                       curstore.addr, curstore.data, curstore.width);
                if (check && !cmd.step)
                    continue;
                printf(check ? "Cycle %d:\n" : "Difference found at cycle %d:\n", i);
                printf("DUT:\n    pc: 0x%016lx\n", dut->cmtpc[j]);
                printf("    x%d: 0x%016lx\n", dut->cmtaddr[j], dut->cmtdata[j]);
                if (curstore.width < 8)
                    curstore.data &= ~((uint64_t)-1 << (8 * curstore.width));
                if (sim->get_mwwidth())
                    printf("    mem%d@0x%lx: 0x%0*lx\n",
                           curstore.width, curstore.addr,
                           curstore.width * 2, curstore.data);
                printf("SIM:\n    pc: 0x%016lx\n", sim->get_pc());
                printf("    x%d: 0x%016lx\n", dut->cmtaddr[j], sim->get_arreg()[dut->cmtaddr[j]]);
                if (sim->get_mwwidth())
                    printf("    mem%d@0x%lx: 0x%0*lx\n",
                           sim->get_mwwidth(), sim->get_mwaddr(),
                           sim->get_mwwidth() * 2, sim->get_mwdata());
                printf("Press Enter to continue...\n");
                getchar();
            }
    }
    cmd.debug ? printf("Maximum cycle %d reached.\n", cmd.simtime) : 0;

    // Clean
    delete (trace ? trace->close(), trace : NULL);
    delete (cmd.debug ? sim : NULL);
    delete dut;
    return 0;
}
