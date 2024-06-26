#include <cstdio>
#include <map>
#include <queue>
#include <algorithm>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include <elf.h>
#include <fcntl.h>
#include <sys/stat.h>
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
#define BITS(dw, s, e) ((uint64_t)(dw) << (63 - (e)) >> (63 - (e) + (s)))
#define BIT(dw, i) BITS(dw, i, i)
#define NOP ((uint64_t)0x13)

typedef struct struct_cmd
{
    const char *filename = 0, *vcd = 0, *dtb = 0;
    std::vector<const char *> args;
    uint8_t help = 0, filetype = 0, debug = 0, step = 0, pc = 0;
    int maxtime = INT32_MAX, mintime = 0;
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
    uint8_t rqst = 0, bits = 0, wena = 0, rsrv = 0;
    uint64_t addr = 0, wdata = 0, vaddr = 0;
} dcache_req_t;

typedef struct struct_commit
{
    int cycle, addr;
    uint64_t pc, data;
} commit_t;

typedef struct struct_store
{
    uint64_t addr, data;
    uint8_t width;
} store_t;

typedef struct struct_csr
{
    uint64_t addr, data;
} csrcmt_t;

extern const uint8_t dtb_htif[676]; // HTIF device tree

void dumpmem(std::map<uint64_t, uint8_t> &mem, uint64_t addr, uint64_t size)
{
    printf("Memory@%016lx:", addr);
    for (int i = 0; i < size; i++)
    {
        i % 16 ? printf(i % 2 ? "" : " ") : printf("\n%016lx: ", addr + i);
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

void disasmem(std::map<uint64_t, uint8_t> &mem, uint64_t addr, uint64_t size)
{
    printf("Memory@%016lx:\n", addr);
    simulator sim(addr, mem);
    while (sim.get_pc() < addr + size)
    {
        sim.step(1);
        char s[16];
        uint32_t code = DLE(mem, sim.get_pc());
        if ((code & 3) != 3)
            sprintf(s, "    %04x", code & 0xffff);
        else
            sprintf(s, "%08x", code);
        printf("    0x%016lx:  %s  %s\n", sim.get_pc(), s, sim.get_asmcode());
    }
}

uint64_t paddr(std::map<uint64_t, uint8_t> &mem, uint64_t satp, uint64_t vaddr, uint8_t wena = 0)
{
    uint64_t ppn, vpn[4], offset;
    int start = -1;
    ppn = BITS(satp, 0, 43);
    offset = vaddr & 0xfffllu;
    if (satp >> 60 == 0) // bare
        return vaddr;
    if (satp >> 60 == 8) // sv39
    {
        start = 2;
        offset = BITS(vaddr, 0, 11);
        vpn[0] = BITS(vaddr, 12, 20);
        vpn[1] = BITS(vaddr, 21, 29);
        vpn[2] = BITS(vaddr, 30, 38);
    }
    for (int i = start; i >= 0; i--)
    {
        uint64_t pte = DLE(mem, (ppn << 12) + (vpn[i] << 3));
        ppn = pte >> 10;
        if (BIT(pte, 0) == 0)
            return -1;
        if (BIT(pte, 1) || BIT(pte, 3)) // pte.r = 1 or pte.x = 1 -> leaf node
        {
            pte |= 1 << 6;
            if (wena)
                pte |= 1 << 7;
            for (int j = 0; j < i; j++)
                ppn |= vpn[j] << (j * 9); // super page
            break;
        }
    }
    return (ppn << 12) | offset;
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
            else if (strcmp(argv[i] + j, "dtb") == 0)
                cmd.dtb = argv[++i];
            else if (strcmp(argv[i] + j, "w") == 0)
                cmd.vcd = argv[++i];
            else if (strcmp(argv[i] + j, "t") == 0)
            {
                if (i + 1 < argc)
                    cmd.mintime = atoi(argv[i + 1]);
                if (i + 2 < argc)
                    cmd.maxtime = atoi(argv[i + 2]);
                if (cmd.mintime <= 0)
                    cmd.mintime = 0;
                if (cmd.maxtime <= 0)
                    cmd.maxtime = INT32_MAX;
                i += 2;
            }
            else if (strcmp(argv[i] + j, "d") == 0)
                cmd.debug = 1;
            else if (strcmp(argv[i] + j, "s") == 0)
                cmd.step = 1;
            else if (strcmp(argv[i] + j, "h") == 0)
                cmd.help = 1;
            else if (strcmp(argv[i] + j, "pc") == 0)
                cmd.pc = 1;
        }
        else if (cmd.filename == NULL)
            cmd.filename = argv[i], cmd.args.push_back(argv[i]);
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
        printf("    -dtb `binary`: specify device tree binary.\n");
        printf("    -t `t1` `t2`: simulation time between `t1` and `t2`\n");
        printf("    -d: debug mode\n");
        printf("    -d -s: debug mode with step\n");
        printf("    -pc: output PC trace\n");
        return 0;
    }
    if (cmd.debug)
    {
        printf("[Info] Running simulation in %s mode with:\n[Info]     ",
               cmd.filetype == 0 ? "dump" : "elf");
        for (int i = 0; i < cmd.args.size(); i++)
            printf(" %s", cmd.args[i]);
        printf("\n");
        if (cmd.vcd)
            printf("[Info] Recording waveform in file: %s\n", cmd.vcd);
    }

    // Load and set reset code in memory
    std::map<uint64_t, uint8_t> memory, reserved;
    std::vector<uint32_t> ini_code;
    htif_t htif = {0, 0, 0};
    uint64_t dtbaddr = 0x10000000;   // [0-0x7ffff]000, or modify initial code
    uint64_t pkargaddr = 0x80020000; // proxy kernel arguments address
    FILE *fp = fopen(cmd.filename, "r");
    if (!fp)
        return printf("[Error] Unable to open file %s.\n", cmd.filename), 1;
    if (cmd.filetype == 0) // direct dumped hex code
    {
        int code;
        while (fscanf(fp, "%x", &code) > 0)
            ini_code.push_back(code);
        htif = {0x100000, 0x100008, 0x100010};
    }
    else if (cmd.filetype == 1) // code in ELF file
    {
        // Read and check ELF header, and print info
        Elf64_Ehdr elf_h; // ELF header
        if (fread(&elf_h, sizeof(elf_h), 1, fp) < 0)
            exit((perror("fread"), 1));
        if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)) ||
            elf_h.e_ident[EI_CLASS] != ELFCLASS64)
            return printf("[Error] Not 64-bit ELF format.\n"), 1;
        if (elf_h.e_type != ET_EXEC && elf_h.e_type != ET_DYN)
            return printf("[Error] Not an executable file.\n"), 1;
        if (elf_h.e_machine != EM_RISCV)
            return printf("[Error] Not RISC-V architecture.\n"), 1;
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
                ;
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
        if (htif.fromhost == 0 || htif.tohost == 0)
        {
            htif = {0x100000, 0x100008, 0x100010}; // default htif addresses
            printf("[Info] HTIF address not specified, set to default.\n");
        }
        delete[] shdr;
        fclose(fp);
        if (cmd.dtb)
        {
            fp = fopen(cmd.dtb, "r");
            if (!fp)
                return printf("[Error] Unable to open file %s.\n", cmd.dtb), 1;
            fseek(fp, 0, SEEK_END);
            int sz = ftell(fp);
            rewind(fp);
            for (int i = 0; i < sz; i++)
                if (fread(&memory[dtbaddr + i], 1, 1, fp) < 0)
                    exit((perror("fread"), 1));
        }
        else
        {
            printf("[Info] Using default device tree (HTIF).\n");
            for (int i = 0; i < sizeof(dtb_htif) / sizeof(dtb_htif[0]); i++)
                memory[dtbaddr + i] = dtb_htif[i];
        }
        // start section: jump from reset address to ELF entry
        // a1 -> dtb address
        ini_code.push_back(0x5b7 | dtbaddr & 0xfffff000); // lui a1, `dtbaddr >> 12`
        // ra -> entry point
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
    memory[0] = 0x6f;

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
    std::queue<commit_t> commits;
    std::queue<store_t> stores;
    std::queue<csrcmt_t> csrs;
    std::map<uint64_t, int> stalls;
    int i = 0, exitcall = 0, exitcode = 0;
    while (i < cmd.maxtime)
    {
        int cmt_check = commits.size() > 1 || exitcall;
        if (commits.empty() && exitcall)
            break;
        if (commits.size() <= 1 && !exitcall)
        {
            // negedge clock
            dut->clk = 0, dut->eval(), trace && i >= cmd.mintime ? trace->dump(st++), 0 : 0;
            // posedge clock
            if (dut->icache_rqst) // record stats before posedge
            {
                // if (i == 0004) // delay for some cycles in some conditions
                //     for (int i = 0; i < 0004; i++)
                //         i_delay.push({0, 0});
                i_delay.push({1, paddr(memory, dut->csr_satp, dut->icache_addr)});
            }
            if (dut->dcache_rqst)
                d_delay.push({dut->dcache_rqst, dut->dcache_bits, dut->dcache_wena, dut->dcache_rsrv,
                              paddr(memory, dut->csr_satp, dut->dcache_addr, dut->dcache_wena),
                              dut->dcache_wdat, dut->dcache_addr});
            while (dut->icache_flsh && !i_delay.empty())
                i_delay.pop();
            while (dut->dcache_flsh && !d_delay.empty())
                d_delay.pop();
            dut->clk = 1, dut->eval(); // clock changes first
            i_delay.empty() ? i_delay.push({0}), 0 : 0;
            d_delay.empty() ? d_delay.push({0}), 0 : 0;
            dut->icache_done = i_delay.front().rqst; // other signals change after clk
            if (i_delay.front().rqst && i_delay.front().addr != -1)
                for (int j = 0; j < 4; j++)
                    dut->icache_data[j] = DLE(memory, i_delay.front().addr + 4 * j);
            dut->icache_pgft = i_delay.front().rqst && i_delay.front().addr == -1;
            dut->dcache_done = d_delay.front().rqst;
            if (d_delay.front().rqst)
                if (d_delay.front().addr == -1)
                    dut->dcache_rdat = d_delay.front().vaddr;
                else
                    dut->dcache_rdat = DLE(memory, d_delay.front().addr);
            if (d_delay.front().rqst && d_delay.front().addr == -1)
                dut->dcache_pgft = d_delay.front().wena ? 2 : 1;
            else
                dut->dcache_pgft = 0;
            if (d_delay.front().rqst && !d_delay.front().wena && d_delay.front().rsrv)
                reserved[d_delay.front().addr] = 1;
            // bits width (funct3) decode: 00b -> 8  01b -> 16  10b -> 32  11b -> 64
            uint64_t bitwidth = 8 * (1 << (d_delay.front().bits & 3));
            uint64_t mask = bitwidth < 64 ? (1llu << bitwidth) - 1 : ~0llu;
            if (!dut->dcache_pgft)
            {
                dut->dcache_rdat &= mask;
                if (((1 << bitwidth - 1) & dut->dcache_rdat) && !(d_delay.front().bits >> 2))
                    dut->dcache_rdat |= ~mask; // msb = 1 and sign extended
                if (d_delay.front().rqst && d_delay.front().wena)
                {
                    if (d_delay.front().rsrv != 1 || reserved[d_delay.front().addr])
                    {
                        uint64_t addr = d_delay.front().addr, data = d_delay.front().wdata;
                        uint8_t width = 1 << (d_delay.front().bits & 3);
                        for (int j = 0; j < width; j++)
                            memory[addr + j] = DTOB(data, j);
                        stores.push({addr, data, width});
                        if (d_delay.front().rsrv == 1)
                            reserved[addr] = dut->dcache_rdat = 0;
                    }
                    else
                        dut->dcache_rdat = 1;
                }
            }
            i_delay.pop(), d_delay.pop();
            dut->eval(), (trace && i >= cmd.mintime) ? trace->dump(st++), 0 : 0; // evaluate again
            for (int j = 0; j < sizeof(dut->cmtpc) / sizeof(dut->cmtpc[0]); j++)
                if (dut->cmtpc[j])
                    commits.push({i, dut->cmtaddr[j], dut->cmtpc[j], dut->cmtdata[j]});
            if (dut->cmtcsrena)
                csrs.push({dut->cmtcsraddr, dut->cmtcsrval});
            if (dut->stallpc)
            {
                if (stalls.find(dut->stallpc) == stalls.end())
                    stalls[dut->stallpc] = 0;
                stalls[dut->stallpc]++;
            }
            i++;
        }
        // simulator checker
        if (cmd.debug && cmt_check)
        {
            sim->step();
            commit_t curcommit = commits.front();
            store_t curstore;
            csrcmt_t curcsr;
            if (sim->get_mwwidth() && !stores.empty())
                curstore = stores.front(), stores.pop();
            else
                curstore = {0, 0, 0};
            if (sim->get_csraddr() != -1)
                curcsr = csrs.front(), csrs.pop();
            else
                curcsr = {(uint64_t)-1, 0};
            int check = sim->check(curcommit.pc, curcommit.addr, curcommit.data,
                                   curstore.addr, curstore.data, curstore.width,
                                   curcsr.addr, curcsr.data);
            if ((!check || cmd.step) && curcommit.cycle >= cmd.mintime)
            {
                printf(check ? "[Info] Cycle %d:\n" : "[Info] Difference found at cycle %d:\n", curcommit.cycle);
                printf("[Info] DUT:\n[Info]     pc: 0x%016lx\n", curcommit.pc);
                printf("[Info]     %c%d: 0x%016lx\n", curcommit.addr < 32 ? 'x' : 'f',
                       curcommit.addr % 32, curcommit.data);
                if (curstore.width < 8)
                    curstore.data &= ~((uint64_t)-1 << (8 * curstore.width));
                if (sim->get_csraddr() != -1)
                    printf("[Info]     %s: 0x%016lx\n", sim->get_csrname(curcsr.addr), curcsr.data);
                if (curstore.width)
                    printf("[Info]     mem%d@0x%lx: 0x%0*lx\n",
                           curstore.width, curstore.addr,
                           curstore.width * 2, curstore.data);
                printf("[Info] SIM:\n[Info]     pc: 0x%016lx    %s\n", sim->get_pc(), sim->get_asmcode());
                printf("[Info]     %c%d: 0x%016lx\n", curcommit.addr < 32 ? 'x' : 'f',
                       curcommit.addr % 32, sim->get_arreg()[curcommit.addr]);
                if (sim->get_csraddr() != -1)
                    printf("[Info]     %s: 0x%016lx\n",
                           sim->get_csrname(sim->get_csraddr()), sim->get_csrdata());
                if (sim->get_mwwidth())
                    printf("[Info]     mem%d@0x%lx: 0x%0*lx\n",
                           sim->get_mwwidth(), sim->get_mwaddr(),
                           sim->get_mwwidth() * 2, sim->get_mwdata());
                printf("[Info] Press Enter to continue...\n[Info] ");
                getchar();
            }
        }
        if (cmt_check & cmd.pc & commits.front().cycle > cmd.mintime)
            printf("[Info] %d: 0x%016lx\n", commits.front().cycle, commits.front().pc);
        if (cmt_check)
            commits.pop();
        // HTIF requests handler
        uint64_t tohost_dev = DLE(memory, htif.tohost) >> 56;
        uint64_t tohost_cmd = DLE(memory, htif.tohost) << 8 >> 56;
        uint64_t tohost_dat = DLE(memory, htif.tohost) << 16 >> 16;
        if (tohost_dev == 0 && tohost_cmd == 0)
        {
            if (tohost_dat & 1) // exit
                exitcall = 1, exitcode = DLE(memory, htif.tohost) >> 1;
            else if (tohost_dat != 0) // proxied ststem call
            {
                uint64_t magic_mem = tohost_dat, which = DLE(memory, magic_mem);
                uint64_t retval = 0;
                if (which == 0x38) // sysopenat
                {
                    uint64_t arg0, arg1, arg2, arg3, arg4;
                    arg0 = DLE(memory, magic_mem + 8);  // directory file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // filename
                    arg2 = DLE(memory, magic_mem + 24); // filename size
                    arg3 = DLE(memory, magic_mem + 32); // flags
                    arg4 = DLE(memory, magic_mem + 40); // mode
                    char *filename = new (std::nothrow) char[arg2];
                    if (!filename)
                        return printf("[Error] Memory allocation failed.\n"), 1;
                    for (int i = 0; i < arg2; i++)
                        filename[i] = memory[arg1 + i];
                    retval = openat(arg0, filename, arg3, arg4);
                    delete[] filename;
                }
                else if (which == 0x39) // sysclose
                {
                    uint64_t arg0;
                    arg0 = DLE(memory, magic_mem + 8); // file descriptor
                    retval = close(arg0);
                }
                else if (which == 0x3e) // syslseek
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = DLE(memory, magic_mem + 8);  // file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // pointer
                    arg2 = DLE(memory, magic_mem + 24); // directive
                    retval = lseek(arg0, arg1, arg2);
                }
                else if (which == 0x3f) // sysread
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = DLE(memory, magic_mem + 8);  // file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // memory address
                    arg2 = DLE(memory, magic_mem + 24); // max read size
                    uint8_t *buf = new (std::nothrow) uint8_t[arg1];
                    if (!buf)
                        return printf("[Error] Memory allocation failed.\n"), 1;
                    retval = read(arg0, buf, arg2);
                    for (int i = 0; i < retval; i++)
                        memory[arg1 + i] = buf[i];
                }
                else if (which == 0x40) // syswrite
                {
                    uint64_t arg0, arg1, arg2;
                    arg0 = DLE(memory, magic_mem + 8);  // file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // memory address
                    arg2 = DLE(memory, magic_mem + 24); // write size
                    uint8_t *buf = new (std::nothrow) uint8_t[arg2];
                    if (!buf)
                        return printf("[Error] Memory allocation failed.\n"), 1;
                    for (int i = 0; i < arg2; i++)
                        buf[i] = memory[arg1 + i];
                    retval = write(arg0, buf, arg2);
                    delete[] buf;
                }
                else if (which == 0x43) // syspread
                {
                    uint64_t arg0, arg1, arg2, arg3;
                    arg0 = DLE(memory, magic_mem + 8);  // file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // memory address
                    arg2 = DLE(memory, magic_mem + 24); // read size
                    arg3 = DLE(memory, magic_mem + 32); // read offset
                    uint8_t *buf = new (std::nothrow) uint8_t[arg2];
                    if (!buf)
                        return printf("[Error] Memory allocation failed.\n"), 1;
                    retval = pread(arg0, buf, arg2, arg3);
                    for (int i = 0; i < retval; i++)
                        memory[arg1 + i] = buf[i];
                    delete[] buf;
                }
                else if (which == 0x50) // sysfstat
                {
                    uint64_t arg0, arg1;
                    struct stat s;
                    arg0 = DLE(memory, magic_mem + 8);  // file descriptor
                    arg1 = DLE(memory, magic_mem + 16); // memory address
                    retval = fstat(arg0, &s);
                    for (int i = 0; i < sizeof(s); i++)
                        memory[arg1 + i] = *((uint8_t *)&s + i);
                }
                else if (which == 0x5d) // exit
                    exitcall = 1, exitcode = (DLE(memory, magic_mem + 8) << 1) | 1;
                else if (which == 0x7db) // pk-sysgetmainvars
                {
                    // buffer format: argc(64) argv[0](64) argv[1](64) ...
                    uint64_t arg0, arg1;
                    arg0 = DLE(memory, magic_mem + 8);  // argument buffer address
                    arg1 = DLE(memory, magic_mem + 16); // argument buffer size
                    for (int i = 0; i < 8; i++)
                        memory[arg0 + i] = DTOB(cmd.args.size(), i);
                    uint64_t addr = pkargaddr;
                    for (int i = 0; i < cmd.args.size(); i++)
                    {
                        for (int j = 0; j < 8; j++)
                            memory[arg0 + (i + 1) * 8 + j] = DTOB(addr, j);
                        for (int j = 0; j < strlen(cmd.args[i]); j++)
                            memory[addr++] = cmd.args[i][j];
                        memory[addr++] = '\0';
                    }
                    if (addr - pkargaddr >= arg1)
                        retval = -1;
                }
                else
                    printf("[Info] Unhandled proxied system call 0x%lx@0x%lx\n", which, dut->epc);
                for (int i = 0; i < 8; i++)
                    memory[magic_mem + i] = DTOB(retval, i);
            }
        }
        else if (tohost_dev == 1 && tohost_cmd == 1) // console write
            putchar(tohost_dat);
        else
            printf("[Info] Unrecognized HTIF command:\n  dev: 0x%lx  cmd: 0x%lx  data: 0x%lx\n",
                   tohost_dev, tohost_cmd, tohost_dat);
        for (int i = 0; i < 8; i++)
            memory[htif.tohost + i] = memory[htif.fromhost + i] = 0;
        memory[htif.fromhost] = 1;
        cmd.debug ? sim->get_mem()[htif.fromhost] = 1 : 0;
    }
    if (cmd.debug && exitcall)
        printf("[Info] Exit with code %d.\n", exitcode);
    else if (cmd.debug)
    {
        // final status check
        for (int i = sim->csr[0xb00].val; i < cmd.maxtime; i++)
            sim->step();
        for (int i = 0; i < 64; i++)
            if (sim->get_arreg()[i] != dut->arregs[i])
            {
                printf("[Info] Difference found at maximum cycle:\n");
                printf("[Info]     DUT: %c%d: 0x%016lx\n", i < 32 ? 'x' : 'f', i % 32, dut->arregs[i]);
                printf("[Info]     SIM: %c%d: 0x%016lx\n", i < 32 ? 'x' : 'f', i % 32, sim->get_arreg()[i]);
                printf("[Info] Press Enter to continue...\n");
                getchar();
            }
        printf("[Info] Maximum cycle %d reached.\n", cmd.maxtime);
    }
    if (cmd.debug)
    {
        printf("[Info] Statistics:\n");
        printf("[Info]     CPI: %lu / %lu = %.3lf    MPKI: %lu / %.3lf = %.3lf\n",
               dut->cycle, dut->instret, (double)dut->cycle / dut->instret,
               dut->misp, dut->instret / 1000., (double)dut->misp / dut->instret * 1000);
    }

    // Clean
    delete (trace ? trace->close(), trace : NULL);
    delete (cmd.debug ? sim : NULL);
    delete dut;
    return exitcode;
}

const uint8_t dtb_htif[676] = {
    0xd0, 0x0d, 0xfe, 0xed, 0x00, 0x00, 0x02, 0xa4, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x02, 0x2c,
    0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x63, 0x68,
    0x69, 0x6e, 0x65, 0x00, 0x00, 0x00, 0x00, 0x01, 0x63, 0x70, 0x75, 0x00, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x68, 0x61, 0x72, 0x74, 0x40, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x26, 0x63, 0x70, 0x75, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x69, 0x6e, 0x74, 0x65,
    0x72, 0x72, 0x75, 0x70, 0x74, 0x2d, 0x63, 0x6f, 0x6e, 0x74, 0x6f, 0x6c, 0x6c, 0x65, 0x72, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x01, 0x23,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x4f, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
    0x6d, 0x65, 0x6d, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0b,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1a,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x6d, 0x65, 0x6d, 0x40, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x26, 0x6d, 0x65, 0x6d, 0x6f,
    0x72, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x32,
    0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x01, 0x63, 0x6c, 0x69, 0x65, 0x6e, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x63, 0x6c, 0x69, 0x65, 0x6e, 0x74, 0x40, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x72, 0x69, 0x73, 0x63, 0x76, 0x2c, 0x63, 0x6c,
    0x69, 0x6e, 0x74, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x32, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x01, 0x23, 0x00, 0x00, 0x04, 0x56, 0x00, 0x00, 0x01, 0x23,
    0x00, 0x00, 0x07, 0x89, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
    0x68, 0x74, 0x69, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0a,
    0x00, 0x00, 0x00, 0x00, 0x75, 0x63, 0x62, 0x2c, 0x68, 0x74, 0x69, 0x66, 0x30, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x09, 0x63, 0x6f, 0x6d, 0x70,
    0x61, 0x74, 0x69, 0x62, 0x6c, 0x65, 0x00, 0x23, 0x61, 0x64, 0x64, 0x72, 0x65, 0x73, 0x73, 0x2d,
    0x63, 0x65, 0x6c, 0x6c, 0x73, 0x00, 0x23, 0x73, 0x69, 0x7a, 0x65, 0x2d, 0x63, 0x65, 0x6c, 0x6c,
    0x73, 0x00, 0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 0x5f, 0x74, 0x79, 0x70, 0x65, 0x00, 0x72, 0x65,
    0x67, 0x00, 0x70, 0x68, 0x61, 0x6e, 0x64, 0x6c, 0x65, 0x00, 0x23, 0x69, 0x6e, 0x74, 0x65, 0x72,
    0x72, 0x75, 0x70, 0x74, 0x2d, 0x63, 0x65, 0x6c, 0x6c, 0x73, 0x00, 0x69, 0x6e, 0x74, 0x65, 0x72,
    0x72, 0x75, 0x70, 0x74, 0x2d, 0x63, 0x6f, 0x6e, 0x74, 0x72, 0x6f, 0x6c, 0x6c, 0x65, 0x72, 0x00,
    0x69, 0x6e, 0x74, 0x65, 0x72, 0x72, 0x75, 0x70, 0x74, 0x73, 0x2d, 0x65, 0x78, 0x74, 0x65, 0x6e,
    0x64, 0x65, 0x64, 0x00};
