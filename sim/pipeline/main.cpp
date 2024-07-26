#include <cstdio>
#include <queue>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include <elf.h>
#include <signal.h>
#include "../simulator/state.h"
#include "Vstats.h"

typedef struct
{
    const char *file = 0, *dtb = 0, *initrd = 0, *vcd = 0;
    std::vector<const char *> args;
    uint8_t debug = 0, help = 0, filetype = 0;
    int mintime = 0, maxtime = INT32_MAX;
} cmd_t;

typedef struct
{
    uint8_t rqst = 0;
    uint64_t addr = 0;
} icache_req_t;

typedef struct
{
    uint8_t rqst = 0, bits = 0, wena = 0, rsrv = 0;
    uint64_t addr = 0, wdata = 0, vaddr = 0;
} dcache_req_t;

int interrupt = 0;
void intrhandler(int) { fprintf(stderr, "[Info] Interrupted\n"), interrupt = 1; }

int main(int argc, char *argv[])
{
    /* Read command line */
    cmd_t cmd;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] == '-' && cmd.file == NULL)
        {
            int j = 1;
            while (argv[i][j] == '-')
                j++;
            if (strcmp(argv[i] + j, "bin") == 0)
                cmd.filetype = 2;
            else if (strcmp(argv[i] + j, "hex") == 0)
                cmd.filetype = 1;
            else if (strcmp(argv[i] + j, "elf") == 0)
                cmd.filetype = 0;
            else if (strcmp(argv[i] + j, "dtb") == 0)
                cmd.dtb = argv[++i];
            else if (strcmp(argv[i] + j, "initrd") == 0)
                cmd.initrd = argv[++i];
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
            else if (strcmp(argv[i] + j, "h") == 0)
                cmd.help = 1;
        }
        else if (cmd.file == NULL)
            cmd.args.push_back(cmd.file = argv[i]);
        else
            cmd.args.push_back(argv[i]);
    if (!cmd.help && cmd.file == NULL)
        printf("Not enough arguments\n"), cmd.help = 1;
    if (cmd.help)
    {
        printf("Usage: exec [options] file [arguments]\n");
        printf("Available options:\n");
        printf("    -bin: (force) input file as binary file\n");
        printf("    -hex: (force) input file as hex text file\n");
        printf("    -elf: (default) input file as RISC-V ELF executable\n");
        printf("    -w `waveform`: output waveform to `waveform`\n");
        printf("    -dtb `binary`: specify device tree binary\n");
        printf("    -initrd `binary`: specify initial rootfs\n");
        printf("    -t `t1` `t2`: simulation time between `t1` and `t2`\n");
        printf("    -d: debug mode\n");
        return 0;
    }
    fprintf(stderr, "[Info] Running simulation in %s mode with:\n[Info]     ",
            cmd.filetype == 0 ? "elf" : (cmd.filetype == 1 ? "hex" : "bin"));
    for (int i = 0; i < cmd.args.size(); i++)
        fprintf(stderr, " %s", cmd.args[i]);
    fprintf(stderr, "\n");
    if (cmd.vcd)
        fprintf(stderr, "[Info] Recording waveform in file: %s\n", cmd.vcd);

    /* Initialize memory */
    uint64_t entry = 0x400000;
    memory mem;
    std::map<uint64_t, uint8_t> rsrv;
    htifaddr_t htifaddr;
    uint64_t dtbaddr = 0x1020;
    uint64_t initrdaddr = 0xfe60fe00;
    uint64_t hexsz;
    if (cmd.filetype == 0)
    {
        /* ELF format */
        /* Read and check ELF header */
        Elf64_Ehdr elf_h; // ELF header
        FILE *fp = fopen(cmd.file, "r");
        if (!fp)
            return fprintf(stderr, "[Error] Unable to open file %s\n", cmd.file), 1;
        if (fread(&elf_h, sizeof(elf_h), 1, fp) < 0)
            return fprintf(stderr, "[Error] Fread failed\n"), 1;
        if (strncmp((char *)elf_h.e_ident, ELFMAG, strlen(ELFMAG)) ||
            elf_h.e_ident[EI_CLASS] != ELFCLASS64)
            return fprintf(stderr, "[Error] Not 64-bit ELF format\n"), 1;
        if (elf_h.e_type != ET_EXEC && elf_h.e_type != ET_DYN)
            return fprintf(stderr, "[Error] Not an executable file\n"), 1;
        if (elf_h.e_machine != EM_RISCV)
            return fprintf(stderr, "[Error] Not RISC-V architecture\n"), 1;
        /* sections from ELF file */
        Elf64_Shdr *shdr = new (std::nothrow) Elf64_Shdr[elf_h.e_shnum]; // section headers
        fseek(fp, elf_h.e_shoff, SEEK_SET);
        if (fread(shdr, sizeof(Elf64_Shdr) * elf_h.e_shnum, 1, fp) < 0)
            return fprintf(stderr, "[Error] Fread failed\n"), 1;
        for (int i = 0; i < elf_h.e_shnum; i++)
            if (shdr[i].sh_flags & SHF_ALLOC)
                if (shdr[i].sh_type == SHT_NOBITS)
                {
                    if (!mem.add(shdr[i].sh_size, shdr[i].sh_addr))
                        return fprintf(stderr, "[Error] Adding memory failed\n"), 1;
                }
                else
                {
                    fseek(fp, shdr[i].sh_offset, SEEK_SET);
                    if (!mem.read(fp, shdr[i].sh_size, shdr[i].sh_addr))
                        return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
                }
            else if (shdr[i].sh_type == SHT_SYMTAB)
            {
                /* check section name */
                char name[1024];
                fseek(fp, shdr[elf_h.e_shstrndx].sh_offset + shdr[i].sh_name, SEEK_SET);
                if (fscanf(fp, "%1023s", name) <= 0)
                    return fprintf(stderr, "[Error] Fscanf failed\n"), 1;
                if (strcmp(name, ".symtab") != 0)
                    continue;
                /* read symbol table and search for fromhost and tohost */
                int sym_sz = shdr[i].sh_size / shdr[i].sh_entsize;
                for (int j = 0; j < sym_sz; j++)
                {
                    Elf64_Sym sym;
                    fseek(fp, shdr[i].sh_offset + j * shdr[i].sh_entsize, SEEK_SET);
                    if (fread(&sym, sizeof(sym), 1, fp) < 0)
                        return fprintf(stderr, "[Error] Fread failed\n"), 1;
                    fseek(fp, shdr[shdr[i].sh_link].sh_offset + sym.st_name, SEEK_SET);
                    if (fscanf(fp, "%1023s", name) <= 0)
                        return fprintf(stderr, "[Error] Fscanf failed\n"), 1;
                    if (strcmp(name, "fromhost") == 0)
                        htifaddr.fromhost = sym.st_value;
                    else if (strcmp(name, "tohost") == 0)
                        htifaddr.tohost = sym.st_value;
                    else if (strcmp(name, "htif_lock") == 0)
                        htifaddr.lock = sym.st_value;
                }
            }
        if (htifaddr.fromhost == 0 || htifaddr.tohost == 0)
        {
            htifaddr = {0x2000, 0x2008, 0x2010}; // default htif addresses
            mem.add(4096, 0x2000);
            fprintf(stderr, "[Info] HTIF address not specified, set to default\n");
        }
        delete[] shdr;
        fclose(fp);
        /* start section: jump from reset address to ELF entry */
        mem.add(0x1000, entry);
        mem.ui32(entry + 0) = 0x5b7 | dtbaddr & 0xfffff000; // lui a1, `dtbaddr >> 12`
        mem.ui32(entry + 4) = 0x58593 | dtbaddr << 20;      // addi a1, a1, `dtbaddr & 0xfff`
        mem.ui32(entry + 8) = 0x93;                         // addi ra, zero, 0
        for (int i = 0; i < 8; i++)
        {
            mem.ui32(entry + 12 + 8 * i) = 0x8093 | (uint8_t(elf_h.e_entry >> 8 * (7 - i)) << 20);
            mem.ui32(entry + 16 + 8 * i) = (i == 7 ? 0x8067 : 0x809093); // ret : slli ra, ra, 8
        }
    }
    else if (cmd.filetype == 1)
    {
        /* hex code */
        hexsz = 0;
        FILE *fp = fopen(cmd.file, "r");
        if (!fp)
            return fprintf(stderr, "Unable to open file '%s'\n", cmd.file), 1;
        uint32_t inst;
        while (fscanf(fp, "%x", &inst) == 1)
            hexsz++;
        hexsz *= 4;
        uint8_t *buffer;
        if (hexsz > 0x80000000ull || (buffer = new (std::nothrow) uint8_t[hexsz + 4]) == 0)
            return fprintf(stderr, "Require too much memory\n"), 1;
        rewind(fp);
        for (int i = 0; i < hexsz; i += 4)
            if (fscanf(fp, "%x", (uint32_t *)(buffer + i)) != 1)
                return fprintf(stderr, "Read file failed\n"), 1;
        fclose(fp);
        ((uint32_t *)buffer)[hexsz / 4] = 0x6f; // j 0(pc)
        if (!mem.copy(buffer, hexsz += 4, entry))
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        if (!mem.add(0x1000, 0x10010000)) // data segment
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        delete[] buffer;
    }
    else if (cmd.filetype == 2)
    {
        /* bin code */
        uint64_t binentry = 0x80000000;
        FILE *fp = fopen(cmd.file, "r");
        fseek(fp, 0, SEEK_END);
        size_t binsz = ftell(fp);
        rewind(fp);
        if (!mem.read(fp, binsz, binentry))
            return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
        fclose(fp);
        htifaddr = {0x800421b0, 0x800421b8}; // buildroot default
        mem.add(0x1000, entry);
        mem.ui32(entry + 0) = 0x5b7 | dtbaddr & 0xfffff000; // lui a1, `dtbaddr >> 12`
        mem.ui32(entry + 4) = 0x58593 | dtbaddr << 20;      // addi a1, a1, `dtbaddr & 0xfff`
        mem.ui32(entry + 8) = 0x93;                         // addi ra, zero, 0
        for (int i = 0; i < 8; i++)
        {
            mem.ui32(entry + 12 + 8 * i) = 0x8093 | (uint8_t(binentry >> 8 * (7 - i)) << 20);
            mem.ui32(entry + 16 + 8 * i) = (i == 7 ? 0x8067 : 0x809093); // ret : slli ra, ra, 8
        }
    }
    if (cmd.dtb)
    {
        FILE *fp = fopen(cmd.dtb, "r");
        if (!fp)
            return fprintf(stderr, "[Error] Unable to open file %s\n", cmd.dtb), 1;
        fseek(fp, 0, SEEK_END);
        size_t sz = ftell(fp);
        rewind(fp);
        if (!mem.read(fp, sz, dtbaddr))
            return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
        fclose(fp);
    }
    if (cmd.initrd)
    {
        FILE *fp = fopen(cmd.initrd, "r");
        if (!fp)
            return fprintf(stderr, "[Error] Unable to open file %s\n", cmd.initrd), 1;
        fseek(fp, 0, SEEK_END);
        size_t sz = ftell(fp);
        rewind(fp);
        if (!mem.read(fp, sz, initrdaddr))
            return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
        fclose(fp);
    }
    mem.ui64(htifaddr.fromhost) = mem.ui64(htifaddr.tohost) = 0;

    /* Simulate */
    Vstats *dut = new (std::nothrow) Vstats;
    state_t *sim = cmd.debug ? new (std::nothrow) state_t : NULL;
    VerilatedVcdC *trace = cmd.vcd ? new (std::nothrow) VerilatedVcdC : NULL;
    uint64_t wt = 0; // waveform record time
    if (trace)
    {
        Verilated::traceEverOn(true); // trace waveform
        dut->trace(trace, 5);
        trace->open(cmd.vcd);
    }
    // reset
    dut->rst = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    dut->rst = 1, dut->clk = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    for (int i = 0; i < 8; i++)
        dut->clk = !dut->clk, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    dut->rst = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    // clock and memory loop
    std::queue<icache_req_t> ireq;
    std::queue<dcache_req_t> dreq;
    std::queue<delta_t> deltas;
    uint64_t cycle = 0, htifexit = 0;
    mem.add(0xc0000, 0x2000000); // CLINT area
    mem.ui64(0x200bff8) = 0;     // mtime
    mem.ui64(0x2004000) = -1ull; // mtimecmp
    signal(SIGINT, intrhandler);
    while (!interrupt && cycle <= cmd.maxtime)
    {
        if (deltas.empty())
        {
            // negedge clock
            dut->clk = 0, dut->eval(), trace && cycle >= cmd.mintime ? trace->dump(wt++), 0 : 0;
            // posedge clock
            if (dut->icache_rqst) // record stats before posedge
                ireq.push({1, paddr(mem, dut->csr_satp, dut->icache_addr)});
            if (dut->dcache_rqst)
                dreq.push({dut->dcache_rqst, dut->dcache_bits, dut->dcache_wena, dut->dcache_rsrv,
                           paddr(mem, dut->csr_satp, dut->dcache_addr, dut->dcache_wena),
                           dut->dcache_wdat, dut->dcache_addr});
            while (dut->icache_flsh && !ireq.empty())
                ireq.pop();
            while (dut->dcache_flsh && !dreq.empty())
                dreq.pop();
            ireq.empty() ? ireq.push({0}), 0 : 0;
            dreq.empty() ? dreq.push({0}), 0 : 0;
            dut->clk = 1, dut->eval();            // clock changes first
            dut->icache_done = ireq.front().rqst; // other signals change after clk
            if (ireq.front().rqst && ireq.front().addr != -1)
                for (int j = 0; j < 4; j++)
                    dut->icache_data[j] = mem.ui32(ireq.front().addr + 4 * j);
            dut->icache_pgft = ireq.front().rqst && ireq.front().addr == -1;
            dut->dcache_done = dreq.front().rqst;
            if (dreq.front().rqst)
                if (dreq.front().addr == -1)
                    dut->dcache_rdat = dreq.front().vaddr; // fault vaddr in rdat as tval
                else
                    dut->dcache_rdat = mem.ui64(dreq.front().addr);
            dut->dcache_pgft = 0;
            if (dreq.front().rqst && dreq.front().addr == -1)
                dut->dcache_pgft = dreq.front().wena ? 2 : 1; // [1:0] -> [WPF,RPF]
            // bits width (funct3) decode: 00b -> 8  01b -> 16  10b -> 32  11b -> 64
            uint64_t width = 1 << (dreq.front().bits & 3);
            uint64_t mask = width < 8 ? (1llu << 8 * width) - 1 : ~0llu;
            if (dreq.front().rqst && !dreq.front().wena && dreq.front().rsrv == 1)
                for (int i = 0; i < width; i++)
                    rsrv[dreq.front().addr + i] = 1; // register reservation set (LR)
            if (!dut->dcache_pgft)
            {
                dut->dcache_rdat &= mask;
                if (((1 << 8 * width - 1) & dut->dcache_rdat) && !(dreq.front().bits >> 2))
                    dut->dcache_rdat |= ~mask; // msb = 1 and sign extended
                if (dreq.front().rqst && dreq.front().wena)
                { // store instructions
                    if (dreq.front().rsrv != 1 || rsrv[dreq.front().addr])
                    {
                        uint64_t addr = dreq.front().addr, data = dreq.front().wdata;
                        for (int j = 0; j < width; j++)
                            mem[addr + j] = bits(data).range(j * 8, j * 8 + 7);
                        if (dreq.front().rsrv == 1)
                            rsrv[addr] = dut->dcache_rdat = 0;
                    }
                    else
                        dut->dcache_rdat = 1;
                }
            }
            ireq.pop(), dreq.pop();
            // extract delta
            for (int i = 0; i < 4; i++)
                if (dut->cmt[i])
                    deltas.push({.pc = dut->stt_pc[i]});
            // evaluate again
            dut->eval(), (trace && cycle >= cmd.mintime) ? trace->dump(wt++), 0 : 0;
            cycle++;
        }
        else
        { // check deltas
            if (cycle >= cmd.mintime)
                cmd.debug ? fprintf(stderr, "[Debug] %ld %lx\n", cycle, deltas.front().pc), 0 : 0;
            deltas.pop();
        }
        // HTIF requests handler
        htif(mem, htifaddr, cmd.args);
    }

    /* Clean and exit */
    delete (trace ? trace->close(), trace : NULL);
    delete dut;
    delete sim;
    if (cycle > cmd.maxtime)
        fprintf(stderr, "[Info] Exceeded maximum cycle %d\n", cmd.maxtime);
    if (htifexit & 1)
        fprintf(stderr, "[Info] Exited with code %hhu\n", (int)htifexit >> 1);
    return htifexit >> 1;
}
