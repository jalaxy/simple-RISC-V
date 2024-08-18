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
    uint8_t sim = 0, verbose = 0, help = 0, filetype = 0;
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

typedef struct
{
    uint64_t cycle, time;
    uint8_t level;
    uint64_t pc;
    uint32_t ir;
    uint8_t gpr, csr, mem;
    /* CSRs that may change when trap or return from trap */
    uint64_t mexc, sexc, intr, ret, mstatus, misa, mtvec, mcause, mepc, mtval;
    uint64_t stvec, scause, sepc, stval, mip, mcycle, minstret;
} cmt_t;

typedef struct
{
    uint64_t w, a, v;
} del_t;

int interrupt = 0;
void intrhandler(int) { fprintf(stderr, "[Info] Interrupted\n"), interrupt = 1; }

bool check(delta_t del, delta_t ref, memory &localmem)
{
    if (del.level != ref.level || del.pc != ref.pc)
        return false;
    del.memv &= (del.memw == 8 ? -1ul : (1ul << (del.memw & 0xf) * 8) - 1);
    ref.memv &= (ref.memw == 8 ? -1ul : (1ul << (ref.memw & 0xf) * 8) - 1);
    if (del.memw >> 4 == 0x8)
        del.memv = ref.memv = 0;
    if (del.gprw != ref.gprw || del.memw != ref.memw)
        return false;
    if (del.gprw && (del.gpra != ref.gpra || del.gprv != ref.gprv))
        if (!(ref.ldlocal && del.gprv == localmem.ui64(ref.ldaddr)))
            // in load value axiom of RVWMO, the load value can be not only the global
            // memory value, but also local store value
            // this could happen when setting HTIF fromhost by host machine
            return false;
    if (del.memw && (del.mema != ref.mema || del.memv != ref.memv))
        return false;
    ref.csr.erase("mcycle");
    del.csr.erase("mcycle");
    ref.csr.erase("minstret");
    del.csr.erase("minstret");
    ref.csr.erase("mip");
    del.csr.erase("mip");
    for (auto i : ref.csr)
        if (del.csr.find(i.first) != del.csr.end() & del.csr[i.first] == i.second)
            del.csr.erase(i.first);
        else
            return false;
    if (!del.csr.empty())
        return false;
    return true;
}

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
            else if (strcmp(argv[i] + j, "v") == 0)
                cmd.verbose = 1;
            else if (strcmp(argv[i] + j, "s") == 0)
                cmd.sim = 1;
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
        printf("    -bin: (force) load binary file\n");
        printf("    -hex: (force) load hex text file\n");
        printf("    -elf: (default) load RISC-V ELF executable\n");
        printf("    -dtb `binary`: specify device tree binary\n");
        printf("    -initrd `binary`: specify initial rootfs\n");
        printf("    -w `waveform`: output waveform to `waveform`\n");
        printf("    -t `t1` `t2`: simulation time between `t1` and `t2`\n");
        printf("    -s: run and check with simulator\n");
        printf("    -v: verbose mode\n");
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
    mem.add(0xc0000, 0x2000000); // CLINT area
    const uint64_t mtime = 0x200bff8, mtimecmp = 0x2004000;
    mem.ui64(mtime) = 0, mem.ui64(mtimecmp) = -1ull;

    /* Simulate */
    Vstats *dut = new (std::nothrow) Vstats;
    state_t *sim = cmd.sim ? new (std::nothrow) state_t : NULL;
    VerilatedVcdC *trace = cmd.vcd ? new (std::nothrow) VerilatedVcdC : NULL;
    uint64_t wt = 0; // waveform record time
    if (sim)
        sim->pc = entry, sim->mem = mem, sim->csr["mtime"] = mtime;
    if (trace)
        Verilated::traceEverOn(true), dut->trace(trace, 5), trace->open(cmd.vcd);
    // reset
    dut->rst = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    dut->rst = 1, dut->clk = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    for (int i = 0; i < 8; i++)
        dut->clk = !dut->clk, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    dut->rst = 0, dut->eval(), trace ? trace->dump(wt++), 0 : 0;
    // clock and memory loop
    std::queue<icache_req_t> ireq;
    std::queue<dcache_req_t> dreq;
    std::queue<cmt_t> cmts;
    std::queue<del_t> gprs, csrs, mems;
    memory localmem = mem; // record local store values
    uint64_t cycle = 0, htifexit = 0;
    const char *exitcause = NULL;
    uint8_t exitcode;
    signal(SIGINT, intrhandler);
    while (!interrupt && !exitcause)
    {
        static int emptytimes = 0, stabletimes = 0;
        if (cmts.size() < 2) // drive clock and handle memory requests
        {
            // negedge clock
            dut->clk = 0, dut->eval(), trace && cycle >= cmd.mintime ? trace->dump(wt++), 0 : 0;
            // posedge clock
            if (dut->icache_rqst) // record stats before posedge
                ireq.push({1, paddr(mem, dut->csr_satp, dut->icache_addr, 1 << 3)});
            if (dut->dcache_rqst)
                dreq.push({dut->dcache_rqst, dut->dcache_bits, dut->dcache_wena, dut->dcache_rsrv,
                           paddr(mem, dut->csr_satp, dut->dcache_addr, dut->dcache_wena << 2),
                           dut->dcache_wdat, dut->dcache_addr});
            while (dut->icache_flsh && !ireq.empty())
                ireq.pop();
            while (dut->dcache_flsh && !dreq.empty())
                dreq.pop();
            ireq.empty() ? ireq.push({0}), 0 : 0;
            dreq.empty() ? dreq.push({0}), 0 : 0;
            dut->clk = 1, dut->eval(); // clock changes first, other signals change after clk
            dut->mtime = mem.ui64(mtime);
            if (cycle % 16 == 0) // set interrupt
                mem.ui64(mtime)++;
            if (mtime > mtimecmp)
                dut->eiptip |= 1 << 3;
            dut->icache_done = ireq.front().rqst; // handle an icache request
            if (ireq.front().rqst && ireq.front().addr != -1)
                for (int j = 0; j < 4; j++)
                    dut->icache_data[j] = mem.ui32(ireq.front().addr + 4 * j);
            dut->icache_pgft = ireq.front().rqst && ireq.front().addr == -1;
            dut->dcache_done = dreq.front().rqst; // handle an dcache request
            if (dreq.front().rqst)
                if (dreq.front().addr == -1)
                    dut->dcache_rdat = dreq.front().vaddr; // fault vaddr in rdat as tval
                else
                {
                    if (mem.issegfault(mem[dreq.front().addr]))
                        mem.add(1, dreq.front().addr);
                    dut->dcache_rdat = mem.ui64(dreq.front().addr);
                }
            dut->dcache_pgft = 0;
            if (dreq.front().rqst && dreq.front().addr == -1) // [1:0] -> [WPF,RPF]
                dut->dcache_pgft = dreq.front().wena || dreq.front().rsrv == 2 ? 2 : 1;
            // bits width (funct3) decode: 00b -> 8  01b -> 16  10b -> 32  11b -> 64
            uint64_t width = 1 << (dreq.front().bits & 3); // process data of dcache response
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
                    { // check SC
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
            // evaluate again
            dut->eval(), (trace && cycle >= cmd.mintime) ? trace->dump(wt++), 0 : 0;
            ireq.pop(), dreq.pop();
            // extract commits
            for (int i = 0; i < 4; i++)
                if (dut->cmt[i])
                    cmts.push({.cycle = cycle,
                               .time = dut->mtime,
                               .level = dut->cmt_level[i],
                               .pc = dut->cmt_pc[i],
                               .ir = dut->cmt_ir[i],
                               .gpr = dut->cmt_gpr[i],
                               .csr = dut->cmt_csr[i],
                               .mem = dut->cmt_mem[i],
                               .mexc = dut->cmt_mexc && (i == 3 || !dut->cmt[i + 1]), // last commit
                               .sexc = dut->cmt_sexc && (i == 3 || !dut->cmt[i + 1]),
                               .intr = dut->cmt_int,
                               .ret = dut->cmt_ret && (i == 3 || !dut->cmt[i + 1]),
                               .mstatus = dut->cmt_mstatus,
                               .misa = dut->cmt_misa,
                               .mtvec = dut->cmt_mtvec,
                               .mcause = dut->cmt_mcause,
                               .mepc = dut->cmt_mepc,
                               .mtval = dut->cmt_mtval,
                               .stvec = dut->cmt_stvec,
                               .scause = dut->cmt_scause,
                               .sepc = dut->cmt_sepc,
                               .stval = dut->cmt_stval,
                               .mip = dut->cmt_mip,
                               .mcycle = dut->cmt_mcycle,
                               .minstret = dut->cmt_minstret});
            for (int i = 0; i < 4; i++)
                if (dut->del_gprw[i])
                    gprs.push({.w = 1, .a = dut->del_gpra[i], .v = dut->del_gprv[i]});
            if (dut->del_csrw)
                csrs.push({.w = 1, .a = dut->del_csra, .v = dut->del_csrv});
            uint64_t va;
            if (dut->del_memw && (va = paddr(mem, dut->csr_satp, dut->del_mema, 1 << 2)) != -1ull)
                mems.push({.w = dut->del_memw, .a = va, .v = dut->del_memv});
            cycle++;
            emptytimes++;
            if (cmd.filetype == 1 && emptytimes > 1024)
                exitcause = "no commits within long time (hex mode)", exitcode = 255;
            if (cycle > cmd.maxtime)
                exitcause = "reaching maximum cycle", exitcode = 0;
            // HTIF requests handler
            if ((htifexit = htif(mem, htifaddr, cmd.args, sim ? &sim->mem : 0)) & 1)
                exitcause = "HTIF exit call", exitcode = htifexit >> 1;
        }
        else // check deltas
        {
            state_t stt; // for print
            delta_t del;
            uint64_t cyc = cmts.front().cycle;
            uint8_t mexc = cmts.front().mexc, sexc = cmts.front().sexc;
            uint8_t intr = cmts.front().intr, ret = cmts.front().ret;
            stt.level = cmts.front().level;
            stt.pc = cmts.front().pc;
            stt.ir = cmts.front().ir;
            sim ? sim->csr["mip"] = intr ? cmts.front().mip : 0, 0 : 0;
            sim ? sim->csr["mcycle"] = cmts.front().mcycle, 0 : 0;
            sim ? sim->csr["minstret"] = cmts.front().minstret, 0 : 0;
            sim ? sim->mem.ui64(mtime) = cmts.front().time, 0 : 0;
            if (cmts.front().gpr)
                del.gprw = 1, del.gpra = gprs.front().a, del.gprv = gprs.front().v, gprs.pop();
            else
                del.gprw = 0;
            if (cmts.front().mem)
            {
                del.memw = mems.front().w;
                del.mema = mems.front().a;
                del.memv = mems.front().v;
                mems.pop();
                if (del.memw >> 4 == 0xc && del.gprw && del.gprv == 1) // failed SC
                    del.memw = 0;
            }
            else
                del.memw = 0;
            if (cmts.front().csr)
            {
                uint64_t a = csrs.front().a;
                if (a == 0x100 || a == 0x144 || a == 0x104) // sstatus/sip/sie
                    a += 0x200;
                if (!mexc && !sexc && csrname.find(a) != csrname.end())
                    del.csr[csrname[a]] = csrs.front().v;
                csrs.pop();
            }
            cmts.pop();
            del.level = cmts.front().level;
            del.pc = cmts.front().pc;
            if (del.csr.find("mstatus") != del.csr.end() || ret)
                del.csr["mstatus"] = cmts.front().mstatus;
            if (del.csr.find("misa") != del.csr.end())
                del.csr["misa"] = cmts.front().misa;
            if (del.csr.find("mtvec") != del.csr.end())
                del.csr["mtvec"] = cmts.front().mtvec;
            if (del.csr.find("stvec") != del.csr.end())
                del.csr["stvec"] = cmts.front().stvec;
            if (mexc)
            {
                del.csr["mstatus"] = cmts.front().mstatus;
                del.csr["mcause"] = cmts.front().mcause;
                del.csr["mepc"] = cmts.front().mepc;
                del.csr["mtval"] = cmts.front().mtval;
            }
            if (sexc)
            {
                del.csr["mstatus"] = cmts.front().mstatus;
                del.csr["scause"] = cmts.front().scause;
                del.csr["sepc"] = cmts.front().sepc;
                del.csr["stval"] = cmts.front().stval;
            }
            if (sim)
            {
                delta_t delsim = next(*sim);
                if (!check(del, delsim, localmem))
                {
                    fprintf(stderr, "[Debug] ------ Difference detected ------\n");
                    fprintf(stderr, "[Debug] DUT/SIM:\n");
                    print(cyc, stt, del);
                    print(cycle, *sim, delsim);
                    fprintf(stderr, "[Debug] DUT CSRs:");
                    for (auto i : del.csr)
                        if (i.first != "mcycle" && i.first != "minstret")
                            fprintf(stderr, " %s: %lx", i.first.c_str(), (uint64_t)i.second);
                    fprintf(stderr, " npc: %lx\n", del.pc);
                    fprintf(stderr, "[Debug] SIM CSRs:");
                    for (auto i : delsim.csr)
                        if (i.first != "mcycle" && i.first != "minstret")
                            fprintf(stderr, " %s: %lx", i.first.c_str(), (uint64_t)i.second);
                    fprintf(stderr, " npc: %lx\n", delsim.pc);
                    fprintf(stderr, "[Debug] ---------------------------------\n");
                    exitcause = "checking failure", exitcode = 255;
                }
                localmem.ui64(del.mema) = del.memv;
                apply(*sim, del);
            }
            if (cyc >= cmd.mintime)
                cmd.verbose ? print(cyc, stt, del), 0 : 0;
            emptytimes = 0;
            if (cmd.filetype == 1 && cmd.maxtime == INT32_MAX)
                if (del.level == stt.level && del.pc == stt.pc && !del.gprw && !del.memw)
                    stabletimes++;
            if (stabletimes > 16)
                exitcause = "reaching stable state (hex mode)", exitcode = 0;
        }
    }
    if (interrupt)
        exitcause = "SIGINT", exitcode = 130;
    if (cmd.filetype == 1 && cmd.verbose)
        disasmem(&mem[0x400000], hexsz);

    /* Clean and exit */
    delete (trace ? trace->close(), trace : NULL);
    delete dut;
    delete sim;
    fprintf(stderr, "[Info] Exited at cycle %ld with code %hhu due to %s\n", cycle, exitcode, exitcause);
    return exitcode;
}
