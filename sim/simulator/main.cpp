#include <cstdio>
#include <cstring>
#include <vector>
#include <elf.h>
#include <sys/signal.h>
#include "status.h"

typedef struct
{
    const char *file = 0, *dtb = 0, *initrd = 0;
    std::vector<const char *> args;
    uint8_t debug = 0, help = 0, filetype = 0;
    int mintime = 0, maxtime = INT32_MAX;
} cmd_t;

int interrupt = 0;
void intrhandler(int) { fprintf(stderr, "[Info] Interrupted\n"), interrupt = 1; }

void print(uint64_t cycle, status_t &status, const delta_t &delta)
{
    char s[256], lch[4] = {'U', 'S', 'H', 'M'};
    sprintf(s, "[Debug] cycle %ld: %c@%lx: %8x %s", cycle,
            lch[status.level & 3], status.pc, status.ir, disas(status.ir).c_str());
    if (strlen(s) < 63)
    {
        for (int i = strlen(s); i < 63; i++)
            s[i] = ' ';
        s[63] = 0;
    }
    fputs(s, stderr);
    if (delta.gprw)
        fprintf(stderr, " %s: %lx", gprname[delta.gpra], delta.gprv);
    if (delta.memw && delta.memw >> 4 != 0x8)
        fprintf(stderr, " d%d@%lx: %lx", delta.memw & 0xf, delta.mema, delta.memv);
    fprintf(stderr, "\n");
}

void print(status_t &s, uint64_t addr = 0, uint64_t size = 0)
{
    fprintf(stderr, "[Debug] General-purpose registers:\n");
    for (int i = 0; i < 16; i++)
    {
        fprintf(stderr, "[Debug]");
        for (int j = 0; j < 4; j++)
            fprintf(stderr, " %8s: %016lx", gprname[i * 4 + j], (uint64_t)s.gpr[i * 4 + j]);
        fprintf(stderr, "\n");
    }
    if (size)
        dumpmem(&s.mem[addr], addr, size);
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
        printf("    -hex: (force) input file as hex hump\n");
        printf("    -elf: (default) input file as RISC-V ELF executable\n");
        printf("    -dtb `binary`: specify device tree binary\n");
        printf("    -initrd `binary`: specify initial rootfs\n");
        printf("    -t `t1` `t2`: simulation time between `t1` and `t2`\n");
        return 0;
    }
    fprintf(stderr, "[Info] Running simulation in %s mode with:\n[Info]     ",
            cmd.filetype == 0 ? "elf" : (cmd.filetype == 1 ? "hex" : "bin"));
    for (int i = 0; i < cmd.args.size(); i++)
        fprintf(stderr, " %s", cmd.args[i]);
    fprintf(stderr, "\n");

    /* Initialize memory and registers */
    status_t s;
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
                    if (!s.mem.add(shdr[i].sh_size, shdr[i].sh_addr))
                        return fprintf(stderr, "[Error] Adding memory failed\n"), 1;
                }
                else
                {
                    fseek(fp, shdr[i].sh_offset, SEEK_SET);
                    if (!s.mem.read(fp, shdr[i].sh_size, shdr[i].sh_addr))
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
            s.mem.add(4096, 0x2000);
            fprintf(stderr, "[Info] HTIF address not specified, set to default\n");
        }
        delete[] shdr;
        fclose(fp);
        s.pc = elf_h.e_entry;
        s.gpr[11] = dtbaddr; // `a1` as device tree address
    }
    else if (cmd.filetype == 1)
    {
        /* hex code */
        s.pc = 0x80000000;
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
        if (!s.mem.copy(buffer, hexsz += 4, s.pc))
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        if (!s.mem.add(0x1000, 0x10010000)) // data segment
            return fprintf(stderr, "[Error] Memory allocation failed\n"), 1;
        delete[] buffer;
    }
    else if (cmd.filetype == 2)
    {
        /* bin code */
        s.pc = 0x80000000;
        FILE *fp = fopen(cmd.file, "r");
        fseek(fp, 0, SEEK_END);
        size_t binsz = ftell(fp);
        rewind(fp);
        if (!s.mem.read(fp, binsz, s.pc))
            return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
        fclose(fp);
        htifaddr = {0x800421b0, 0x800421b8}; // buildroot default
        s.gpr[11] = dtbaddr;
    }
    if (cmd.dtb)
    {
        FILE *fp = fopen(cmd.dtb, "r");
        if (!fp)
            return fprintf(stderr, "[Error] Unable to open file %s\n", cmd.dtb), 1;
        fseek(fp, 0, SEEK_END);
        size_t sz = ftell(fp);
        rewind(fp);
        if (!s.mem.read(fp, sz, dtbaddr))
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
        if (!s.mem.read(fp, sz, initrdaddr))
            return fprintf(stderr, "[Error] Adding memory from file failed\n"), 1;
        fclose(fp);
    }

    /* Simulate */
    uint64_t cycle = 0, htifexit;
    s.mem.add(0xc0000, 0x2000000);                     // CLINT area
    s.mem.ui64(s.csr["mtime"] = 0x200bff8) = 0;        // mtime
    s.mem.ui64(s.csr["mtimecmp"] = 0x2004000) = -1ull; // mtimecmp
    signal(SIGINT, intrhandler);
    while (!interrupt && cycle <= cmd.maxtime)
    {
        /* set interrupts */
        if (cycle % 10 == 0) // increase mtime
            s.mem.ui64(s.csr["mtime"])++;
        s.csr["mip"].write(7, s.mem.ui64(s.csr["mtime"]) >= s.mem.ui64(s.csr["mtimecmp"]));

        /* get next status */
        delta_t d = next(s);
        if (cycle >= cmd.mintime)
            cmd.debug ? print(cycle, s, d), 0 : 0;
        apply(s, d);

        /* cycle increment */
        if (cycle && cycle % 1000000 == 0)
            fprintf(stderr, "[Info] Keep-alive: cycle %d: pc: 0x%lx ir: 0x%x\n",
                    (int)cycle, s.pc, s.ir);
        cycle++;

        /* handle HTIF requests */
        if ((htifexit = htif(s.mem, htifaddr, cmd.args)) & 1)
            break;
    }
    if (cmd.filetype == 1 && cmd.debug)
        disasmem(&s.mem[s.pc], hexsz), print(s, 0x10010000, 256);

    if (cycle > cmd.maxtime)
        fprintf(stderr, "[Info] Exceeded maximum cycle %d\n", cmd.maxtime);
    if (htifexit & 1)
        fprintf(stderr, "[Info] Exited with code %hhu\n", (int)htifexit >> 1);
    return htifexit >> 1;
}
