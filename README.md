# simple-RISC-V
Simple Verilog HDL implementation of RISC-V.

## Behavior simulation

### Prerequisites

Running a behavior simulation requires docker environment.

### Generate RISC-V executables

the code running in RISC-V CPU can be compiled from C/C++ source code. It is also recommended to run in a container. Using
```
./scripts/riscv-bash.bash
```
will enter a container with RISC-V toolchain installed, or simply use
```
./scripts/cross-compile.sh <C-source> <ELF-executable>
```
to generate ELF executables.

### Running with verilator

First docker environment is suggested, so use
```
./scripts/verilator-bash.sh
```
or modified command inside to create and enter a container.

Inside appropriate environment, verilator compiles the verilog code and generates obj_dir directory with C++ source code and an executable inside.
Use following bash command to make excutable objects.
```
make verilator
```
The executable file is ```obj_dir/V<top-module-name>```. For example, with top module ```core_tb``` and ELF file ```test.elf```, the execution would be
```
./obj_dir/Vcore_tb test.elf
```
and it will output registers info, etc.
