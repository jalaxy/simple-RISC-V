DOCKER = docker
CC = $(DOCKER) run --rm -it -v `pwd`:/work -w /work \
	docker.io/jxy324/riscv-toolchain:v0 /opt/riscv/bin/riscv64-unknown-elf-gcc
PROG = hello add

all : $(PROG)

clean :
	$(RM) $(PROG)
