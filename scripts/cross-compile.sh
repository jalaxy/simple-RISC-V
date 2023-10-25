if [ -n $1 ] && [ -n $2 ]
then
    docker run --rm -v .:/work docker.io/jxy324/riscv-toolchain:v0 bash -c \
        "/opt/riscv/bin/riscv64-unknown-elf-gcc /work/$1 -o /work/$2"
fi
