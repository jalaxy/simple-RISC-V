if [ $1 ] && [ $2 ]
then
    docker run --rm -w /work -v .:/work docker.io/jxy324/riscv-toolchain:v0 bash -c \
        "/opt/riscv/bin/riscv64-unknown-elf-gcc /work/$1 -o /work/$2 -march=rv64imafd
         /opt/riscv/bin/spike pk /work/$2"
else
    echo "Not enough arguments"
fi
