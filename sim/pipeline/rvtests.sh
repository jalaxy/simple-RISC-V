# run isa tests
echo "ISA tests errors:"
for file in ../../util/riscv-tests-bin/isa/*; do
    ./main -s -elf $file >/dev/null 2>&1
    exitcode=$?
    if [ $exitcode -ne 0 ]; then
        echo "$file exited with code $exitcode"
    fi
done

# run benchmarks
pids=()
names=()
for file in ../../util/riscv-tests-bin/*.riscv; do
    touch /tmp/$(basename $file)
    ./main -s -elf $file >/tmp/$(basename $file) 2>&1 &
    pids+=($!)
    names+=(/tmp/$(basename $file))
done
for i in ${!pids[@]}; do
    wait ${pids[i]}
    echo "****************************************************************"
    cat ${names[i]}
done
for name in ${names[@]}; do
    rm $name
done
