# run isa tests
echo "ISA tests errors:"
bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/isa/*; do
    $bashdir/../obj_dir/Vstats -elf $file > /dev/null
    exitcode=$?
    if [ $exitcode -ne 0 ]; then
        echo "$file exited with code $exitcode"
    fi
done
# run benchmarks
pids=()
names=()
for file in $bashdir/*.riscv; do
    touch /tmp/$(basename $file)
    $bashdir/../obj_dir/Vstats -elf $file > /tmp/$(basename $file) &
    pids+=($!)
    names+=(/tmp/$(basename $file))
done
for i in ${!pids[@]}; do
    wait ${pids[i]}
    echo "****************************************************************"
    cat ${names[i]}
done
for name in $names; do
    rm $name
done
