bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/isa/*; do
    $bashdir/../obj_dir/Vstats -elf $file > /dev/null
    exitcode=$?
    if [ $exitcode -ne 0 ]; then
        echo "$file exited with code $exitcode"
    fi
done