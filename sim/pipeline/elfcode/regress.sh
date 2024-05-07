bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/*; do
    $bashdir/../obj_dir/Vstats -d -elf $file
done