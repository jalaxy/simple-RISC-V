bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/*.dump; do
    $bashdir/../obj_dir/Vstats -t 0 4096 -d $file
done