bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/*.dump; do
    $bashdir/../obj_dir/Vstats -dump -t 0 4096 -d $file
done