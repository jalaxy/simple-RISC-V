bashdir=$(dirname $BASH_SOURCE)
for file in $bashdir/*.dump; do
    $bashdir/../obj_dir/Vstats -t 1024 -d $file
done