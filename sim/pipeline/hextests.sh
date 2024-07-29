for file in ../../util/hexcode/*; do
    ./main -s -hex $file >/dev/null 2>&1
    exitcode=$?
    if [ $exitcode -ne 0 ]; then
        echo "$file exited with code $exitcode"
    fi
done