export DISPLAY=localhost:10.0
docker run --rm -it -w /work/sim/pipeline --net=host -e DISPLAY \
    -v .:/work -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/fonts:/etc/fonts \
    -v ~/.Xauthority:/root/.Xauthority docker.io/jxy324/riscv-sim:verilator