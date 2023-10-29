docker run -it -w /work --net=host -e DISPLAY \
    -v .:/work -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority ubuntu:22.04