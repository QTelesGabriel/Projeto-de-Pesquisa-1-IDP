#!/bin/bash
# 1. Permite acesso ao X11
xhost +local:docker

# 2. Build da imagem (garante que está atualizada)
docker build -t drone-sim-image .

# 3. Remove container antigo se existir
docker rm -f drone_tcc 2>/dev/null

# 4. Cria e roda o container
docker run -it \
    --name drone_tcc \
    --gpus all \
    --device /dev/nvidia0 \
    --device /dev/nvidiactl \
    --device /dev/nvidia-modeset \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    --privileged \
    -v $(pwd):/home/gabriel/project \
    drone-sim-image /bin/bash
