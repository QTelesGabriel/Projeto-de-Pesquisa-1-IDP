#!/bin/bash
xhost +local:docker

# Inicia o container caso esteja parado
docker start drone_tcc

# Entra no terminal principal
docker exec -it drone_tcc /bin/bash
