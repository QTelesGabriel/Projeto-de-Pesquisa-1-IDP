FROM ubuntu:24.04

# Evita perguntas geográficas durante a instalação
ENV DEBIAN_FRONTEND=noninteractive

# Instala dependências básicas, Gazebo e editores úteis
RUN apt-get update && apt-get install -y \
    sudo wget gnupg2 lsb-release git python3-pip nano \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    libgz-sim8-dev \
    libgz-cmake3-dev \
    libgz-msgs10-dev \
    libgz-transport13-dev \
    python3-gz-sim8 \
    && apt-get clean

# Remove o usuário padrão 'ubuntu' (UID 1000) se ele existir, para evitar conflito
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi

# Agora criamos o usuário gabriel com o UID 1000 com segurança
RUN useradd -m -u 1000 gabriel && \
    echo "gabriel:gabriel" | chpasswd && \
    adduser gabriel sudo

# Garante permissões na pasta
RUN mkdir -p /home/gabriel/project && chown -R gabriel:gabriel /home/gabriel/project

USER gabriel
WORKDIR /home/gabriel/project

CMD ["/bin/bash"]
