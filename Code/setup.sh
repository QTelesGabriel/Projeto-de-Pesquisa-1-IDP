#!/usr/bin/env bash
# ==============================================================================
# SCRIPT DE INICIALIZAÇÃO E INSTALAÇÃO COMPLETA
# Projeto de Pesquisa 1 - IDP (Drone Coaxial + ArduPilot + Gazebo + ROS 2 + YOLO)
# Compatibilidade: Ubuntu 24.04 LTS
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "======================================================================"
echo " [1/7] Verificando Diretórios e Sistema Operacional"
echo "======================================================================"
echo "Diretório do Projeto: $PROJECT_ROOT"
echo "Diretório de Código:  $SCRIPT_DIR"

if ! grep -qi "ubuntu" /etc/os-release; then
    echo "[AVISO] Este script foi otimizado para Ubuntu 24.04 LTS."
fi

# 1. Dependências Base e Ferramentas do Sistema
echo ""
echo "======================================================================"
echo " [2/7] Instalando Pacotes Base do Sistema e Utilitários"
echo "======================================================================"
sudo apt update
sudo apt install -y \
    locales \
    curl \
    wget \
    gnupg \
    gnupg2 \
    software-properties-common \
    git \
    gcc \
    make \
    bzip2 \
    tar \
    unzip \
    cmake \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    python3-pip \
    python3-venv \
    python3-dev \
    htop

# Configuração de Locales (Necessário para ROS 2 e ArduPilot)
sudo locale-gen en_US en_US.UTF-8 > /dev/null 2>&1 || true
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 > /dev/null 2>&1 || true
export LANG=en_US.UTF-8

# 2. Gazebo Harmonic
echo ""
echo "======================================================================"
echo " [3/7] Instalando Gazebo Harmonic e Bibliotecas de Simulação"
echo "======================================================================"
if ! dpkg -s gz-harmonic >/dev/null 2>&1; then
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt update
    sudo apt install -y \
        gz-harmonic \
        libgz-sim8-dev \
        libgz-transport13-dev \
        libgz-msgs10-dev
else
    echo "Gazebo Harmonic já instalado."
fi

# 3. ROS 2 Jazzy
echo ""
echo "======================================================================"
echo " [4/7] Instalando ROS 2 Jazzy e Pontes (Bridges)"
echo "======================================================================"
if [ ! -d "/opt/ros/jazzy" ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
else
    echo "ROS 2 Jazzy já instalado."
fi

# Pacotes adicionais de ROS 2 para ponte Gazebo e Visão
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rqt-image-view \
    python3-opencv

# 4. ArduPilot SITL
echo ""
echo "======================================================================"
echo " [5/7] Configurando ArduPilot SITL"
echo "======================================================================"
if [ ! -d "$SCRIPT_DIR/ardupilot" ]; then
    echo "Clonando ArduPilot oficial com submódulos..."
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$SCRIPT_DIR/ardupilot"
    cd "$SCRIPT_DIR/ardupilot"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile || true
    ./waf configure --board sitl
    ./waf copter
else
    echo "Pasta ArduPilot já presente em: $SCRIPT_DIR/ardupilot"
    if [ ! -f "$SCRIPT_DIR/ardupilot/build/sitl/bin/arducopter" ]; then
        echo "Compilando firmware SITL ArduCopter..."
        cd "$SCRIPT_DIR/ardupilot"
        ./waf configure --board sitl
        ./waf copter
    fi
fi

# 5. Plugin ArduPilot Gazebo e Modelos Customizados
echo ""
echo "======================================================================"
echo " [6/7] Compilando Plugin ArduPilot Gazebo e Restaurando Modelos do TCC"
echo "======================================================================"
if [ ! -d "$SCRIPT_DIR/ardupilot_gazebo" ]; then
    echo "Clonando plugin oficial ardupilot_gazebo..."
    git clone https://github.com/ArduPilot/ardupilot_gazebo "$SCRIPT_DIR/ardupilot_gazebo"
fi

cd "$SCRIPT_DIR/ardupilot_gazebo"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j"$(nproc)"

# Copiar os modelos exclusivos do projeto para o diretório do plugin se necessário
echo "Garantindo modelos da simulação (drone_coaxial, armadilha, grama, tcc.sdf)..."
mkdir -p "$SCRIPT_DIR/ardupilot_gazebo/models" "$SCRIPT_DIR/ardupilot_gazebo/worlds"
if [ -d "$SCRIPT_DIR/simulacao_gazebo/models" ]; then
    cp -ru "$SCRIPT_DIR/simulacao_gazebo/models/"* "$SCRIPT_DIR/ardupilot_gazebo/models/"
fi
if [ -d "$SCRIPT_DIR/simulacao_gazebo/worlds" ]; then
    cp -ru "$SCRIPT_DIR/simulacao_gazebo/worlds/"* "$SCRIPT_DIR/ardupilot_gazebo/worlds/"
fi

# 6. Ambiente Virtual Python e Dependências do YOLO
echo ""
echo "======================================================================"
echo " [7/7] Configurando Ambiente Python e Dependências de IA"
echo "======================================================================"
VENV_DIR="$SCRIPT_DIR/venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Criando virtualenv com acesso aos pacotes ROS 2 (--system-site-packages)..."
    python3 -m venv --system-site-packages "$VENV_DIR"
fi

echo "Instalando dependências de Python (ultralytics, dronekit, pymavlink, etc.)..."
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"

# 7. Configuração de Variáveis de Ambiente no ~/.bashrc
BASHRC="$HOME/.bashrc"
echo ""
echo "Atualizando variáveis no ~/.bashrc..."

grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC" 2>/dev/null || echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"

GZ_PLUGIN_LINE="export GZ_SIM_SYSTEM_PLUGIN_PATH=\"$SCRIPT_DIR/ardupilot_gazebo/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH}\""
grep -q "ardupilot_gazebo/build" "$BASHRC" 2>/dev/null || echo "$GZ_PLUGIN_LINE" >> "$BASHRC"

GZ_RESOURCE_LINE="export GZ_SIM_RESOURCE_PATH=\"$SCRIPT_DIR/simulacao_gazebo/models:$SCRIPT_DIR/simulacao_gazebo/worlds:$SCRIPT_DIR/ardupilot_gazebo/models:$SCRIPT_DIR/ardupilot_gazebo/worlds:/opt/ros/jazzy/share:\${GZ_SIM_RESOURCE_PATH}\""
grep -q "simulacao_gazebo" "$BASHRC" 2>/dev/null || echo "$GZ_RESOURCE_LINE" >> "$BASHRC"

echo ""
echo "======================================================================"
echo " [✓] Instalação Concluída com Sucesso!"
echo "======================================================================"
echo "Para carregar o novo ambiente no seu terminal atual, execute:"
echo "    source ~/.bashrc"
echo "    source $VENV_DIR/bin/activate"
echo ""
echo "Para executar a simulação e missão, siga o arquivo 'Textos/como_ligar.txt'."
echo "======================================================================"
