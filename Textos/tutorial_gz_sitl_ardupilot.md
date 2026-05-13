# Tutorial de Instalação Ubuntu 24.04 LTS GAZEBO HARMONIC + SILT + ARDUPILOT

## 1. Atualização do Sistema e Pacotes Base

```bash
# Entrar na sua pasta de projeto
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Atualizar repositórios e pacotes do sistema
sudo apt update && sudo apt dist-upgrade -y

# Instalar ferramentas essenciais de terminal e Python
sudo apt install -y htop neofetch git gcc make curl bzip2 tar unzip python3-pip
```

## 2. Código Fonte do ArduPilot e Dependências

```bash
# Garantir que está na pasta correta
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Clonar o repositório com todos os submódulos
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

# Entrar na pasta clonada
cd ardupilot

# Executar o script oficial de pré-requisitos (instala compiladores e libs)
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Recarregar o perfil do terminal para aplicar as mudanças de PATH
. ~/.profile

# Instala a versão exata que o ArduPilot exige
pip install empy==3.3.4
```

## 3. Compilação do ArduPilot SITL

```bash
# Entrar na pasta do ardupilot (se já não estiver nela)
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ardupilot

# Configurar o ambiente de build para o SITL
./waf configure --board sitl

# Compilar o firmware para Copter e Plane
./waf copter
./waf plane
```

## 4. Instalação do MAVProxy e Pymavlink

```bash
# Voltar para a base do projeto
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Instalar bibliotecas de controle via Python
# Nota: O uso de --break-system-packages é necessário no Ubuntu 24.04 para instalações fora de venv
pip install --user --upgrade pymavlink MAVProxy --break-system-packages
```

## 5. Gazebo Harmonic

```bash
# Entrar na sua pasta de projeto
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Atualizar a lista de pacotes
sudo apt-get update

# Instalar ferramentas para gerenciar repositórios e chaves
sudo apt-get install -y lsb-release wget gnupg

# Baixar e instalar a chave GPG do repositório Gazebo
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Adicionar o repositório estável do Gazebo às fontes do sistema
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Atualizar novamente para incluir o novo repositório
sudo apt-get update

# Instalar o Gazebo Harmonic e todas as dependências de simulação e visão (OpenCV/GStreamer)
sudo apt-get install -y gz-harmonic libgz-sim8-dev rapidjson-dev libopencv-dev \
libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
gstreamer1.0-libav gstreamer1.0-gl libgz-transport13-dev libgz-msgs10-dev
```

## 6. Instalação do Plugin Ardupilot Gazebo

```bash
# Garantir que está na base do projeto
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Clonar o repositório do plugin
git clone https://github.com/ArduPilot/ardupilot_gazebo

# Entrar na pasta do plugin
cd ardupilot_gazebo

# Criar e entrar na pasta de build para compilação
mkdir build && cd build

# Configurar o projeto com CMake (modo RelWithDebInfo para melhor performance com debug)
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Compilar o plugin usando todos os núcleos do processador do seu Nitro
make -j$(nproc)
```

## 7. Definição de Variáveis de Ambiente

```bash
# Entrar na sua pasta base
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code

# Adicionar o caminho do plugin compilado às variáveis do Gazebo
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc

# Adicionar os caminhos dos modelos e dos mundos (essencial para carregar a pista e o drone)
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/models:$HOME/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

# Atualizar o terminal atual com as novas configurações
source ~/.bashrc
```

## Limpeza para a execução

OBS: Sempre fazer antes de usar

```bash
# Mata qualquer processo remanescente do ArduPilot, MAVProxy ou Gazebo
pkill -9 -f "ardupilot|mavproxy|gz|sim_vehicle|ruby"
```

## Execução

### Terminal 1

```bash
source ~/.profile
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code
gz sim -v4 -r iris_runway.sdf
```

### Terminal 2

```bash
source ~/.profile

# Entrar no diretório específico do firmware
cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ardupilot/ArduCopter

# Iniciar o script de simulação com os parâmetros para Gazebo Harmonic
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```