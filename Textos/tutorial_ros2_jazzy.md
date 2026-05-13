# Tutorial de instalação do ROS 2 Jazzy

## Configurar Locales

```bash
sudo apt update
sudo apt install locales -y

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

# Para testar se funcionou
locale
```

## Adicionar o Repositório do ROS2

```bash
# Instale as dependências
sudo apt install software-properties-common curl gnupg2 -y

# Adicione a chave
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

# Adicione o repositório
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Instalar ROS 2 Jazzy

```bash
# Atualize
sudo apt update

# Agora instale
sudo apt install ros-jazzy-desktop -y
```

## Configurar o Ambiente

```bash
# Adicione ao bashrc:
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Atualize
source ~/.bashrc

# Testar
ros2
```

## Instalar a Bridge do Gazebo

```bash
# Agora instale
sudo apt install ros-jazzy-ros-gz -y
```

## Instalar Ferramentas de Visão

```bash
sudo apt install \
ros-jazzy-cv-bridge \
ros-jazzy-image-transport \
ros-jazzy-rqt-image-view \
python3-opencv \
-y
```

## Teste final

```bash
ros2 topic list
```
