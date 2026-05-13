# Tutorial de como acessar a câmera do Drone

## Instalar a bridge e Criar Ponte da Câmera

```bash
# Teste
ros2 run ros_gz_bridge parameter_bridge \
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image

# Se não existir rode o seguinte comando e depois tente novamente:
sudo apt install ros-jazzy-ros-gz -y

```

## Verificar o ROS2

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic list

# Você deve ver:
# /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

## Visualizar a imagem

```bash
# Instale 
sudo apt install ros-jazzy-rqt-image-view -y

# Depois:
ros2 run rqt_image_view rqt_image_view
```