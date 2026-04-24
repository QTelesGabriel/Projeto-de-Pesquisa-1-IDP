# 1. Ativar o ambiente virtual
source ~/project/.venv_drone/bin/activate

# 2. Configurar os caminhos para o Gazebo encontrar os modelos
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/project/ardupilot_gazebo/worlds:~/project/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/project/ardupilot_gazebo/build

# 3. Iniciar o simulador na pista de pouso
cd ~/project/ardupilot_gazebo/worlds
gz sim -v4 -r iris_runway.sdf
