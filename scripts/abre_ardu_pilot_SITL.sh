# 1. Entrar no container existente
docker exec -it drone_tcc /bin/bash

# 2. Ativar o ambiente virtual
source ~/project/.venv_drone/bin/activate

# 3. Ir para a pasta do veículo e iniciar o código de voo
cd ~/project/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
