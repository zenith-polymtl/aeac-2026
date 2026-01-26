Pour démarrer le conteneur docker : 
docker start aeac-
docker exec -it aeac- bash

Dans une autre terminale
cd compose
docker compose -f dev.yml up --build 

Rouler le programme
colcon build
source install/setup.bash
ros2 pkg list | grep bringup      # vérifier que ROS voit le package
ros2 launch bringup phase1.launch.py





Changement apporté dans le dockerfile.dev : 
Tu remplaces cette ligne : COPY packages/ros2_gremsy /app/packages/ros2_gremsy

par celle-ci 

COPY packages/src/ros2_gremsy /app/packages/ros2_gremsy

