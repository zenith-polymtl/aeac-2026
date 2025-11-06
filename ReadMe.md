chmod +x scripts/*.sh

echo "WS=workspaces/ws_heavy_task_2" > .env   # or small_ws, ws_heavy_task_1

# Alias
alias aeac-build='docker exec -it aeac-dev bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"'
alias aeac-run='docker exec -it aeac-dev bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch mission_core_big_t2 dev_sitl.launch.py"'


docker exec -it aeac-dev bash