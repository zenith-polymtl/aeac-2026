# ===== Select compose (dev|payload|water|relay) =====
C ?= dev
DOMAIN ?= 2
TCP_PORT ?= 5762
# ----- Paths / names -----
COMPOSE_FILE := compose/$(C).yml
CONTAINER    ?= aeac-$(C)

# Workspace path RELATIVE to repo root (e.g., workspaces/dev_ws)
WS_REL := workspaces/$(C)_ws

# In-container paths
REPO_IN := /aeac
WS_IN   := $(REPO_IN)/$(WS_REL)

#Relay settings
DEVICE ?= /dev/ttyUSB0
BAUD   ?= 57600

# Host UID/GID so files aren’t root-owned
UID ?= $(shell id -u)
GID ?= $(shell id -g)
export UID GID

# Inject BOTH variables so compose can use them
ENV_INJECT := C=$(C) WS=$(WS_REL)

# All compose files for the *-all targets
COMPOSES := compose/dev.yml compose/payload.yml compose/water.yml compose/mavros.yml compose/zed.yml compose/zenoh-air.yml compose/zenoh-ground.yml

# ===== Pretty help =====
help: ## Show help
	@awk 'BEGIN{FS":.*##"; printf "\nTargets (use C=<dev|payload|water|relay>):\n"} /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

print-vars: ## Show resolved variables (debug)
	@echo "C=$(C)"
	@echo "COMPOSE_FILE=$(COMPOSE_FILE)"
	@echo "CONTAINER=$(CONTAINER)"
	@echo "WS_REL=$(WS_REL)"
	@echo "WS_IN=$(WS_IN)"


zed-shell: ## Open bash in the ZED container (with ROS sourced)
	docker compose -f compose/zed.yml exec -it zed-ros2 bash -lc '\
	  source /root/ros2_ws/install/setup.bash; \
	  exec bash -i'

zed-launch:
	docker compose -f compose/zed.yml up -d zed-ros2
	docker compose -f compose/zed.yml exec -it zed-ros2 bash -lc '\
	  source /root/ros2_ws/install/setup.bash; \
	  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i ros_params_override_path:=config/zenith_stereo.yaml publish_tf:=false publish_map_tf:=false \
	'
zed-launch-mini:
	docker compose -f compose/zed.yml up -d zed-ros2
	docker compose -f compose/zed.yml exec -it zed-ros2 bash -lc '\
	  source /root/ros2_ws/install/setup.bash; \
	  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm ros_params_override_path:=config/zenith_stereo_mini.yaml \
	'

zenoh-ground: ## Start zenoh ground bridge
	docker compose -f compose/zenoh-ground.yml up

zenoh-air: ## Start zenoh air bridge
	docker compose -f compose/zenoh-air.yml up

zenoh-ground-build: ## Start zenoh ground bridge
	docker compose -f compose/zenoh-ground.yml up --build

zenoh-air-build: ## Start zenoh air bridge
	docker compose -f compose/zenoh-air.yml up --build


# ===== Docker lifecycle (selected compose) =====
build: ## Build image for C
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) build


up: ## Up (detached) for C, rebuild if needed
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) up -d

down: ## Down for C (remove orphans)
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) down --remove-orphans

shell sh bash: ## Open bash in the running container (with ROS sourced)
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    exec bash -i'

# ===== Workspace helpers =====
link: 
	docker exec -it $(CONTAINER) bash -lc 'cd "$(REPO_IN)" && ./scripts/link_ws.sh "$(WS_REL)"'


connect: up    
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && \
	  source /opt/ros/humble/setup.bash && ros2 daemon start && bash'


launch: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    exec bash -i'

payload:
	docker compose -f compose/payload.yml up -d --build
	# Launch ZED inside the already-running zed-ros2 service (detached)
	docker compose -f compose/payload.yml exec -T zed-ros2 bash -lc " \
		source /root/ros2_ws/install/setup.bash && \
		nohup ros2 launch zed_wrapper zed_camera.launch.py \
			camera_model:=zed2i \
			ros_params_override_path:=config/zenith_stereo.yaml \
			> /tmp/zed_launch.log 2>&1 & \
	"


	# Enter payload dev shell (your original behavior)
	WS=$(WS_IN) docker compose -f compose/payload.yml exec -it payload \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    exec bash -i'

water-stack:
	docker compose -f compose/water.yml up -d
	
	docker compose -f compose/water.yml exec -T zed-ros2 bash -lc " \
		source /root/ros2_ws/install/setup.bash && \
		nohup ros2 launch zed_wrapper zed_camera.launch.py \
			camera_model:=zed2i \
			ros_params_override_path:=config/zenith_stereo.yaml \
			publish_tf:=false \
			publish_map_tf:=false \
			> /tmp/zed_launch.log 2>&1 & \
	"

	# Enter water dev shell (your original behavior)
	WS=$(WS_IN) docker compose -f compose/water.yml exec -it water \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    exec bash -i'

mavros-sim: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:$(TCP_PORT) fcu_protocol:=v2.0'

mavros-gazebo: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=udp://:14550@ fcu_protocol:=v2.0 use_sim_time:=true'

mavros-jetson: 
	docker compose -f compose/mavros.yml up -d --build
	docker compose -f compose/mavros.yml exec -T mavros bash -lc '\
	source /opt/ros/humble/setup.bash; \
	ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600 fcu_protocol:=v2.0'

mavros-restart:
	sudo systemctl restart mavros-jetson

mavros-status:
	systemctl status mavros-jetson --no-pager

mavros-logs:
	journalctl -u mavros-jetson -f

zed-restart:
	sudo systemctl restart zed

zed-status:
	systemctl status zed --no-pager

zed-logs:
	journalctl -u zed -f

zed-mini-restart:
	sudo systemctl restart zed-mini

zed-mini-status:
	systemctl status zed-mini --no-pager

zed-mini-logs:
	journalctl -u zed-mini -f

zenoh-restart:
	sudo systemctl restart zenoh-air

zenoh-status:
	systemctl status zenoh-air --no-pager

zenoh-logs:
	journalctl -u zenoh-air -f

mavros-ofa: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyAMA10:115200'

water:
	docker compose -f compose/water.yml up -d

	# Launch water mission in the container session
	WS=$(WS_IN) docker compose -f compose/water.yml exec -it water \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    ros2 launch bringup water_mission.launch.py'

water-build:
	docker compose -f compose/water.yml up -d

	# Launch water mission in the container session
	WS=$(WS_IN) docker compose -f compose/water.yml exec -it water \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build'

payload:
	docker compose -f compose/payload.yml up -d

	WS=$(WS_IN) docker compose -f compose/payload.yml exec -it payload \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    ros2 launch bringup payload_mission.launch.py'

payload-build:
	docker compose -f compose/payload.yml up -d

	WS=$(WS_IN) docker compose -f compose/payload.yml exec -it payload \
	  bash -lc '\
	    cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build'

gcs-payload: up
	docker compose -f compose/zenoh-ground.yml up -d

	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    ros2 daemon start; \
		source install/setup.bash; \
	    ros2 run payload_web_server_node payload_web_server_node'

gcs-payload-build: up
	docker compose -f compose/zenoh-ground.yml up -d

	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
		colcon build --packages-select payload_web_server_node custom_interfaces; \
	    ros2 daemon start; \
		source install/setup.bash; \
	    ros2 run payload_web_server_node payload_web_server_node'

gcs-water: up
	docker compose -f compose/zenoh-ground.yml up -d
	
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
		colcon build --packages-select water_web_server_node custom_interfaces; \
	    source install/setup.bash; \
		ros2 daemon start; \
	    ros2 run water_web_server_node water_web_server_node'

gcs-water-build: up
	docker compose -f compose/zenoh-ground.yml up -d
	
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
		colcon build --packages-select water_web_server_node custom_interfaces; \
	    source install/setup.bash; \
		ros2 daemon start; \
	    ros2 run water_web_server_node water_web_server_node'

rviz: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  rviz2'

foxglove: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'

clean: ## Remove build/install/log (host + container)
	sudo rm -rf "$(WS_REL)/build" "$(WS_REL)/install" "$(WS_REL)/log" "workspaces/install" "workspaces/log" "workspaces/build"
	-docker exec -it $(CONTAINER) bash -lc 'rm -rf "$(WS_IN)/build" "$(WS_IN)/install" "$(WS_IN)/log"'

# ===== Multi-compose convenience =====
build-all: ## Build all compose files
	@for f in $(COMPOSES); do \
	  C=$$(basename $$f .yml); WS=workspaces/$${C}_ws docker compose -f $$f build || exit $$?; \
	done

up-all: ## Up all compose files (detached)
	@for f in $(COMPOSES); do \
	  C=$$(basename $$f .yml); WS=workspaces/$${C}_ws docker compose -f $$f up -d --build || exit $$?; \
	done

down-all: ## Down all compose files + orphans
	@for f in $(COMPOSES); do \
	  C=$$(basename $$f .yml); WS=workspaces/$${C}_ws docker compose -f $$f down --remove-orphans || true; \
	done

delete-images: ## Delete local images/volumes for these composes
	@for f in $(COMPOSES); do \
	  C=$$(basename $$f .yml); WS=workspaces/$${C}_ws docker compose -f $$f down --rmi local --volumes --remove-orphans || true; \
	done

nuke-images: delete-images ## Alias for delete-images

nuke-all: ## Stop everything + remove images/volumes + wipe all WS artifacts
	$(MAKE) down-all || true
	$(MAKE) nuke-images || true
	@for ws in $(patsubst compose/%.yml,workspaces/%_ws,$(COMPOSES)); do \
	  echo "*** cleaning $$ws"; sudo rm -rf "$$ws/build" "$$ws/install" "$$ws/log"; \
	done

.PHONY: help print-vars build up down connect mavros-sim shell sh bash launch clean \
	build-all up-all down-all nuke-images nuke-all relay relay-down \
	mavros-restart mavros-status mavros-logs \
	zed-restart zed-status zed-logs \
	zenoh-restart zenoh-status zenoh-logs
