# ===== Select compose (dev|payload|recon|water|relay) =====
C ?= dev
DOMAIN ?= 2
TCP_PORT ?= 5762
# ----- Paths / names -----
COMPOSE_FILE := compose/$(C).yml
CONTAINER    ?= aeac-$(C)

DRONE_IP ?= 192.168.0.13

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
COMPOSES := compose/dev.yml compose/payload.yml compose/recon.yml compose/water.yml compose/relay.yml

# ===== Pretty help =====
help: ## Show help
	@awk 'BEGIN{FS":.*##"; printf "\nTargets (use C=<dev|payload|recon|water>):\n"} /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

print-vars: ## Show resolved variables (debug)
	@echo "C=$(C)"
	@echo "COMPOSE_FILE=$(COMPOSE_FILE)"
	@echo "CONTAINER=$(CONTAINER)"
	@echo "WS_REL=$(WS_REL)"
	@echo "WS_IN=$(WS_IN)"


relay: ## Start SIYI relay (UDP 14540 → MAVROS, Pymavlink, Mission Planner)
	docker compose -f compose/relay.yml up -d --build && \
	docker compose -f compose/relay.yml exec -it mavlink-router \
	  sh -lc 'command -v bash >/dev/null && exec bash -i || exec sh -l'


relay-down:
	@docker compose -f compose/relay.yml down

# ===== Docker lifecycle (selected compose) =====
build: ## Build image for C
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) build


up: ## Up (detached) for C, rebuild if needed
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) up -d --build

down: ## Down for C (remove orphans)
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) down --remove-orphans

shell sh bash: ## Open bash in the running container (with ROS sourced)
	docker exec -it $(CONTAINER) bash -lc "export ROS_DOMAIN_ID=$(DOMAIN) && source /opt/ros/humble/setup.bash && ros2 daemon start && exec bash -i"

# ===== Workspace helpers =====
link: 
	docker exec -it $(CONTAINER) bash -lc 'cd "$(REPO_IN)" && ./scripts/link_ws.sh "$(WS_REL)"'


connect: up    
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && \
	  export RMW_IMPLEMENTATION=rmw_zenoh_cpp && \
	  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$(DRONE_IP):7447\"]" && \
	  source /opt/ros/humble/setup.bash && ros2 daemon start && bash'


launch: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	bash -lc 'source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 & exec bash -lc "source /opt/ros/humble/setup.bash && ros2 daemon start && bash"'


mavros-sim: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'export ROS_DOMAIN_ID=$(DOMAIN); \
	  source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 & \
	  ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:$(TCP_PORT) fcu_protocol:=v2.0'


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

nuke-all: ## Stop everything + remove images/volumes + wipe all WS artifacts
	$(MAKE) down-all || true
	$(MAKE) nuke-images || true
	@for ws in $(patsubst compose/%.yml,workspaces/%_ws,$(COMPOSES)); do \
	  echo "*** cleaning $$ws"; sudo rm -rf "$$ws/build" "$$ws/install" "$$ws/log"; \
	done

.PHONY: help print-vars build up down connect mavros-sim shell sh bash launch clean \
        build-all up-all down-all nuke-images nuke-all relay relay-down
