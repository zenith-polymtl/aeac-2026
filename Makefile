# ===== Select compose (dev|payload|recon|water) =====
C ?= dev

# ----- Paths / names -----
COMPOSE_FILE := compose/$(C).yml
CONTAINER    ?= aeac-$(C)

# Workspace path RELATIVE to repo root (e.g., workspaces/dev_ws)
WS_REL := workspaces/$(C)_ws

# In-container paths
REPO_IN := /aeac
WS_IN   := $(REPO_IN)/$(WS_REL)

# Host UID/GID so files aren’t root-owned
UID ?= $(shell id -u)
GID ?= $(shell id -g)
export UID GID

# Inject BOTH variables so compose can use them
ENV_INJECT := C=$(C) WS=$(WS_REL)

# All compose files for the *-all targets
COMPOSES := compose/dev.yml compose/payload.yml compose/recon.yml compose/water.yml

# ===== Pretty help =====
help: ## Show help
	@awk 'BEGIN{FS":.*##"; printf "\nTargets (use C=<dev|payload|recon|water>):\n"} /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

print-vars: ## Show resolved variables (debug)
	@echo "C=$(C)"
	@echo "COMPOSE_FILE=$(COMPOSE_FILE)"
	@echo "CONTAINER=$(CONTAINER)"
	@echo "WS_REL=$(WS_REL)"
	@echo "WS_IN=$(WS_IN)"

# ===== Docker lifecycle (selected compose) =====
build: ## Build image for C
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) build

up: ## Up (detached) for C, rebuild if needed
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) up -d --build

down: ## Down for C (remove orphans)
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) down --remove-orphans

shell sh bash: ## Open bash in the running container (with ROS sourced)
	docker exec -it $(CONTAINER) bash -lc "source /opt/ros/humble/setup.bash && exec bash -i"

# ===== Workspace helpers =====
link: 
	docker exec -it $(CONTAINER) bash -lc 'mkdir -p "$(WS_IN)/src" && cd "$(REPO_IN)" && ./scripts/link_ws.sh "$(WS_REL)"'

launch: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'colcon build && source install/setup.bash && exec bash -i'


clean: ## Remove build/install/log (host + container)
	-rm -rf "$(WS_REL)/build" "$(WS_REL)/install" "$(WS_REL)/log"
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
	  echo "*** cleaning $$ws"; rm -rf "$$ws/build" "$$ws/install" "$$ws/log"; \
	done

.PHONY: help print-vars build up down shell sh bash launch clean \
        build-all up-all down-all nuke-images nuke-all
