# AEAC-2026 – Autonomous Drone System

ROS 2 (Humble) Docker-based development environment with mission-specific workspaces for the AEAC 2026 competition. Features MAVLink integration, Zenoh networking, and ZED camera support.

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/zenith-polymtl/aeac-2026)

## Prerequisites

* Docker (with BuildKit support)
* Git
* Linux host (or WSL2 on Windows)

## Quick Start

```bash
# Clone repository
git clone <repo-url> aeac-2026
cd aeac-2026

# Initialize submodules (if any)
git submodule update --init --recursive

# Choose your environment and launch
make C=dev up        # Start dev environment
make C=dev shell     # Open interactive shell
```

## Repository Structure [1](#0-0) 

The repository contains:
- **`compose/`**: Docker Compose files for each environment (dev, payload, water)
- **`docker/`**: Dockerfiles for each environment
- **`workspaces/`**: ROS 2 workspaces for each mission
- **`packages/`**: ROS 2 packages (managed as submodules)
- **`config/`**: Configuration files (Zenoh, ZED camera)
- **`scripts/`**: Utility scripts for workspace management

## Environments Overview

### 🖥️ **Dev Environment** (Personal/GCS Computers) [2](#0-1) 

**Use Case**: Development, testing, and ground control station (GCS) operations

**Features**:
- Full ROS 2 Humble with visualization tools (RViz2)
- CycloneDDS middleware for local development
- X11 forwarding for GUI applications
- Zenoh bridge for connecting to drones

**Launch**:
```bash
make C=dev up        # Start container (detached)
make C=dev shell     # Open shell with ROS sourced
```

### 🚁 **Payload Environment** (Mission 1 - Drone) [3](#0-2) 

**Use Case**: Payload delivery mission on the drone

**Features**:
- Optimized for drone deployment
- CycloneDDS with `LOCALHOST` discovery for reduced network traffic
- ZED camera integration for vision tasks
- Zenoh air bridge for ground communication

**Packages Included**: [4](#0-3) 

**Launch on Drone**:
```bash
make C=payload up
make C=payload shell
# Inside container:
colcon build
source install/setup.bash
ros2 launch bringup payload_mission.launch.py
```

### 💧 **Water Environment** (Mission 2 - Drone) [5](#0-4) 

**Use Case**: Water-based mission (Mission 2) on the drone

**Features**:
- Zenoh middleware for improved network performance
- Gremsy gimbal support
- Zenoh air bridge for ground communication

**Packages Included**: [6](#0-5) 

**Launch on Drone**:
```bash
make C=water up
make C=water shell
# Inside container:
colcon build
source install/setup.bash
ros2 launch bringup water_mission.launch.py
```

## Common Makefile Commands [7](#0-6) 

### Essential Commands

| Command | Description |
|---------|-------------|
| `make help` | Show all available targets |
| `make C=<env> up` | Build and start container (detached) |
| `make C=<env> down` | Stop and remove container |
| `make C=<env> shell` | Open bash with ROS environment sourced |
| `make C=<env> launch` | Build workspace and launch interactive shell |
| `make C=<env> clean` | Remove build/install/log directories | [8](#0-7) 

### Mission-Specific Commands [9](#0-8) 

| Command | Description |
|---------|-------------|
| `make C=<env> mission-sim` | Launch MAVROS + mission for simulation |
| `make C=<env> mavros-sim` | Launch MAVROS only (TCP simulation) |
| `make C=<env> hexa` | Launch MAVROS for Hexacopter (serial connection) |
| `make gcs` | Run web server node for ground control |

### Special Targets [10](#0-9) 

| Command | Description |
|---------|-------------|
| `make zed-shell` | Open shell in ZED camera container |
| `make launch-zed` | Launch ZED camera node |
| `make zenoh-ground` | Start Zenoh ground bridge |
| `make zenoh-air` | Start Zenoh air bridge |

### Multi-Environment Commands [11](#0-10) 

| Command | Description |
|---------|-------------|
| `make build-all` | Build all environments |
| `make up-all` | Start all environments |
| `make down-all` | Stop all environments |
| `make nuke-all` | Complete cleanup (images + workspaces) |

## Customizable Variables [12](#0-11) 

You can override these when running make commands:

```bash
make C=payload up                          # Select environment
make C=dev DOMAIN=5 up                     # Set ROS domain ID
make C=dev TCP_PORT=5763 mavros-sim       # Change MAVROS TCP port
make C=payload DRONE_IP=192.168.144.10 up # Set drone IP
```

## Network Architecture

### Zenoh Bridge Configuration

The system uses Zenoh bridges to connect ground stations with drones:

**Ground Station** (dev environment): [13](#0-12) 

**Drone** (payload/water environments): [14](#0-13) 

Configuration file: [15](#0-14) 

### ROS Domain IDs

- **Dev Environment**: Domain 3 (for local development and GCS)
- **Payload Environment**: Domain 2 (drone operations)
- **Water Environment**: Domain 2 (drone operations)

## Typical Workflows

### Development on Personal Computer

```bash
# 1. Start dev environment
make C=dev up

# 2. Enter shell
make C=dev shell

# 3. Build packages
colcon build

# 4. Source and test
source install/setup.bash
ros2 topic list
```

### Simulation with ArduPilot SITL [16](#0-15) 

```bash
# Terminal 1: Start ArduPilot SITL (outside container)
# Follow ArduPilot SITL setup

# Terminal 2: Launch MAVROS connection
make C=dev mavros-sim TCP_PORT=5762

# Terminal 3: Launch mission
make C=dev mission-sim
```

### Deploying to Drone

**On the Drone:**
```bash
# For Mission 1 (Payload)
make C=payload up
make C=payload shell
colcon build
source install/setup.bash
ros2 launch bringup payload_mission.launch.py

# For Mission 2 (Water)
make C=water up
make C=water shell
colcon build
source install/setup.bash
ros2 launch bringup water_mission.launch.py
```

**On Ground Control Station:**
```bash
# Start dev environment with Zenoh bridge
make C=dev up

# Monitor topics or run ground control
make gcs
```

### Working with ZED Camera (Payload Environment) [17](#0-16) 

```bash
# Start ZED container
make launch-zed

# Or manually:
make zed-shell
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

## Package Management

### Linking Packages to Workspaces [18](#0-17) 

Each workspace has a `pkgs.txt` file that lists which packages to include:

```bash
# Link packages (run from repo root)
./scripts/link_ws.sh workspaces/payload_ws

# Or use make target
make C=payload link
```

**Important**: The `pkgs.txt` file must end with a newline character!

### Adding a New Package

1. Add the package as a submodule:
```bash
git submodule add -b main https://github.com/your-org/package_name.git packages/package_name
```

2. Add the package name to the appropriate `workspaces/<env>_ws/pkgs.txt`

3. Run the link script:
```bash
./scripts/link_ws.sh workspaces/<env>_ws
```

## Troubleshooting

### Permission Issues

If you encounter permission errors with files:

```bash
# Fix ownership
sudo chown -R $USER:$USER ~/aeac-2026/

# Make scripts executable
chmod +x scripts/*.sh
```

### ROS Communication Issues

**No topics visible?**
- Check ROS_DOMAIN_ID: `echo $ROS_DOMAIN_ID`
- Verify RMW implementation: `echo $RMW_IMPLEMENTATION`
- Check Zenoh daemon: `ps aux | grep rmw_zenohd`

**MAVROS not connecting?**
- Verify serial device permissions
- Check FCU URL configuration
- For Jetson: Verify correct UART device (`/dev/ttyTHS0` for JetPack 5, `/dev/ttyTHS1` for JetPack 6)

### Docker Issues

```bash
# Rebuild specific environment
make C=dev down
make C=dev build
make C=dev up

# Complete cleanup
make nuke-all

# View logs
docker logs aeac-dev
```

## Hardware Connections

### Jetson Serial Connections [19](#0-18) 

- **JetPack 5**: UART1 → `/dev/ttyTHS0` (115200 baud)
- **JetPack 6**: UART1 → `/dev/ttyTHS1` (115200 baud)

Use `make hexa` for direct hexacopter connection via serial.

## Additional Resources

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Zenoh Documentation: https://zenoh.io/docs/
- MAVLink Protocol: https://mavlink.io/
- MAVROS: https://github.com/mavlink/mavros

## Notes

- All containers run with `network_mode: host` for simplified networking
- The dev environment includes GUI support via X11 forwarding
- Payload and water environments are optimized for on-board computing resources
- Always ensure proper network configuration when connecting GCS to drones
- The Zenoh bridge configuration may need adjustment based on your network topology

---

## Notes

This README is based on the current codebase structure. Key areas covered:

1. **Makefile System**: The main interface uses environment selection via `C=<env>` parameter, with `dev` as default [7](#0-6) 

2. **Three Environments**: 
   - `dev` for development/GCS with CycloneDDS and visualization tools
   - `payload` for Mission 1 with ZED camera and CycloneDDS
   - `water` for Mission 2 with Zenoh middleware and gimbal support

3. **Network Architecture**: Uses Zenoh bridges to connect ground stations (domain 3) with drones (domain 2), with configurable endpoints [20](#0-19) 

4. **Package Management**: Uses symlink-based workspace configuration via `pkgs.txt` files and the `link_ws.sh` script [18](#0-17) 

The Makefile provides comprehensive targets for building, launching, and managing each environment, with special targets for simulation, ZED camera, and multi-environment operations [21](#0-20)

### Citations

**File:** Makefile (L1-10)
```text
# ===== Select compose (dev|payload|water|relay) =====
C ?= dev
DOMAIN ?= 2
TCP_PORT ?= 5762
# ----- Paths / names -----
COMPOSE_FILE := compose/$(C).yml
CONTAINER    ?= aeac-$(C)

DRONE_IP ?= 192.168.144.12

```

**File:** Makefile (L11-16)
```text
# Workspace path RELATIVE to repo root (e.g., workspaces/dev_ws)
WS_REL := workspaces/$(C)_ws

# In-container paths
REPO_IN := /aeac
WS_IN   := $(REPO_IN)/$(WS_REL)
```

**File:** Makefile (L33-185)
```text
# ===== Pretty help =====
help: ## Show help
	@awk 'BEGIN{FS":.*##"; printf "\nTargets (use C=<dev|payload|water|relay>):\n"} /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

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

zed-shell: ## Open bash in the ZED container (with ROS sourced)
	docker compose -f compose/zed.yml exec -it zed-ros2 bash -lc '\
	  source /root/ros2_ws/install/setup.bash; \
	  exec bash -i'

zenoh-ground: ## Start zenoh ground bridge
	docker compose -f compose/zenoh-ground.yml up --build

zenoh-air: ## Start zenoh air bridge
	docker compose -f compose/zenoh-air.yml up --build

launch-zed:
	docker compose -f compose/zed.yml up -d zed-ros2
	docker compose -f compose/zed.yml exec -it zed-ros2 bash -lc '\
	  source /root/ros2_ws/install/setup.bash; \
	  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i ros_params_override_path:=config/zenith_stereo.yaml \
	'

# ===== Docker lifecycle (selected compose) =====
build: ## Build image for C
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) build


up: ## Up (detached) for C, rebuild if needed
	$(ENV_INJECT) docker compose -f $(COMPOSE_FILE) up -d --build

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

mavros-sim: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:$(TCP_PORT) fcu_protocol:=v2.0'

mission-sim: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    colcon build; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:$(TCP_PORT) fcu_protocol:=v2.0 & \
	    sleep 2; \
	    ros2 launch bringup payload_mission.launch.py \
	  '

gcs: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'cd "$$WS"; \
	    source /opt/ros/humble/setup.bash; \
	    source install/setup.bash; \
	    ros2 daemon start; \
	    ros2 run web_server_node web_server_node \
	  '

hexa: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS0:115200 fcu_protocol:=v2.0'

mavros-ofa: up
	WS=$(WS_IN) docker compose -f $(COMPOSE_FILE) exec -it $(C) \
	  bash -lc 'source /opt/ros/humble/setup.bash; \
	  ros2 daemon start; \
	  ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyAMA10:115200'

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
```

**File:** compose/dev.yml (L1-23)
```yaml
services:
  dev:
    build:
      context: ..
      dockerfile: docker/dockerfile.dev
      network: host
    container_name: aeac-${C}
    network_mode: host
    tty: true
    ipc: host
    privileged: true
    stdin_open: true
    volumes:
      - ..:/aeac
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

    working_dir: /aeac/workspaces    # set WS in .env, e.g. workspaces/small_ws
    command: ["bash","-lc","while :; do sleep 86400; done"]
    environment:  
      - ROS_DOMAIN_ID=3
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - DISPLAY=$DISPLAY

```

**File:** compose/dev.yml (L24-33)
```yaml
  zenoh-bridge-ground:  
    image: eclipse/zenoh-bridge-ros2dds:latest  
    network_mode: host  
    volumes:  
      - ../config/zenoh-ground-config.json5:/config.json5:ro  
    command: ["-c", "/config.json5"]  
    environment:  
      - ROS_DOMAIN_ID=3
      - RUST_LOG=debug
      - ROS_DISTRO=humble
```

**File:** compose/payload.yml (L1-22)
```yaml
services:
  payload:
    build:
      context: ..
      dockerfile: docker/dockerfile.payload
    container_name: aeac-${C}
    network_mode: host
    tty: true
    stdin_open: true
    volumes:
      - ../workspaces/payload_ws:/payload_ws
      - ../packages/nav_stack:/payload_ws/src/nav_stack
      - ../packages/UI:/payload_ws/src/UI
      - ../packages/tools:/payload_ws/src/tools
      - ../packages/custom_interfaces:/payload_ws/src/custom_interfaces
    working_dir: /payload_ws     # set WS in .env, e.g. workspaces/payload_ws
    command: ["bash","-lc","while :; do sleep 86400; done"]
    environment:  
      - ROS_DOMAIN_ID=2
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST  # Add this 

```

**File:** compose/payload.yml (L57-66)
```yaml
  zenoh-bridge-air:  
    image: eclipse/zenoh-bridge-ros2dds:latest  
    network_mode: host  
    volumes:  
      - ../config/zenoh-air-config.json5:/config.json5:ro  
    command: ["-c", "/config.json5"]  
    environment:  
      - ROS_DOMAIN_ID=2  
      - RUST_LOG=debug
      - ROS_DISTRO=humble
```

**File:** workspaces/payload_ws/pkgs.txt (L1-7)
```text
custom_interfaces
nav_stack
tools
bringup
vision
polar_system

```

**File:** compose/water.yml (L1-23)
```yaml
services:
  water:
    build:
      context: ..
      dockerfile: docker/dockerfile.water
    container_name: aeac-${C}
    network_mode: host
    tty: true
    stdin_open: true
    volumes:
      - ../workspaces/water_ws:/water_ws
      - ../packages/bringup:/water_ws/src/bringup
      - ../packages/nav_stack:/water_ws/src/nav_stack
      - ../packages/polar_system:/water_ws/src/polar_system
      - ../packages/tools:/water_ws/src/tools
      - ../packages/vision:/water_ws/src/vision
      - ../packages/custom_interfaces:/water_ws/src/custom_interfaces
    working_dir: /water_ws     # set WS in .env, e.g. workspaces/water_ws
    command: ["bash","-lc","while :; do sleep 86400; done"]
    environment:  
      - ROS_DOMAIN_ID=2
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp

```

**File:** workspaces/water_ws/pkgs.txt (L1-6)
```text
custom_interfaces
nav_stack
tools
bringup
vision
polar_system
```

**File:** config/zenoh-ground-config.json5 (L1-39)
```text
{  
  plugins: {  
    ros2dds: {  
      domain: 3,  
      allow: {  
        publishers: [  
          "/tf",  
          "/tf_static",  
          "/zed/zed_node/rgb/color/rect/image",  
          "/zed/zed_node/rgb/color/rect/image/compressed",  
          "/zed/zed_node/rgb/color/rect/camera_info",  
          "/chatter",  
          "/mavros.*"  
        ],  
        subscribers: [  
          "/tf",  
          "/tf_static",  
          "/zed/zed_node/rgb/color/rect/image",  
          "/zed/zed_node/rgb/color/rect/image/compressed",  
          "/zed/zed_node/rgb/color/rect/camera_info",  
          "/chatter",  
          "/mavros.*"  
        ]  
      }  
    }  
  },  
  mode: "peer",  
  connect: {  
    endpoints: [  
      "tcp/192.168.0.21:7447",  
      "tcp/192.168.30.204:7447"  
    ]  
  },  
  listen: {  
    endpoints: [  
      "tcp/0.0.0.0:7447"  
    ]  
  }  
}
```

**File:** scripts/link_ws.sh (L1-32)
```shellscript
#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:?Usage: $0 <workspace_dir>}"
PKGLIST="$WS_DIR/pkgs.txt"
SRC="$WS_DIR/src"

[ -f "$PKGLIST" ] || { echo "Missing $PKGLIST"; exit 1; }
mkdir -p "$SRC"

# repo root (works whether inside meta-repo or via symlink)
ROOT="$(git rev-parse --show-toplevel 2>/dev/null || realpath "$(cd "$(dirname "$0")/.." && pwd)")"

# remove old symlinks only
find "$SRC" -mindepth 1 -maxdepth 1 -type l -exec rm -f {} \;

while IFS= read -r PKG; do
  [[ -z "$PKG" || "$PKG" =~ ^# ]] && continue
  PKG_PATH="$ROOT/packages/$PKG"
  [ -d "$PKG_PATH" ] || { echo "ERROR: $PKG_PATH not found"; exit 2; }

  # make a RELATIVE symlink so it works both on host and in container
  REL_TARGET="$(realpath --relative-to="$SRC" "$PKG_PATH" 2>/dev/null || python3 - <<PY
import os,sys
print(os.path.relpath("$PKG_PATH","$SRC"))
PY
)"
  ln -s "$REL_TARGET" "$SRC/$PKG"
  echo "linked $PKG"
done < "$PKGLIST"

echo "Workspace $WS_DIR is set up."
```
