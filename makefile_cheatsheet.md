# Makefile Cheatsheet

Quick reference for the main `Makefile` targets in this repository.

---

## Variables (override at runtime)

- `C` — compose profile/container selector (`dev`, `payload`, `water`, `relay`)
- `TCP_PORT` — MAVROS TCP port (default `5762`)
- `DOMAIN` — ROS domain ID (default `2`)
- `CONTAINER` — container name override (defaults to `aeac-$(C)`)
- `WS_REL` / `WS_IN` — workspace path helpers used in recipes

Examples:

- `make up C=payload`
- `make shell C=water`
- `make mavros-sim C=dev TCP_PORT=5762`

---

## Daily basics

- `make help` — list documented targets
- `make print-vars` — show resolved variable values
- `make build C=<profile>` — build selected compose image
- `make up C=<profile>` — start selected compose stack (detached, with build)
- `make down C=<profile>` — stop selected compose stack
- `make shell C=<profile>` — open shell in running container with ROS sourced
- `make launch C=<profile>` — build workspace and open interactive ROS shell
- `make connect C=<profile>` — build workspace and open a bash shell
- `make clean C=<profile>` — remove build/install/log artifacts

---

## Relay / ZED / Zenoh helpers

- `make relay` / `make relay-down`
- `make zed-shell`
- `make zed-launch`
- `make zed-launch-mini`
- `make zenoh-ground`
- `make zenoh-air`
- `make zenoh-ground-build`
- `make zenoh-air-build`

---

## Payload / Water stacks

- `make payload`
- `make water-stack-build`
- `make water-stack`
- `make gcs-payload C=payload`
- `make gcs-water C=water`
- `make payload-mission-sim`

---

## MAVROS targets

- `make mavros-sim C=<profile>`
- `make mavros-gazebo C=<profile>`
- `make mavros-jetson`
- `make mavros-ofa C=<profile>`

---

## Systemd service shortcuts

### MAVROS service

- `make mavros-restart`
- `make mavros-status`
- `make mavros-logs`

### ZED service

- `make zed-restart`
- `make zed-status`
- `make zed-logs`

### Zenoh service

- `make zenoh-restart`
- `make zenoh-status`
- `make zenoh-logs`

---

## Visualization and tools

- `make rviz C=<profile>`
- `make foxglove C=<profile>`

---

## Cleanup / global operations

- `make build-all`
- `make up-all`
- `make down-all`
- `make delete-images`
- `make nuke-images` — alias to `delete-images`
- `make nuke-all`

---

## Common flows

### Bring up payload stack + web server

1. `make up C=payload`
2. `make gcs-payload C=payload`

### Water stack with ZED launch

1. `make water-stack-build`
2. `make gcs-water C=water`

### Diagnose MAVROS systemd service

1. `make mavros-status`
2. `make mavros-logs`
3. `make mavros-restart`

---

## Notes

- Targets that run `docker compose ... exec -it` require an interactive terminal.
- Some targets are specialized and hard-code compose files (`compose/mavros.yml`, `compose/zenoh-ground.yml`, etc.).
- Use `make print-vars C=<profile>` if behavior looks wrong; it helps confirm paths and compose selection.
