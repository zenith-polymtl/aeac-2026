# AEAC-2026 – Dev Environment (Essentials)

ROS 2 (Humble) + Docker setup with per-mission workspaces, a MAVLink router, and Zenoh transport.

## Prereqs

* Docker 
* Git

## Clone

```bash
git clone <repo> aeac-2026
cd aeac-2026
./scripts/ensure_submodules.sh   
```

## Quick start (pick one mission: `dev` | `payload` | `recon` | `water`)

```bash
make C=dev up        # build + start container (detached)
make C=dev shell     #Open new shell, note that dev is the default value between, so dev is not necessary for non-drone computers
# inside:
colcon build
source install/setup.bash
```

## Repo layout (short)

```
compose/     # dev.yml, payload.yml, recon.yml, water.yml, relay.yml
docker/      # dockerfile.dev (ros+deps), dockerfile.relay (mavlink-router)
packages/    # bringup, custom_interfaces, nav_stack, tools, UI, vision, state*
workspaces/  # <mission>_ws
relay/       # sim.conf (example)
zenoh/       # config.json5 (optional)
scripts/     # ensure_submodules.sh, link_ws.sh
Makefile
```

## Make targets (minimal)

```bash
make C=<mission> up           # build+up selected compose
make C=<mission> down         # stop/remove selected compose
make C=<mission> link         # symlink packages into <mission>_ws/src
make C=<mission> connect      # shell with ROS sourced (and zenoh override)
make C=<mission> launch       # start rmw_zenohd background, then shell
make C=<mission> mavros-sim   # rmw_zenohd + launch MAVROS (TCP)
make relay                    # start mavlink-router compose
make relay-down               # stop mavlink-router
make clean                    # wipe build/install/log (host+container)
```

**Defaults:** `C=dev`, `DOMAIN=2`, `TCP_PORT=5762`, `DRONE_IP=192.168.0.13`.

## Common workflows



### On drone computer
On one computer on same network (drone), of ip 192.168.XX.XXX:
```bash
make launch C=<mission_name (ex : recon)>
```

### On the GCS computer:

```bash
make connect DRONE_IP=<192.168.XX.XXX>
```
This command will set all variables and start a zenoh router. Make sure to change <> to actual IP.



### For ardupilot sitl simulation

```bash
make mavros-sim
```

**MAVLink router**

```bash
make relay
# then:
docker compose -f compose/relay.yml exec -it mavlink-router bash
mavlink-routerd -c /relay/sim.conf
```
Honnêtement quite buggy as of yet, à voir comment bien utiliser

## Zenoh (rmw_zenoh_cpp)

`make connect` exports:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<DRONE_IP>:7447"]'
```

*(optional)* use `zenoh/config.json5` for file-based config.
Run daemon if needed:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Packages (roles)

* `bringup`: launch files
* `custom_interfaces`: msg/srv
* `nav_stack`: autonomy/nav
* `tools`: utils/logging
* `UI`: operator HMI
* `vision`: detection/localization
* `state*`: mission state machine(s)

## Troubleshooting (short)

* **No `ros2` talking?** Check `ps aux | grep rmw_zenohd` and `echo "$ZENOH_CONFIG_OVERRIDE"`  and `echo "$ROS_DOMAIN_ID"`.
* **MAVROS no link?** Verify FCU URL (TCP 5762) or serial path/baud.


## Notes pas organisées

chmod +x scripts/*.sh

echo "WS=workspaces/ws_heavy_task_2" > .env   # or small_ws, ws_heavy_task_1



docker exec -it aeac-dev bash


If not able to create or delete files in the repo packages:

sudo chown -R $USER:$USER  ~/aeac-2026/


To link packages to a ws :

./scripts/link_ws.sh workspaces/ws_heavy_task_2

Update submodules so everythin is up to date :

 ./scripts/ensure_submodules.sh


build a specific docker :

* Change crashproof.yml for the docker compose you want
docker compose -f compose/crashproof.yml build

Possible to add a tag after build to specify the service needed (check compose for services)


To start the docker:
(-d tag for detached mode)
docker compose -f compose/crashproof.yml up 

### ***Si ```colcon build``` fail (pendant le quick start):***

* si c'est qqch comme ça: " PermissionError: [Errno 13] Permission denied: 'log' "
* Le problème peut être que tu es dans root --> root@tonNom:~/aeac-2026#
* Assure‑toi que le dossier du projet appartient à ton user normal (pas à root):
  ```bash
  cd ~/aeac-2026
  ls -ld .
  ls -ld workspaces
  ```
  Si tu vois ```root root```, corrige:
  ```bash
  sudo chown -R $USER:$USER  ~/aeac-2026/
  ```
  Si ça marche toujours pas continue avec les étapes.
* Si tu n’as que root et pas d’autre compte “normal”, tu peux en créer un
  
   * Dans WSL (en root), fait la commande:
     ```bash
     adduser tonNom
     ```
   * Il va te demander un mot de passe, quelques infos (que tu peux laisser vides en appuyant sur Entrée)
* Ensuite, tu peux tester:
  ```bash
  su - tonNom
  whoami #vérifie qui tu es
  pwd
  ```
* tu devrais voir qqch genre:
  ```bash
  tonNom@qqch:~$
  ```
* C’est ce compte-là que tu utiliseras pour:
   * cloner le repo,
   * lancer Docker,
   * travailler dans ~/aeac-2026 (pas dans /root/...)
* Définir ce user comme “par défaut” dans WSL
* Pour ne plus arriver en root à chaque ouverture de WSL, tu peux dire à WSL:
   * ouvre une session avec l’utilisateur tonNom:
   * Dans PowerShell (hors WSL), tape:
       ```powershell
       wsl -l -v
       ```
   * Ça te donne le nom de ta distro (par ex. Ubuntu, Ubuntu-22.04, etc.).
       ```powershell
       wsl -d <NomDeTaDistro> -u tonNom
       ```
   * Si ça marche, tu as un shell WSL en tant que tonNom. Pour le mettre comme user par défaut, dans WSL, en root:
       ```bash
       nano /etc/wsl.conf
       ```
       * ajoute (ou modifie) ça dans le fichier:
       ```text
       [user]
       default=tonNom
       ```
       repart WSL (dans Powershell) :
       ```text
       wsl --shutdown
       ```
       puis
       ```text
       wsl -d Ubuntu-22.04
       ```
   * à chaque fois que tu vas ouvrir WSL maintenant tu devrais voir:
       ```bash
       tonNom@tonNom #ou tonNom@qqch
       ```
### *fin des étapes du ```colcon build``` fail^*

## pkgs.txt file must have a newline at the end to work!!!!! Else will not link

Pour créer un package :
# Wherever you want the repo folder to live
mkdir my_new_repo && cd my_new_repo
git init -b main
echo "# my_new_repo" > README.md
printf ".venv/\nbuild/\n*.log\n" > .gitignore

git add .
git commit -m "chore: init repo"


ajouter aux sous modules :
git submodule add -b main https://github.com/Astro-Coco/<repo_name>.git packages/<repo_name>
git add .gitmodules packages/bringup
git commit -m "register bringup as proper submodule"
git push
git submodule update --init --recursive

ajouter le package au pkgs.txt *attention de bien mettre le dernier new line

./scripts/link_ws.sh workspaces/

echo "$RMW_IMPLEMENTATION"

export ZENOH_ROUTER_CONFIG_URI=zenoh/config.json5
export ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/DRONE_IP:7447"]'
export ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/192.168.0.13:7447"]'

mavlink-routerd -t 5764 -e 127.0.0.1:14554

mavlink-routerd -t 5764 -e 127.0.0.1:5762
mavlink-routerd -t 5764  127.0.0.1:5762

ros2 run mavros mavros_node --ros-args -p fcu_url:="tcp://127.0.0.1:5762" &

Internal mavlink connection setup :

JetPack 5: UART1 → /dev/ttyTHS0

JetPack 6: UART1 → /dev/ttyTHS1




