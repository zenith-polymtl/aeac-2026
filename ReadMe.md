chmod +x scripts/*.sh

echo "WS=workspaces/ws_heavy_task_2" > .env   # or small_ws, ws_heavy_task_1



docker exec -it aeac-dev bash


If not able to create or delete files in the repo packages:

sudo chown -R $USER:$USER  ~/aeac-2026/packages/nav_stack \
                           ~/aeac-2026/packages/custom_interfaces \
                           ~/aeac-2026/packages/tools \
                           ~/aeac-2026/packages/UI


To link packages to a ws :

./scripts/link_ws.sh workspaces/ws_heavy_task_2

Update submodules so everythin is up to date :

 ./scripts/ensure_submodules.sh

 .env file to precise which workspace to launch


build a specific docker :

* Change crashproof.yml for the docker compose you want
docker compose -f compose/crashproof.yml build

Possible to add a tage after build to specify the service needed (check compose for services)


To start the docker:
(-d tag for detached mode)
docker compose -f compose/crashproof.yml up 

