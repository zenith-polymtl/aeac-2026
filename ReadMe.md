chmod +x scripts/*.sh

echo "WS=workspaces/ws_heavy_task_2" > .env   # or small_ws, ws_heavy_task_1



docker exec -it aeac-dev bash


If not able to create or delete files in the repo packages:

sudo chown -R $USER:$USER  ~/aeac-2026/


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

## pkgs.txt file must have a newline at the end to work!!!!! Else will not link

Pour créer un package :
initialiser un repo (terminal ou git web)
ajouter aux sous modules :
git submodule add -b main https://github.com/Astro-Coco/<repo_name>.git packages/<repo_name>
git add .gitmodules packages/bringup
git commit -m "register bringup as proper submodule"
git push

ajouter le package au pkgs.txt *attention de bien mettre le dernier new line

./scripts/link_ws.sh workspaces/


mavlink-routerd -t 5764 -p 127.0.0.1:5762

mavlink-routerd -t 5764 -e 127.0.0.1:5762
mavlink-routerd -t 5764  127.0.0.1:5762