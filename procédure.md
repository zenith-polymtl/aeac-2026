# Procédure d'opération (Water / Payload)

## Préparation

1. Confirmer que la manette est allumée.
2. Démarrer l'ordinateur de **Ground Control Station (GCS)** et ouvrir le repo
3. Démarrer la Jetson (drone).

## Démarrage mission

1. Se connecter au drone en SSH :
   - `ssh zenith@jetson-hexa.local`
   - ou `ssh zenith@192.168.144.30`
2. Aller dans le dossier du projet :
   - `cd ~/aeac-2026`
3. Lancer la stack selon le cas :
   - Water : `make water`
   - Payload : `make payload`
4. (Option développement / debug) Ouvrir un shell dans le conteneur :
   - Water : `make shell C=water`
   - Payload : `make shell C=payload`
5. Sur l'ordinateur GCS, lancer :
   - Water : `make gcs-water`
   - Payload : `make gcs-payload`

---

## Procédure de debug

### 1) Vérifier l'environnement ROS côté drone

1. Ouvrir un shell dans le bon conteneur :
   - `make shell C=water`
   - ou `make shell C=payload`
2. Lister les topics :
   - `ros2 topic list`
3. Identifier ce qui manque.

### 2) Si un service système manque

Utiliser les commandes de service selon le composant concerné (`mavros`, `zed`, `zenoh`) :

- Vérifier l'état :
  - `make mavros-status`
  - `make zed-status`
  - `make zenoh-status`
- Redémarrer le service si nécessaire :
  - `make mavros-restart`
  - `make zed-restart`
  - `make zenoh-restart`
- Suivre les logs :
  - `make mavros-logs`
  - `make zed-logs`
  - `make zenoh-logs`

### 3) Si `ros2 topic list` ne fonctionne pas

Probable problème réseau profond (DDS / routage / interface).

Vérifications recommandées :

- Connectivité réseau entre GCS et drone
- Même segment réseau / IP correctes
- Services ROS et bridges (MAVROS / Zenoh / ZED) actifs
