#!/usr/bin/env python3
import json
import time
import sys
from pathlib import Path


from zenmav.core import Zenmav

def load_mission(path: str) -> dict:
    p = Path(path).expanduser().resolve()
    if not p.exists():
        raise FileNotFoundError(f"Fichier mission introuvable: {p}")
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)

    # Validations minimales
    if "steps" not in data or not isinstance(data["steps"], list):
        raise ValueError("Le fichier mission doit contenir une liste 'steps'.")
    for i, step in enumerate(data["steps"]):
        if "target" not in step or not (isinstance(step["target"], list) and len(step["target"]) == 3):
            raise ValueError(f"Step {i}: 'target' doit être une liste de 3 nombres [N, E, D].")
        if "sleep" in step and not isinstance(step["sleep"], (int, float)):
            raise ValueError(f"Step {i}: 'sleep' doit être un nombre (secondes).")

    return data

def main():
    DEFAULT_TRAJ = Path(__file__).resolve().parents[1] / "trajectoirePhase1.json"
    mission_path = sys.argv[1] if len(sys.argv) > 1 else str(DEFAULT_TRAJ)

    mission = load_mission(mission_path)

    ip = mission.get("ip", "tcp:127.0.0.1:5763")
    gps_thresh = mission.get("gps_thresh", 3)
    takeoff_alt = mission.get("takeoff_alt", 10)
    end_mode = mission.get("end_mode", "RTL")


    drone = Zenmav(ip='tcp:host.docker.internal:5763', gps_thresh=3)

    drone.set_mode("GUIDED")
    drone.arm()
    drone.takeoff(altitude=takeoff_alt)

    for step in mission["steps"]:
        n, e, d = step["target"]
        drone.local_target((float(n), float(e), float(d)))
        time.sleep(float(step.get("sleep", 0)))

    drone.RTL()


if __name__ == "__main__":
    main()
