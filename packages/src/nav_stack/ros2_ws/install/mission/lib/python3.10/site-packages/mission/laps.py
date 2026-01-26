import rclpy
from rclpy.node import Node
from pathlib import Path
import json
import time

from zenmav.core import Zenmav
from custom_interfaces.srv import GpsToLocal


class laps(Node):
    def __init__(self):
        super().__init__('coordonnee')

        # --- Paramètre ROS2 : chemin vers la mission ---
        self.declare_parameter("mission_file", "")
        mission_file_param = self.get_parameter("mission_file").get_parameter_value().string_value

        if not mission_file_param:
            raise ValueError("Le paramètre 'mission_file' doit être défini")

        self.mission_file = Path(mission_file_param).expanduser().resolve()
        if not self.mission_file.is_file():
            raise FileNotFoundError(f"Fichier mission introuvable: {self.mission_file}")

        # --- Charger la mission (sans conversion GPS ici) ---
        self.mission = self.load_mission(self.mission_file)
        self.current_step = 0

        self.pending_future = None
        self.sleep_until = None

        # -------------------------------------------------
        # CHRONO (mission + steps) + affichage en direct
        # -------------------------------------------------
        self.mission_started = False
        self.t_mission_start = None
        self.t_mission_end = None

        self.step_send_times = []     # timestamps perf_counter au moment du local_target
        self.step_indices_sent = []   # index des steps envoyés

        self.last_chrono_print = 0.0  # pour limiter l'affichage à 1 Hz

        # --- Client du service GPS → Local ---
        self.gps_client = self.create_client(GpsToLocal, 'gps_to_local')
        while not self.gps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("En attente du service gps_to_local...")

        # -------------------------------------------------
        # WARM-UP DDS du service (évite Step 1 très lent)
        # -------------------------------------------------
        try:
            first = self.mission["steps"][0]["target_gps"]
            req = GpsToLocal.Request()
            req.latitude = float(first[0])
            req.longitude = float(first[1])
            req.altitude = float(first[2])

            t0 = time.time()
            future = self.gps_client.call_async(req)

            # Ici c'est OK de bloquer: une seule fois au démarrage
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            dt = time.time() - t0

            if future.result() is None:
                self.get_logger().error("Warm-up gps_to_local a échoué (pas de réponse)")
            else:
                self.get_logger().info(f"Warm-up gps_to_local OK en {dt:.2f}s")

        except Exception as e:
            self.get_logger().warn(f"Warm-up gps_to_local ignoré (erreur: {e})")

        # --- Connexion au drone ---
        self.drone = Zenmav(
            ip=self.mission["ip"],
            gps_thresh=self.mission["gps_thresh"]
        )

        # --- Décollage ---
        self.drone.set_mode("GUIDED")
        self.drone.arm()
        self.drone.takeoff(altitude=self.mission["takeoff_alt"])

        # --- Timer de mission ---
        self.timer = self.create_timer(0.2, self.follow_mission_step)

    # -------------------------------------------------
    # Chargement du JSON (aucune conversion GPS ici)
    # -------------------------------------------------
    def load_mission(self, path: Path) -> dict:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)

        required_keys = [
            "takeoff_alt",
            "gps_thresh",
            "ip",
            "steps"
        ]

        for key in required_keys:
            if key not in data:
                raise ValueError(f"Clé manquante dans le JSON: {key}")

        if not isinstance(data["steps"], list):
            raise ValueError("'steps' doit être une liste")

        for i, step in enumerate(data["steps"]):
            if "target_gps" not in step or not (
                isinstance(step["target_gps"], list) and len(step["target_gps"]) == 3
            ):
                raise ValueError(f"Step {i}: 'target_gps' doit être [lat, lon, alt]")

            if "sleep" in step and not isinstance(step["sleep"], (int, float)):
                raise ValueError(f"Step {i}: 'sleep' doit être numérique")

        data["end_mode"] = data.get("end_mode", "RTL")
        return data

    # -------------------------------------------------
    # Exécution de la mission
    # -------------------------------------------------
    def follow_mission_step(self):
        now = self.get_clock().now()

        # -------------------------------------------------
        # Affichage du chrono en direct (1 Hz)
        # -------------------------------------------------
        if self.mission_started and self.t_mission_start is not None:
            t_now = time.perf_counter()
            if (t_now - self.last_chrono_print) >= 1.0:
                elapsed = t_now - self.t_mission_start
                self.get_logger().info(f"Chrono: {elapsed:.1f} s")
                self.last_chrono_print = t_now

        # Respecter le sleep sans bloquer
        if self.sleep_until is not None and now < self.sleep_until:
            return
        self.sleep_until = None

        # Si une requête est en cours, attendre qu'elle termine
        if self.pending_future is not None:
            if not self.pending_future.done():
                return

            res = self.pending_future.result()
            self.pending_future = None

            if res is None:
                self.get_logger().error("Échec de la conversion GPS → local")
                return

            n, e, d = res.north, res.east, res.down

            # Envoi de la consigne au drone
            self.drone.local_target((float(n), float(e), float(d)))

            sent_step_index = self.current_step - 1
            step = self.mission["steps"][sent_step_index]

            # -------------------------------------------------
            # CHRONO: start + timestamp par step (au moment du send)
            # -------------------------------------------------
            t = time.perf_counter()
            if not self.mission_started:
                self.mission_started = True
                self.t_mission_start = t
                self.last_chrono_print = t  # évite d'imprimer immédiatement 0.0 s
                self.get_logger().info("Chrono mission démarré (1er target envoyé).")

            self.step_send_times.append(t)
            self.step_indices_sent.append(sent_step_index)

            self.get_logger().info(
                f"Step {sent_step_index + 1}/{len(self.mission['steps'])} "
                f"N={n:.2f} E={e:.2f} D={d:.2f}"
            )

            sleep_time = float(step.get("sleep", 0))
            if sleep_time > 0:
                from rclpy.duration import Duration
                self.sleep_until = now + Duration(seconds=sleep_time)

            return

        # Si mission terminée
        if self.current_step >= len(self.mission["steps"]):
            self.finish_mission()
            return

        # Lancer la requête async pour le step courant
        step = self.mission["steps"][self.current_step]
        lat, lon, alt = step["target_gps"]

        req = GpsToLocal.Request()
        req.latitude = float(lat)
        req.longitude = float(lon)
        req.altitude = float(alt)

        self.pending_future = self.gps_client.call_async(req)
        self.current_step += 1

    # -------------------------------------------------
    # Fin de mission
    # -------------------------------------------------
    def finish_mission(self):
        # Arrêter le timer d'abord (évite double exécution)
        try:
            self.timer.cancel()
        except Exception:
            pass

        end_mode = self.mission.get("end_mode", "RTL")

        # -------------------------------------------------
        # CHRONO: total + deltas entre steps
        # -------------------------------------------------
        self.t_mission_end = time.perf_counter()

        if self.t_mission_start is not None:
            total = self.t_mission_end - self.t_mission_start
            self.get_logger().info(f"Temps total (1er target -> finish): {total:.3f} s")

            if len(self.step_send_times) >= 2:
                for i in range(1, len(self.step_send_times)):
                    dt = self.step_send_times[i] - self.step_send_times[i - 1]
                    prev_idx = self.step_indices_sent[i - 1] + 1
                    cur_idx = self.step_indices_sent[i] + 1
                    self.get_logger().info(f"Δt Step {prev_idx} -> Step {cur_idx}: {dt:.3f} s")
        else:
            self.get_logger().warn("Chrono: aucun target n'a été envoyé (temps total indisponible).")

        # -------------------------------------------------
        # Action de fin (RTL ou autre)
        # -------------------------------------------------
        if end_mode == "RTL":
            self.drone.RTL()
            self.get_logger().info("Mission terminée → RTL")
        else:
            self.get_logger().info(f"Mission terminée → mode {end_mode}")


def main(args=None):
    rclpy.init(args=args)
    node = laps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
