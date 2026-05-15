#!/usr/bin/env python3
"""
AEAC Image Transfer Node
========================
Bridges internal target pictures to the ground station (GCS) with:
  - GCS heartbeat health check before transmitting
  - Persistent image queue for offline buffering
  - ACK-based reliable delivery with configurable retries
"""

import os
import time
import threading
import queue
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Header

HEARTBEAT_TIMEOUT_SEC = 3.0   # GCS considered offline after this many seconds
ACK_TIMEOUT_SEC       = 5.0   # Wait this long for an ACK before retrying
MAX_RETRIES           = 999   # Give up after this many failed attempts per image
QUEUE_SAVE_DIR        = "/tmp/aeac_image_queue"  # Persistent disk cache directory


@dataclass(order=True)
class PendingImage:
    """An image waiting to be delivered to the GCS."""
    sort_index: float = field(init=False, repr=False)   # for priority ordering
    timestamp: float = field(default_factory=time.time)
    seq: int = 0
    msg: Optional[CompressedImage] = field(default=None, compare=False)
    retries: int = 0

    def __post_init__(self):
        self.sort_index = self.timestamp


class ImageTransferNode(Node):

    def __init__(self):
        super().__init__("image_transfer_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("heartbeat_timeout_sec", HEARTBEAT_TIMEOUT_SEC)
        self.declare_parameter("ack_timeout_sec",       ACK_TIMEOUT_SEC)
        self.declare_parameter("max_retries",           MAX_RETRIES)
        self.declare_parameter("queue_save_dir",        QUEUE_SAVE_DIR)
        self.declare_parameter("publish_rate_hz",       2.0)

        self._hb_timeout   = self.get_parameter("heartbeat_timeout_sec").value
        self._ack_timeout  = self.get_parameter("ack_timeout_sec").value
        self._max_retries  = self.get_parameter("max_retries").value
        self._save_dir     = self.get_parameter("queue_save_dir").value
        publish_rate       = self.get_parameter("publish_rate_hz").value

        os.makedirs(self._save_dir, exist_ok=True)

        # ── State ────────────────────────────────────────────────────────────
        self._gcs_online:        bool  = False
        self._last_heartbeat:    float = 0.0
        self._seq_counter:       int   = 0
        self._seq_lock                 = threading.Lock()

        # Queue of PendingImage objects (thread-safe)
        self._pending: queue.Queue[PendingImage] = queue.Queue()

        # Currently-in-flight image waiting for ACK
        self._in_transfer:         Optional[PendingImage] = None
        self._in_transfer_time:    float = 0.0
        self._in_transfer_lock             = threading.Lock()

        # ── QoS profiles ────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
        )
        best_effort_qos = QoSProfile(
            reliability  = ReliabilityPolicy.BEST_EFFORT,
            history      = HistoryPolicy.KEEP_LAST,
            depth        = 1,
            durability   = DurabilityPolicy.VOLATILE,
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self._sub_image = self.create_subscription(
            CompressedImage,
            "/aeac/internal/target_picture",
            self._cb_internal_image,
            reliable_qos,
        )

        self._sub_heartbeat = self.create_subscription(
            Bool,
            "/aeac/external/gcs/heartbeat",
            self._cb_heartbeat,
            best_effort_qos,
        )

        self._sub_ack = self.create_subscription(
            Bool,
            "/aeac/external/target_picture/ack",
            self._cb_ack,
            reliable_qos,
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub_image = self.create_publisher(
            CompressedImage,
            "/aeac/external/target_picture",
            reliable_qos,
        )

        # ── Timers ───────────────────────────────────────────────────────────
        # Heartbeat watchdog — checks whether GCS has gone silent
        self._timer_hb_watchdog = self.create_timer(
            1.0, self._cb_heartbeat_watchdog
        )

        # Dispatch loop — tries to send queued images when GCS is online
        self._timer_dispatch = self.create_timer(
            1.0 / publish_rate, self._cb_dispatch
        )

        # ── Restore any images saved from a previous run ─────────────────────
        self._restore_queue_from_disk()

        self.get_logger().info("ImageTransferNode started.")
        self.get_logger().info(
            f"  heartbeat timeout : {self._hb_timeout}s\n"
            f"  ack timeout       : {self._ack_timeout}s\n"
            f"  max retries       : {self._max_retries}\n"
            f"  disk cache        : {self._save_dir}"
        )

    def _cb_internal_image(self, msg: CompressedImage) -> None:
        """Received a new image from the internal pipeline."""
        with self._seq_lock:
            seq = self._seq_counter
            self._seq_counter += 1

        # Tag the image with a sequence number so we can match ACKs
        msg.format = f"{msg.format};seq={seq}"

        # Persist to disk so the queue survives a node restart
        self._save_image_to_disk(seq, msg)

        item = PendingImage(seq=seq, msg=msg)
        self._pending.put(item)

        self.get_logger().info(
            f"[RX] Image seq={seq} received "
            f"({len(msg.data)} bytes, format='{msg.format}'). "
            f"Queue depth: {self._pending.qsize()}"
        )

    def _cb_heartbeat(self, msg: Bool) -> None:
        """GCS heartbeat tick."""
        self._last_heartbeat = time.time()

        was_online = self._gcs_online
        self._gcs_online = msg.data   # False heartbeat = explicit offline signal

        if not was_online and self._gcs_online:
            self.get_logger().info("[HB] GCS back online — flushing queue.")
        elif was_online and not self._gcs_online:
            self.get_logger().warn("[HB] GCS sent explicit offline heartbeat.")

    def _cb_ack(self, msg: Bool) -> None:
        """
        ACK from the ground station.

        The GCS should publish on /aeac/external/target_picture/ack:
          - True  → the last image was received successfully
          - False → the last image was rejected / corrupted
        """
        with self._in_transfer_lock:
            if self._in_transfer is None:
                self.get_logger().debug("[ACK] Received ACK but nothing in-flight.")
                return

            seq = self._in_transfer.seq

            if msg.data:
                # Positive ACK — remove from disk cache
                self.get_logger().info(f"[ACK] ✓ Image seq={seq} acknowledged by GCS.")
                self._delete_image_from_disk(seq)
                self._in_transfer = None
            else:
                # Negative ACK — re-queue for retry
                self.get_logger().warn(
                    f"[ACK] ✗ Image seq={seq} rejected by GCS. Re-queuing."
                )
                self._in_transfer.retries += 1
                if self._in_transfer.retries < self._max_retries:
                    self._pending.put(self._in_transfer)
                else:
                    self.get_logger().error(
                        f"[ACK] Image seq={seq} exceeded max retries. Dropping."
                    )
                    self._delete_image_from_disk(seq)
                self._in_transfer = None

    def _cb_heartbeat_watchdog(self) -> None:
        """Mark GCS offline if we haven't heard a heartbeat recently."""
        if self._gcs_online:
            age = time.time() - self._last_heartbeat
            if age > self._hb_timeout:
                self._gcs_online = False
                self.get_logger().warn(
                    f"[HB] GCS heartbeat lost ({age:.1f}s ago). "
                    "Buffering images until connection restored."
                )

    def _cb_dispatch(self) -> None:
        """
        Main send loop.

        Rules:
          1. Don't send if GCS is offline.
          2. Don't send a new image while waiting for an ACK — unless
             the ACK timeout has expired (treat as lost, retry).
          3. Pop the next image from the queue and publish it.
        """
        if not self._gcs_online:
            return

        with self._in_transfer_lock:
            # Check ACK timeout for the current in-flight image
            if self._in_transfer is not None:
                elapsed = time.time() - self._in_transfer_time
                if elapsed < self._ack_timeout:
                    return  # Still waiting for ACK
                else:
                    self.get_logger().warn(
                        f"[TX] ACK timeout for seq={self._in_transfer.seq} "
                        f"after {elapsed:.1f}s. Retrying."
                    )
                    self._in_transfer.retries += 1
                    if self._in_transfer.retries < self._max_retries:
                        self._pending.put(self._in_transfer)
                    else:
                        self.get_logger().error(
                            f"[TX] Image seq={self._in_transfer.seq} "
                            "exceeded max retries. Dropping."
                        )
                        self._delete_image_from_disk(self._in_transfer.seq)
                    self._in_transfer = None

            # Nothing in-flight — send next queued image
            try:
                item: PendingImage = self._pending.get_nowait()
            except queue.Empty:
                return

            self._pub_image.publish(item.msg)
            self._in_transfer      = item
            self._in_transfer_time = time.time()

            self.get_logger().info(
                f"[TX] Published image seq={item.seq} "
                f"(attempt {item.retries + 1}/{self._max_retries}). "
                f"Waiting for ACK on /aeac/external/target_picture/ack …"
            )

    def _image_path(self, seq: int) -> str:
        return os.path.join(self._save_dir, f"img_{seq:08d}.bin")

    def _save_image_to_disk(self, seq: int, msg: CompressedImage) -> None:
        """Serialize a CompressedImage to a simple binary file."""
        try:
            path = self._image_path(seq)
            # Format: [4-byte format-len][format bytes][4-byte data-len][data bytes]
            fmt_bytes  = msg.format.encode("utf-8")
            data_bytes = bytes(msg.data)
            with open(path, "wb") as f:
                f.write(len(fmt_bytes).to_bytes(4, "little"))
                f.write(fmt_bytes)
                f.write(len(data_bytes).to_bytes(4, "little"))
                f.write(data_bytes)
            self.get_logger().debug(f"[DISK] Saved seq={seq} → {path}")
        except OSError as e:
            self.get_logger().error(f"[DISK] Failed to save seq={seq}: {e}")

    def _delete_image_from_disk(self, seq: int) -> None:
        path = self._image_path(seq)
        try:
            os.remove(path)
            self.get_logger().debug(f"[DISK] Deleted seq={seq}")
        except FileNotFoundError:
            pass
        except OSError as e:
            self.get_logger().error(f"[DISK] Failed to delete seq={seq}: {e}")

    def _restore_queue_from_disk(self) -> None:
        """Load any images left on disk from a previous run into the queue."""
        files = sorted(
            f for f in os.listdir(self._save_dir) if f.endswith(".bin")
        )
        if not files:
            return

        self.get_logger().info(
            f"[DISK] Restoring {len(files)} image(s) from {self._save_dir} …"
        )
        for fname in files:
            path = os.path.join(self._save_dir, fname)
            try:
                with open(path, "rb") as f:
                    fmt_len  = int.from_bytes(f.read(4), "little")
                    fmt      = f.read(fmt_len).decode("utf-8")
                    data_len = int.from_bytes(f.read(4), "little")
                    data     = f.read(data_len)

                # Extract sequence number from format string
                seq = 0
                for part in fmt.split(";"):
                    if part.startswith("seq="):
                        seq = int(part.split("=")[1])
                        break

                msg = CompressedImage()
                msg.format = fmt
                msg.data   = list(data)
                msg.header = Header()

                item = PendingImage(seq=seq, msg=msg)
                self._pending.put(item)

                # Keep seq counter above restored values
                with self._seq_lock:
                    if seq >= self._seq_counter:
                        self._seq_counter = seq + 1

                self.get_logger().info(f"[DISK] Restored seq={seq} from {fname}")
            except Exception as e:
                self.get_logger().error(f"[DISK] Failed to restore {fname}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageTransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()