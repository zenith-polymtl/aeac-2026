#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

# Google API imports
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from custom_interfaces.msg import TargetImage
import mimetypes

from workspaces.ground_station_ws.build.custom_interfaces.rosidl_generator_py.custom_interfaces import msg

# Scopes required for uploading files
SCOPES = ["https://www.googleapis.com/auth/drive.file"]
IMAGE_EXTENSION = ".jpg"


class WaterImageUploader(Node):
    def __init__(self):
        super().__init__("water_image_uploader")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("credentials_path",
                               os.path.expanduser("~/credentials.json"))
        self.declare_parameter("token_path",
                               os.path.expanduser("~/token.json"))
        self.declare_parameter("drive_folder_id", "root")

        self.credentials_path = self.get_parameter(
            "credentials_path").get_parameter_value().string_value
        self.token_path = self.get_parameter(
            "token_path").get_parameter_value().string_value
        self.drive_folder_id = self.get_parameter(
            "drive_folder_id").get_parameter_value().string_value

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            TargetImage,
            "/aeac/internal/mission/water_image",
            self._image_callback,
            10,
        )

        # ── Google Drive service (lazy init on first upload) ──────────────────
        self._drive_service = None

        self.get_logger().info("WaterImageUploader ready.")
        self.get_logger().info(
            f"  credentials : {self.credentials_path}")
        self.get_logger().info(
            f"  token       : {self.token_path}")
        self.get_logger().info(
            f"  folder id   : {self.drive_folder_id}")

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _image_callback(self, msg: TargetImage) -> None:
        image_path = msg.image_path
        target_num = msg.target_number
        self._upload(image_path, target_num)

    def _upload(self, image_path: str, target_num: int) -> None:
        """Validate the file and upload it to Google Drive."""
        if not os.path.isfile(image_path):
            self.get_logger().error(
                f"File not found: {image_path}")
            return

        # Build the destination filename while keeping the original extension
        drive_name = f"Task_2_zenith_target_{target_num}{IMAGE_EXTENSION}"

        self.get_logger().info(
            f"Uploading '{image_path}' → '{drive_name}' …")

        try:
            service = self._get_drive_service()
            mime_type, _ = mimetypes.guess_type(image_path)
            if mime_type is None:
                mime_type = "application/octet-stream"

            file_metadata = {
                "name": drive_name,
                "parents": [self.drive_folder_id],
            }
            media = MediaFileUpload(image_path, mimetype=mime_type,
                                    resumable=True)

            uploaded = (
                service.files()
                .create(body=file_metadata, media_body=media, fields="id,name")
                .execute()
            )

            self.get_logger().info(
                f"Upload complete. Drive file: '{uploaded['name']}' "
                f"(id={uploaded['id']})"
            )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Upload failed: {exc}")

    # ── Google Drive authentication ───────────────────────────────────────────

    def _get_drive_service(self):
        """Return (and cache) an authenticated Drive v3 service object."""
        if self._drive_service is not None:
            return self._drive_service

        creds = None

        # Load cached token if it exists
        if os.path.exists(self.token_path):
            creds = Credentials.from_authorized_user_file(
                self.token_path, SCOPES)

        # Refresh or run the OAuth flow if needed
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                if not os.path.exists(self.credentials_path):
                    raise FileNotFoundError(
                        f"credentials.json not found at: {self.credentials_path}\n"
                        "Download it from Google Cloud Console → APIs & Services "
                        "→ Credentials."
                    )
                flow = InstalledAppFlow.from_client_secrets_file(
                    self.credentials_path, SCOPES
                )
                creds = flow.run_local_server(port=0)

            # Save the token for future runs
            with open(self.token_path, "w") as token_file:
                token_file.write(creds.to_json())

        self._drive_service = build("drive", "v3", credentials=creds)
        return self._drive_service


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WaterImageUploader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()