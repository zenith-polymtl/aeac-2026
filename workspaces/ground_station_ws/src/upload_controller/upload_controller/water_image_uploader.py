#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import json

# Google API imports
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from custom_interfaces.msg import TargetImage
import mimetypes

# Scopes required for uploading files
SCOPES = ["https://www.googleapis.com/auth/drive.file"]
IMAGE_EXTENSION = ".jpg"


class WaterImageUploader(Node):
    def __init__(self):
        super().__init__("water_image_uploader")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("credentials_path",
                               os.path.expanduser("/aeac/credentials.json"))
        self.declare_parameter("token_path",
                               os.path.expanduser("/aeac/token.json"))
        self.declare_parameter("drive_folder_id", "root")

        self.credentials_path = self.get_parameter(
            "credentials_path").get_parameter_value().string_value
        self.token_path = self.get_parameter(
            "token_path").get_parameter_value().string_value
        self.drive_folder_id = self.get_parameter(
            "drive_folder_id").get_parameter_value().string_value

        # ── Eagerly authenticate at startup ───────────────────────────────────
        try:
            self._drive_service = self._init_drive_service()
        except Exception as exc:
            self.get_logger().fatal(f"Failed to authenticate with Google Drive: {exc}")
            raise

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            TargetImage,
            "/aeac/internal/mission/target_image",
            self._image_callback,
            10,
        )

        self.get_logger().info("WaterImageUploader ready.")

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _image_callback(self, msg: TargetImage) -> None:
        image_path = msg.image_path
        target_num = msg.target_num
        self._upload(image_path, target_num)

    def _upload(self, image_path: str, target_num: int) -> None:
        """Validate the file and upload it to Google Drive."""
        if not os.path.isfile(image_path):
            self.get_logger().error(f"File not found: {image_path}")
            return

        drive_name = f"Task_2_zenith_target_{target_num}{IMAGE_EXTENSION}"
        self.get_logger().info(f"Uploading '{image_path}' → '{drive_name}' …")

        try:
            mime_type, _ = mimetypes.guess_type(image_path)
            if mime_type is None:
                mime_type = "application/octet-stream"

            file_metadata = {
                "name": drive_name,
                "parents": [self.drive_folder_id],
            }
            media = MediaFileUpload(image_path, mimetype=mime_type, resumable=True)

            uploaded = (
                self._drive_service.files()
                .create(body=file_metadata, media_body=media, fields="id,name")
                .execute()
            )

            self.get_logger().info(
                f"Upload complete. Drive file: '{uploaded['name']}' "
                f"(url=https://drive.google.com/file/d/{uploaded['id']})"
            )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Upload failed: {exc}")

    # ── Google Drive authentication ───────────────────────────────────────────

    def _init_drive_service(self):
        """Authenticate once at startup and return a Drive v3 service object."""
        self.get_logger().info("Authenticating with Google Drive API…")
        self.get_logger().info(f"  token path : {self.token_path}")

        creds = None

        if os.path.exists(self.token_path):
            self.get_logger().info("Found cached token, loading credentials…")
            creds = Credentials.from_authorized_user_file(self.token_path, SCOPES)
            self.get_logger().info(f"  valid        : {creds.valid}")
            self.get_logger().info(f"  expired      : {creds.expired}")
            self.get_logger().info(f"  has_refresh  : {creds.refresh_token is not None}")
            self.get_logger().info(f"  scopes       : {creds.scopes}")
        else:
            self.get_logger().info("No cached token found.")
    
        if not creds or not creds.valid:
            if creds and creds.refresh_token:
                self.get_logger().info("Refreshing expired credentials…")
                creds.refresh(Request())
            else:
                self.get_logger().info("Token not valid. The upload node needs to be re-authenticated.")
                self.get_logger().info("Stopping the upload node.")
                raise Exception("No valid credentials available. Please re-authenticate.")


            with open(self.token_path, "w") as token_file:
                token_data = {
                    "token": creds.token,
                    "refresh_token": creds.refresh_token,
                    "token_uri": creds.token_uri,
                    "client_id": creds.client_id,
                    "client_secret": creds.client_secret,
                    "scopes": list(creds.scopes) if creds.scopes else [],
                }
                token_file.write(json.dumps(token_data))
            self.get_logger().info(f"Token saved to {self.token_path}")

        self.get_logger().info("Authentication successful.")
        return build("drive", "v3", credentials=creds)


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