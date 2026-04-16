import os
import datetime

import firebase_admin
from firebase_admin import credentials, storage as fb_storage, firestore


class MapUploader:
    def __init__(self):
        self.firebase_ok = False
        self._init_firebase()

    def _init_firebase(self):
        try:
            cred = credentials.Certificate(
                "/home/retail-assistant/SLAM/firebase/firebase_key.json"  # 🔧 CHANGE IF NEEDED
            )

            firebase_admin.initialize_app(cred, {
                'storageBucket': 'sick-lidar-3.firebasestorage.app'  # 🔧 VERIFY
            })

            self.firebase_ok = True
            print("Firebase initialized.")

        except Exception as e:
            print(f"Firebase init failed: {e}")

    # 🔥 THIS IS THE ONLY FUNCTION YOU CALL
    def upload_map(self, path: str):
        """
        Accepts:
        - ".../map.yaml"
        OR
        - ".../map"
        """

        if not self.firebase_ok:
            print("Firebase not initialized.")
            return

        try:
            # ---- Normalize input ----
            if path.endswith(".yaml"):
                yaml_path = path
                base = path[:-5]  # remove .yaml
            else:
                base = path
                yaml_path = base + ".yaml"

            pgm_path = base + ".pgm"

            # ---- Validate files ----
            if not os.path.exists(yaml_path):
                print(f"YAML not found: {yaml_path}")
                return

            if not os.path.exists(pgm_path):
                print(f"PGM not found: {pgm_path}")
                return

            base_name = os.path.basename(base)

            # ---- Timestamp folder ----
            now = datetime.datetime.utcnow()
            ts = now.strftime("%Y%m%d_%H%M%S_%f")
            folder = f"slam_maps/{ts}_{base_name}"

            bucket = fb_storage.bucket()
            db = firestore.client()

            # ---- Upload YAML ----
            yaml_blob = bucket.blob(f"{folder}/{base_name}.yaml")
            yaml_blob.upload_from_filename(yaml_path, content_type="text/yaml")
            yaml_blob.make_public()

            # ---- Upload PGM ----
            pgm_blob = bucket.blob(f"{folder}/{base_name}.pgm")
            pgm_blob.upload_from_filename(
                pgm_path,
                content_type="image/x-portable-graymap"
            )
            pgm_blob.make_public()

            # ---- Save metadata ----
            db.collection("slam_maps").add({
                "timestamp": now.isoformat() + "Z",
                "map_name": base_name,
                "yaml_url": yaml_blob.public_url,
                "pgm_url": pgm_blob.public_url,
                "folder": folder
            })

            print("✅ Map uploaded successfully")
            print("YAML:", yaml_blob.public_url)
            print("PGM :", pgm_blob.public_url)

        except Exception as e:
            print(f"❌ Upload failed: {e}")
def main():
    uploader = MapUploader()
    uploader.upload_map("/home/retail-assistant/SLAM/src/retail_assistant_bringup/Slam_maps/auto_map")  # 🔧 CHANGE THIS TO YOUR MAP PAT
if __name__ == "__main__":
    main()