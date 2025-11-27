import os
import time
import json
from picamera2 import Picamera2
import numpy as np

SHARE_DIR = "/home/pi/camshare"
FRAME_PATH = os.path.join(SHARE_DIR, "frame.npy")
FRAME_TMP_PATH = os.path.join(SHARE_DIR, "frame_tmp.npy")
META_PATH = os.path.join(SHARE_DIR, "meta.json")

# initialize camera
picam = Picamera2()
config = picam.create_video_configuration(main={"size":(640,480), "format":"BGR888"})
picam.configure(config)
picam.start()

width, height = picam.camera_configuration()['main']['size']

with open(META_PATH, 'w') as f:
    json.dump({'width':width, 'height':height}, f)

print(f"Streaming frames into {FRAME_PATH} ({width}x{heigh})")

while True:
    frame = picam.capture_array()

    np.save(FRAME_TMP_PATH, frame)
    os.replace(FRAME_TMP_PATH, FRAME_PATH)

    time.sleep(0.5)
