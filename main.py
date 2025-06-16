import detect
import ros_library
import rospy
import os
import json
from geometry_msgs.msg import Point
import numpy as np

def convert_numpy(obj):
    if isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    elif isinstance(obj, np.generic):
        return obj.item()
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj

# Remove completion status
if os.path.exists("complete.txt"):
    os.remove("complete.txt")

rospy.init_node("navigation_node", anonymous=True)

ros_library.resetOutput()
ros_library.resetPlates()
ros_library.resetPhotos()
ros_library.MAX_DIST = 10.0
ros_library.simpleNavigation([Point(0, 0, 80)], image=True)

detect.init()
for f in os.listdir("images"):
    detect.draw_detections_on_image(f, f"output/det_{f}")
    abs_coords = detect.inference_with_absolute_coordinates(f, remove_repetations=True)
    abs_coords = convert_numpy(abs_coords)  # Convert before saving
    with open("coords.json", "w") as f:
        json.dump(abs_coords, f, indent=2)

ros_library.simpleNavigation([Point(0, 0, 10)], image=False)
ros_library.MAX_DIST = 3.0

ros_library.readPlates(abs_coords)

# Plate reading complete
with open("complete.txt", "w") as f:
    f.write("DONE")