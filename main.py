import detect
import ros_library
import rospy
import os
import json
from geometry_msgs.msg import Point, Pose
import numpy as np
import sys
import shapely.geometry
import navigation

    
# Navigation data
data = json.loads(sys.argv[1])
# Coordinate system of tile
#        +y
#        |
# -x <-- 0 -- +x
#        |
#       -y

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

rospy.init_node("navigation_node", anonymous=True)
rospy.Subscriber('/firefly/ground_truth/pose', Pose, ros_library.pose_callback)
navigation.init()

# Publish drone location
navigation.send_position_to_flask()

ros_library.resetOutput()
ros_library.resetPlates()
ros_library.resetPhotos()
ros_library.MAX_DIST = 10.0

# Now visit and take photos of all the tiles
points = [Point(tile["x"], tile["y"], data["height"]) for tile in data["tiles"]]
for pt in points:
    navigation.navigate(pt, avoid=True, image=False, communicate=True)
    ros_library.stepRotateDrone(0.0, relative=False)
    navigation.takePhoto()

detect.init()
all_abs_coords = []
for f in os.listdir("images"):
    detect.draw_detections_on_image(f, f"output/det_{f}")
    abs_coords = detect.inference_with_absolute_coordinates(f, remove_repetations=True)
    abs_coords = convert_numpy(abs_coords)  # Convert before saving
    all_abs_coords.extend(abs_coords)
    
# Invert x_global for each car entry
for car in all_abs_coords:
    if isinstance(car, dict) and "x_global" in car:
        car["x_global"] = -car["x_global"]
        car["angle"] = -car["angle"]

# Filter out detections outside the area polygon
if "area_edges" in data and data["area_edges"]:
    polygon = shapely.geometry.Polygon([(pt["x"], pt["y"]) for pt in data["area_edges"]])
    filtered_coords = []
    for car in all_abs_coords:
        # Assume car has 'x' and 'y' keys for its center
        if "x" in car and "y" in car:
            if polygon.contains(shapely.geometry.Point(car["x"], car["y"])):
                filtered_coords.append(car)
        else:
            filtered_coords.append(car)  # Keep if no position info
    all_abs_coords = filtered_coords

with open("coords.json", "w") as f:
    json.dump(all_abs_coords, f, indent=2)

ros_library.MAX_DIST = 3.0
ros_library.readPlates(all_abs_coords)

# Plate reading complete
with open("complete.txt", "w") as f:
    f.write("DONE")

# Return to home after reading all the plates, and wait until the next period
# h = 10 for sim, make it land and stop later
# navigation.navigate(Point(data["home"]["x"], data["home"]["y"], 10), avoid=True, image=False, communicate=True)