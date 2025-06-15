import detect
import ros_library
import rospy
import os
import pickle
from geometry_msgs.msg import Point

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
    with open("coords.pickle", "wb") as f:
        pickle.dump(abs_coords, f)

ros_library.simpleNavigation([Point(0, 0, 10)], image=False)
ros_library.MAX_DIST = 1.0

ros_library.readPlates(abs_coords)

# Plate reading complete
with open("complete.txt", "w") as f:
    f.write("DONE")