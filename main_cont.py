# Continue after succesfull detection

import ros_library
import rospy
import json
from geometry_msgs.msg import Point

rospy.init_node("navigation_node", anonymous=True)

with open("coords.json", "r") as f:
    abs_coords = json.load(f)

ros_library.MAX_DIST = 3.0
ros_library.readPlates(abs_coords)

# Plate reading complete
with open("complete.txt", "w") as f:
    f.write("DONE")