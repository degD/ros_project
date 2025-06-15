# Continue after succesfull detection

import ros_library
import rospy
import pickle
from geometry_msgs.msg import Point

rospy.init_node("navigation_node", anonymous=True)

with open("coords.pickle", "rb") as f:
    abs_coords = pickle.load(f)

ros_library.readPlates(abs_coords)