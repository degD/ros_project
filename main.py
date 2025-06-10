import detect
import ros_library
import rospy
import os
from geometry_msgs.msg import Point

rospy.init_node("navigation_node", anonymous=True)

ros_library.resetPlates()
ros_library.resetPhotos()
ros_library.MAX_DIST = 10.0
ros_library.simpleNavigation([Point(0, 0, 100)], image=True)
ros_library.simpleNavigation([Point(0, 0, 10)], image=False)
ros_library.MAX_DIST = 1.0
detect.init()

for f in os.listdir("images"):
    detect.draw_detections_on_image(f, f"output/det_{f}")
    abs_coords = detect.inference_with_absolute_coordinates(f, remove_repetations=True)

ros_library.readPlates(abs_coords)