#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def clear_costmap(event):
    try:
        clear_srv()
        rospy.loginfo("Cleared costmaps")
    except rospy.ServiceException:
        rospy.logwarn("Failed to call /firefly/move_base/clear_costmaps")

if __name__ == "__main__":
    rospy.init_node("periodic_costmap_clearer")

    rospy.wait_for_service("/firefly/move_base/clear_costmaps")
    clear_srv = rospy.ServiceProxy("/firefly/move_base/clear_costmaps", Empty)

    clear_interval = rospy.get_param("~interval", 5.0)  # default: every 5 seconds
    rospy.Timer(rospy.Duration(clear_interval), clear_costmap)

    rospy.loginfo("Periodic costmap clearer running (interval: %.1f sec)", clear_interval)
    rospy.spin()