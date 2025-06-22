# import detect
# import plate
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, PointStamped, Quaternion
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
import math
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf.transformations as tft
import requests
import numpy as np
import tf2_ros
import tf2_geometry_msgs
# import mmrotate


EPS         = 0.5
STEP_DIST   = 1.0

INFERENCE_THRESHOLD = 0.4
INFERENCE_Z = 20
VEHICLE_EPS = 5
URL_POS = "http://localhost:5000/update_drone_position"  
PLATE = True   # as a result makes plate texts DISABLED and skips recognizing plates.
BOX_SIZE = 2.0   # change if you need a larger safety margin
ROTATION_THRESHOLD = 0.2
PUB_RATE_HZ        = 15  
TIMEOUT_SEC        = 10  
VOXEL_SIZE  = 0.2

FALLBACK_ANGLES = [
    {'left': 30},
    {'right': 30},
    {'left': 45, 'up': 15},
    {'right': 45, 'up': 15},
    {'up': 45},
    {'up': 90},
]


latest_pose = {"pose": None}

def pose_callback(msg):
    latest_pose["pose"] = msg

def get_current_pose():
    """
    Helper function to get the current pose of the UAV.
    Returns:
        Pose: The current pose of the UAV.
    """
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_pose["pose"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    if latest_pose["pose"] is None:
        rospy.logwarn("Could not get current pose.")
    return latest_pose["pose"]


def navigate(target: Point, avoid=False, take_image=False):
    
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)

    while True:
        
        current_pose = get_current_pose()
        if current_pose is None:
            rospy.logerr("Current pose unavailable. Aborting navigation.")
            return
        current_position = current_pose.position
        current_quaternion = (
            current_pose.orientation.x, 
            current_pose.orientation.y,
            current_pose.orientation.z, 
            current_pose.orientation.w
        )
        
        dx = target.x - current_position.x
        dy = target.y - current_position.y
        dz = target.z - current_position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < EPS:
            rospy.loginfo("Arrived at target.")
            return     

        step_size = min(STEP_DIST, distance)
        ratio = step_size / distance
        step_point = Point(
            current_position.x + dx * ratio,
            current_position.y + dy * ratio,
            current_position.z + dz * ratio
        )
        
        dx = step_point.x - current_position.x
        dy = step_point.y - current_position.y
        if abs(dx) > EPS or abs(dy) > EPS:            
            angle_to_target = math.atan2(dy, dx)
            roll, pitch, yaw = euler_from_quaternion(current_quaternion)
            yaw_diff = angle_to_target - yaw
            yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
            max_yaw_change = math.radians(45)
            if abs(yaw_diff) > max_yaw_change:
                angle_to_target = yaw + (max_yaw_change if yaw_diff > 0 else -max_yaw_change)
            yaw = angle_to_target
            q = quaternion_from_euler(roll, pitch, yaw)
            step_orientation = Quaternion(
                x=q[0],
                y=q[1],
                z=q[2],
                w=q[3]
            )
        else:
            step_orientation = current_pose.orientation
        
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position = step_point
        pose_stamped.pose.orientation = step_orientation
        pub.publish(pose_stamped)
        

rospy.init_node("navigation_node", anonymous=True)
rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
navigate(Point(0, -5, 2))