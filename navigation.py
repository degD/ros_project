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
BOX_SIZE = (1.0, 1.0, 1.0)   # (up, left, forward)
ROTATION_THRESHOLD = 0.2
PUB_RATE_HZ        = 15  
TIMEOUT_SEC        = 10  
VOXEL_SIZE  = 1.0
Z_OFFSET    = 1.0
URL_POS = "http://localhost:5000/update_drone_position"  

FALLBACK_ANGLES = [
    {'up': 90},
]


def first_open_point(target: Point,
                     angle_offsets_list: list,
                     box_size: tuple = BOX_SIZE) -> Point:
    """
    Return `target` if it is free of obstacles; otherwise iterate through
    angle offsets (relative to the drone) and return the first candidate
    point that is clear.

    Args:
        target (Point): Desired waypoint in world frame.
        angle_offsets_list (list[dict]): e.g. [{'left':30}, {'left':30, 'up':15}]
        box_size (tuple): Dimensions of the box (x, y, z) in meters.

    Returns:
        Point: The chosen open point (target itself or the first fallback).
               If none are clear, the original target is returned.
    """
    # 1) Quick check: is target already clear?
    if box_is_clear(target, box_size):
        return target

    # 2) Get current drone pose (world frame)
    pose = get_current_pose()
    if pose is None:
        rospy.logwarn("Drone pose unavailable; returning original target.")
        return target

    pos = pose.position
    ori = pose.orientation

    # 3) Distance from drone to target (used for all probes)
    dx, dy, dz = target.x - pos.x, target.y - pos.y, target.z - pos.z
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    # 4) Try each angular offset in order
    for offsets in angle_offsets_list:
        probe = point_at_angular_offset(pos, ori, distance, offsets)
        rospy.loginfo(f"Trying point: x:{probe.x:2f}, y:{probe.y:2f}, z:{probe.z:2f}")
        if box_is_clear(probe, box_size):
            return probe  # first clear candidate wins

    # 5) Nothing was clear — raise exception
    rospy.logwarn("No open fallback found.")
    return None


transformed_cloud = None
latest_point_cloud = None  # Global variable to store the latest point cloud

def point_cloud_callback(msg):
    global latest_point_cloud
    latest_point_cloud = msg

def wait_for_pointcloud_and_filter_box(target_pt: Point, box_size: tuple, timeout=2.0):
    """
    Waits for a point cloud and filters points within a box around the target point.

    Args:
        target_pt (Point): Center of the box (global coordinates).
        box_size (tuple): Dimensions of the box (x, y, z) in meters.
        timeout (float): Timeout in seconds.

    Returns:
        bool: True if the box is clear, False otherwise.
    """
    global latest_point_cloud, transformed_cloud

    start_time = rospy.Time.now().to_sec()
    transformed_cloud = None

    while rospy.Time.now().to_sec() - start_time < timeout:
        if latest_point_cloud is None:
            continue

        try:
            stamp = latest_point_cloud.header.stamp
            src_frame = latest_point_cloud.header.frame_id
            target_frame = "world"

            if not tf_buf.can_transform(src_frame, target_frame, stamp, rospy.Duration(0.2)):
                continue

            # Transform box center from world to sensor frame (NOT the cloud to world)
            stamped_target = PointStamped()
            stamped_target.header.frame_id = target_frame
            stamped_target.header.stamp = stamp
            stamped_target.point = target_pt

            target_in_sensor = tf_buf.transform(stamped_target, src_frame, rospy.Duration(0.1)).point

            # Filter directly in the sensor frame (much faster)
            half_x, half_y, half_z = box_size[0] / 2.0, box_size[1] / 2.0, box_size[2] / 2.0
            min_x, max_x = target_in_sensor.x - half_x, target_in_sensor.x + half_x
            min_y, max_y = target_in_sensor.y - half_y, target_in_sensor.y + half_y
            min_z, max_z = target_in_sensor.z - half_z, target_in_sensor.z + half_z

            filtered = []
            for x, y, z in pcl2.read_points(latest_point_cloud, field_names=("x", "y", "z"), skip_nans=True):
                if (min_x <= x <= max_x and
                    min_y <= y <= max_y and
                    min_z <= z <= max_z):
                    filtered.append((x, y, z))

            # Early return if we found something inside the box
            if filtered:
                transformed_cloud = np.array(filtered)
                return True

            transformed_cloud = np.array([])  # Empty result, box is clear
            return True

        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF error: %s", str(e))

    rospy.logwarn("Timeout: No valid pointcloud or transform after %.1f seconds", timeout)
    return False

def box_is_clear(target_pt: Point, box_size: tuple) -> bool:
    """
    Checks if the box around the target point is clear of obstacles.

    Args:
        target_pt (Point): Center of the box (global coordinates).
        box_size (tuple): Dimensions of the box (x, y, z) in meters.

    Returns:
        bool: True if the box is clear, False otherwise.
    """
    if not wait_for_pointcloud_and_filter_box(target_pt, box_size):
        return False

    return transformed_cloud.size == 0

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


def point_at_angular_offset(position: Point,
                            orientation: Quaternion,
                            distance: float,
                            angle_offsets: dict) -> Point:
    """
    Return a point lying `distance` metres from `position` in the direction
    obtained by applying the given angular offsets to the drone's forward axis.

    Args:
        position (Point):    Drone position in world frame.
        orientation (Quaternion): Drone orientation (x,y,z,w).
        distance (float):    Radius of the sphere (metres).
        angle_offsets (dict): e.g. {'left':30, 'up':15}.  Keys may be:
                              'left', 'right', 'up', 'down'.
                              Values are degrees (positive).

    Returns:
        Point: Target location in world frame.

    Notes:
        • At most one yaw key ('left' or 'right') and one pitch key
          ('up' or 'down') will be honoured; extras are ignored.
        • Drone is assumed level (roll = 0), matching your requirement.
    """
    # Basis axes in body frame ---------------------------------------------
    fwd_body   = (1.0, 0.0, 0.0)     # +X
    up_body    = (0.0, 0.0, 1.0)     # +Z
    right_body = (0.0, -1.0, 0.0)    # +Y (right is -Y)

    # World-frame rotation matrix from quaternion --------------------------
    R = tft.quaternion_matrix([orientation.x,
                               orientation.y,
                               orientation.z,
                               orientation.w])[:3, :3]

    # Helper to build axis-angle rotation matrices -------------------------
    def rot_from(key: str, deg: float):
        rad = math.radians(deg)
        if key == 'left':
            axis = R.dot(up_body)    # yaw about up (+Z)  left = +rad
            return tft.rotation_matrix(+rad, axis)[:3, :3]
        if key == 'right':
            axis = R.dot(up_body)
            return tft.rotation_matrix(-rad, axis)[:3, :3]
        if key == 'up':
            axis = R.dot(right_body) # pitch about right axis  up = +rad
            return tft.rotation_matrix(+rad, axis)[:3, :3]
        if key == 'down':
            axis = R.dot(right_body)
            return tft.rotation_matrix(-rad, axis)[:3, :3]
        raise ValueError(f"Invalid key '{key}'")

    # Build composite rotation --------------------------------------------
    R_total = tft.identity_matrix()[:3, :3]

    # Apply yaw first, then pitch (fixed, intuitive order)
    for key in ('left', 'right'):
        if key in angle_offsets:
            R_total = rot_from(key, angle_offsets[key]).dot(R_total)
            break  # only one yaw key allowed

    for key in ('up', 'down'):
        if key in angle_offsets:
            R_total = rot_from(key, angle_offsets[key]).dot(R_total)
            break  # only one pitch key allowed

    # Rotate drone's forward vector ----------------------------------------
    fwd_world = R.dot(fwd_body)
    offset_dir = R_total.dot(fwd_world)

    # Scale & translate ----------------------------------------------------
    goal = Point()
    goal.x = position.x + distance * offset_dir[0]
    goal.y = position.y + distance * offset_dir[1]
    goal.z = position.z + distance * offset_dir[2]
    return goal


def send_position_to_flask():
    pose = get_current_pose()
    if pose is None:
        return
    pos = pose.position
    url = URL_POS
    data = {"x": pos.x, "y": pos.y, "z": pos.z}
    try:
        requests.post(url, json=data)
    except Exception as e:
        rospy.logerr(f"Failed to send position to Flask: {e}")


def navigate(target: Point, avoid=False, image=False, communicate=False):
    
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
            takePhoto()
            return     

        step_size = min(STEP_DIST, distance)
        ratio = step_size / distance
        step_point = Point(
            current_position.x + dx * ratio,
            current_position.y + dy * ratio,
            current_position.z + dz * ratio
        )
        
        if avoid:
            open_point = first_open_point(step_point, FALLBACK_ANGLES, BOX_SIZE)
            if open_point != step_point:
                step_point = open_point
                back_vector = np.array([ -STEP_DIST/4, 0, 0 ])  
                R = tft.quaternion_matrix(current_quaternion)[:3, :3]
                back_offset = R.dot(back_vector)
                back_point = Point(
                    step_point.x + back_offset[0],
                    step_point.y + back_offset[1],
                    step_point.z + Z_OFFSET
                )
                step_point = back_point

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
        
        if communicate:
            send_position_to_flask()

            
def takePhoto(filename=None, front=False):
    bridge = CvBridge()
    latest_image = {"image": None}
    latest_pose = get_current_pose()

    if latest_pose is None:
        rospy.logerr("Current pose unavailable. Cannot take photo.")
        return

    def image_callback(msg):
        latest_image["image"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    if not front:
        sub = rospy.Subscriber('/firefly/image_raw', Image, image_callback)
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout
    else:
        sub = rospy.Subscriber('/firefly/vi_sensor/camera_depth/camera/image_raw', Image, image_callback)
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_image["image"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    sub.unregister()

    if latest_image["image"] is None:
        rospy.logwarn("Could not capture image.")
        return
    
    # Enhance brightness and vibrancy
    # latest_image = cv2.convertScaleAbs(latest_image["image"], alpha=1.2, beta=30)

    # Save the image with the drone's position as the filename
    position = latest_pose.position
    if not filename:
        filename = f"images/{position.x:.2f}_{position.y:.2f}_{position.z:.2f}.png"
    else:
        filename = f"images/{filename}.png"

    os.makedirs("images", exist_ok=True)
    cv2.imwrite(filename, latest_image["image"])
    rospy.loginfo(f"Photo saved as {filename}")
        

rospy.init_node("navigation_node", anonymous=True)
rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
rospy.Subscriber('/firefly/vi_sensor/camera_depth/depth/points', PointCloud2, point_cloud_callback)
tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
tf_listener = tf2_ros.TransformListener(tf_buf)
navigate(Point(-10, 0, 3), avoid=True)