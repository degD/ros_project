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
import tf.transformations as tft
import requests
import numpy as np
import tf2_ros
import tf2_geometry_msgs
# import mmrotate

EPS = 0.5
MAX_DIST = 0.5  # Maximum distance for each step
INFERENCE_THRESHOLD = 0.4
INFERENCE_Z = 20
VEHICLE_EPS = 5
URL_POS = "http://localhost:5000/update_drone_position"  
PLATE = True   # as a result makes plate texts DISABLED and skips recognizing plates.
BOX_SIZE = 0.7   # change if you need a larger safety margin
ROTATION_THRESHOLD = 0.2
PUB_RATE_HZ        = 15  
TIMEOUT_SEC        = 10  

FALLBACK_ANGLES = [
    {'left': 30},
    {'right': 30},
    {'left': 45, 'up': 15},
    {'right': 45, 'up': 15},
    {'up': 45},
    {'up': 90},
]


# Silence TF warnings
os.environ['ROSCONSOLE_CONFIG_FILE'] = '/home/deneg/ros_project/rosconsole.config'


def object_avoiding_navigation(target: Point):
    
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Aborting navigation.")
        return
    current_position = current_pose.position

    while True:
        
        dx = target.x - current_position.x
        dy = target.y - current_position.y
        dz = target.z - current_position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < EPS:
            rospy.loginfo("Arrived at target.")
            return     

        step_size = min(MAX_DIST, distance)
        ratio = step_size / distance
        step_point = Point(
            current_position.x + dx * ratio,
            current_position.y + dy * ratio,
            current_position.z + dz * ratio
        )
        
        # Calculate angle to target point
        dx = step_point.x - current_position.x
        dy = step_point.y - current_position.y
        angle_to_target = math.atan2(dy, dx)

        # Rotate drone to face the target point
        rospy.loginfo(f"Rotating drone to face target point at angle {math.degrees(angle_to_target):.2f} degrees.")
        stepRotateDrone(math.degrees(angle_to_target), relative=False)
        
        # Updating step_point after obstacle detection
        try:
            rospy.loginfo(f"Step point: x:{step_point.x:2f}, y:{step_point.y:2f}, z:{step_point.z:2f}")
            step_point = first_open_point(step_point, FALLBACK_ANGLES, BOX_SIZE)
        except:
            rospy.loginfo("Unable to continue.")
            return
        
        # Move to next step
        simpleMove(step_point)
        current_position = step_point
        rospy.loginfo("Reached step.")


def first_open_point(target: Point,
                     angle_offsets_list: list,
                     box_size: float = BOX_SIZE) -> Point:
    """
    Return `target` if it is free of obstacles; otherwise iterate through
    angle offsets (relative to the drone) and return the first candidate
    point that is clear.

    Args:
        target (Point): Desired waypoint in world frame.
        angle_offsets_list (list[dict]): e.g. [{'left':30}, {'left':30, 'up':15}]
        box_size (float): Edge length (metres) for box_is_clear.

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
    raise Exception

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

def box_is_clear(target_pt: Point, box_size: float, pc_timeout: float = 1.0) -> bool:
    """
    Checks if the cubic area around target_pt is free from obstacles.
    The function transforms points from the camera frame into global frame using TF.

    Args:
        target_pt (Point): Center of the box (global coordinates).
        box_size (float): Side length of the box in meters.
        pc_timeout (float): Timeout for receiving point cloud.

    Returns:
        bool: True if no obstacle intersects the box, False otherwise.
    """
    # Get point cloud
    try:
        pc_msg = rospy.wait_for_message('/firefly/vi_sensor/camera_depth/depth/points', PointCloud2, timeout=pc_timeout)
    except rospy.ROSException:
        rospy.logwarn("Timeout while waiting for point cloud")
        return False

    # Get current drone pose
    pose = get_current_pose()
    if pose is None:
        rospy.logwarn("Drone pose unavailable")
        return False

    global_frame = "world"
    cloud_frame = pc_msg.header.frame_id

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)

    try:
        tf_buf.can_transform(global_frame, cloud_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
    except tf2_ros.LookupException:
        rospy.logwarn("TF transform not available from %s to %s", cloud_frame, global_frame)
        return False

    # Define bounding box limits
    half = box_size / 2.0
    min_x, max_x = target_pt.x - half, target_pt.x + half
    min_y, max_y = target_pt.y - half, target_pt.y + half
    min_z, max_z = target_pt.z - half, target_pt.z + half

    # Go through each point in the point cloud
    for pt in pcl2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
        raw_pt = PointStamped()
        raw_pt.header.frame_id = cloud_frame
        raw_pt.point.x = pt[0]
        raw_pt.point.y = pt[1]
        raw_pt.point.z = pt[2]

        try:
            trans_pt = tf_buf.transform(raw_pt, global_frame, timeout=rospy.Duration(0.1)).point
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            continue  # skip this point if transform fails

        # Check if point is within bounding box
        if (min_x <= trans_pt.x <= max_x and
            min_y <= trans_pt.y <= max_y and
            min_z <= trans_pt.z <= max_z):
            return False  # Obstacle found

    return True  # No obstacle in the box

def get_current_pose():
    """
    Helper function to get the current pose of the UAV.
    Returns:
        Pose: The current pose of the UAV.
    """
    latest_pose = {"pose": None}
    def pose_callback(msg):
        latest_pose["pose"] = msg

    sub = rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_pose["pose"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    sub.unregister()
    if latest_pose["pose"] is None:
        rospy.logwarn("Could not get current pose.")
    return latest_pose["pose"]

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

def stepMove(target_point: Point):
    """
    Moves the UAV to the target point in steps, each not exceeding MAX_DIST.
    Args:
        target_point (Point): The final destination point.
    """
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Aborting stepMove.")
        return

    start = current_pose.position
    distance = math.sqrt(
        (target_point.x - start.x) ** 2 +
        (target_point.y - start.y) ** 2 +
        (target_point.z - start.z) ** 2
    )
    rospy.loginfo(f"Starting stepMove: distance to target = {distance:.2f} m")

    steps = int(math.ceil(distance / MAX_DIST)) if distance > 0 else 1
    rospy.loginfo(f"Splitting move into {steps} steps.")

    for i in range(1, steps + 1):
        dx = target_point.x - start.x
        dy = target_point.y - start.y
        dz = target_point.z - start.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance < EPS:
            break

        # Move a max of MAX_DIST or the remaining distance
        step_size = min(MAX_DIST, distance)
        ratio = step_size / distance
        intermediate = Point(
            start.x + dx * ratio,
            start.y + dy * ratio,
            start.z + dz * ratio
        )
        rospy.loginfo(f"Step {i}/{steps}: Moving to\n{intermediate}")
        simpleMove(intermediate)
        start = intermediate

    rospy.loginfo("Arrived at target using stepMove.")


def simpleMove(point: Point, server=False):
    """
    Publishes a PoseStamped message to a ROS topic and waits until the robot arrives at the target point.
    The drone will face the target point before moving.

    Args:
        point (Point): The target point to move to.
    """
    # Get current drone position
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Aborting simpleMove.")
        return

    current_position = current_pose.position

    # Calculate angle to target point
    dx = point.x - current_position.x
    dy = point.y - current_position.y
    
    if abs(dx) > EPS or abs(dy) > EPS:
        # Rotate drone to face the target point
        
        angle_to_target = math.atan2(dy, dx)
        print(current_position, angle_to_target)
        rospy.loginfo(f"Rotating drone to face target point at angle {math.degrees(angle_to_target):.2f} degrees.")
        stepRotateDrone(math.degrees(angle_to_target), relative=False)
    
    # Get current pose after rotation
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Aborting simpleMove.")
        return

    # Publish PoseStamped message to move to the target point
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "world"
    pose_stamped.pose.position = point
    pose_stamped.pose.orientation = current_pose.orientation 

    rospy.loginfo(f"Publishing target pose:\n{point}")

    latest_pose = {"pose": None}
    def pose_callback(msg):
        latest_pose["pose"] = msg

    sub = rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    rate = rospy.Rate(40)
    arrived = False

    rospy.loginfo("Waiting for the robot to arrive...")
    while not rospy.is_shutdown() and not arrived:
        pub.publish(pose_stamped)
        pose = latest_pose["pose"]
        if pose is not None:
            if (
                abs(pose.position.x - point.x) < EPS and
                abs(pose.position.y - point.y) < EPS and
                abs(pose.position.z - point.z) < EPS
            ):
                arrived = True
        rate.sleep()

    rospy.loginfo("Robot has arrived at the target point.")
    sub.unregister()
    
    if server:
        send_position_to_flask()


def convertNavDataToWaypoints(navData):
    """
    Converts navigation data into a list of waypoint Point objects relative to the home position.

    Args:
        navData (dict): A dictionary containing navigation data with the following keys:
            - "tiles": A list of coordinate pairs (x, y) representing tile positions.
            - "home": A coordinate pair (x, y) representing the home position.
            - "height": A numeric value representing the height for all waypoints.

    Returns:
        list: A list of Point objects, each representing a waypoint relative to the home position at the specified height.
    """

    waypoints = list()
    for pt in navData["tiles"]:
        point = Point(pt["x"] - navData["home"]["x"], pt["y"] - navData["home"]["y"], navData["height"])
        waypoints.append(point)
    return waypoints


def simpleNavigation(waypoints: list, image=False):
    """
    Navigates through a list of waypoints one by one.

    This function iterates over each waypoint, logging progress and then calling
    the simpleMove function to move to the specified waypoint.

    Args:
        waypoints (list[Point]): A list of waypoints where each waypoint is an
        instance of Point indicating the coordinates to navigate to.

    Returns:
        None
    """

    wpNum = len(waypoints)
    for i, wp in enumerate(waypoints):
        rospy.loginfo(f"Moving to waypoint #{i}/{wpNum}...")
        stepMove(wp)
        if image:
            rospy.sleep(3)
            takePhoto()


# Subscribe to the topic '/firefly/image_raw' and save the RGB image into 'images' dir.
# With the drone position (x,y,z) as file name.
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

# Remove all the images in the 'images' folder
def resetPhotos():
    folder = "images"
    if not os.path.exists(folder):
        rospy.loginfo(f"No images to delete. Folder '{folder}' does not exist.")
        return

    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
                rospy.loginfo(f"Deleted {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {file_path}: {e}")
    rospy.loginfo("All images have been deleted.")

def resetOutput():
    folder = "output"
    if not os.path.exists(folder):
        rospy.loginfo(f"No output images to delete. Folder '{folder}' does not exist.")
        return

    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
                rospy.loginfo(f"Deleted {file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {file_path}: {e}")
    rospy.loginfo("All output images have been deleted.")

# Run inference on all the images and return their global coordinates
def inferenceImages():
    folder = "images"
    if not os.path.exists(folder):
        rospy.loginfo(f"No images to delete. Folder '{folder}' does not exist.")
        return 

    coords = set()
    def is_far_enough(new_coord):
        for existing_coord in coords:
            distance = math.sqrt(
                (new_coord[0] - existing_coord[0]) ** 2 +
                (new_coord[1] - existing_coord[1]) ** 2
            )
            if distance < VEHICLE_EPS:
                return False
        return True

    for filename in os.listdir(folder):
        print(filename)
        for data in detect.inference_with_absolute_coordinates(filename):
            if data["score"] > INFERENCE_THRESHOLD:
                new_coord = (
                    round(data["x_global"]), 
                    round(data["y_global"]), 
                    INFERENCE_Z
                )
                if is_far_enough(new_coord):
                    coords.add(new_coord)

                print(f'\t{data["x_global"]}, {data["y_global"]}')

    for p in coords:
        print(f"({p[0]}, {p[1]}, {p[2]})")
    print(f"LEN: {len(coords)}")

    return coords

# ---- helpers -------------------------------------------------------------
def norm_rad(angle):
    """Normalize to (-π, π]."""
    angle = angle % (2.0 * math.pi)
    return angle - 2.0 * math.pi if angle > math.pi else angle

def yaw_from_pose(pose):
    q = (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w)
    _, _, yaw = euler_from_quaternion(q)
    return norm_rad(yaw)

def build_pose(position, roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "world"
    ps.pose.position = position
    ps.pose.orientation = Quaternion(*q)
    return ps

# ---- core rotation -------------------------------------------------------
def rotateDrone(degrees, relative=True):
    """
    Rotate the drone by `degrees`.
      • If relative=True  → turn *delta* degrees.
      • If relative=False → face the *absolute* yaw (deg) in world frame.
    """
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(PUB_RATE_HZ)

    start_pose = get_current_pose()
    if start_pose is None:
        rospy.logerr("Current pose unavailable — cannot rotate.")
        return

    roll, pitch, current_yaw = euler_from_quaternion((
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w
    ))

    target_yaw = norm_rad(current_yaw + math.radians(degrees)) if relative \
                 else norm_rad(math.radians(degrees))

    cmd_pose = build_pose(start_pose.position, roll, pitch, target_yaw)

    rospy.loginfo(f"Rotating to {'relative' if relative else 'absolute'} "
                  f"{degrees:.1f}°  (target yaw {math.degrees(target_yaw):.1f}°)")

    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(cmd_pose)                       # keep sending set-point
        now_pose = get_current_pose()
        if now_pose:
            yaw_err = norm_rad(target_yaw - yaw_from_pose(now_pose))
            if abs(yaw_err) <= ROTATION_THRESHOLD:
                rospy.loginfo("Rotation arrived (|Δyaw| ≤ %.2f°)",
                              math.degrees(ROTATION_THRESHOLD))
                return
        if (rospy.Time.now() - t0).to_sec() > TIMEOUT_SEC:
            rospy.logwarn("Rotate timeout after %.1f s; continuing anyway", TIMEOUT_SEC)
            return
        rate.sleep()

# ---- stepped version -----------------------------------------------------
def stepRotateDrone(degrees, stepsize=90, relative=True):
    """
    Rotate in multiple segments (useful for large turns).
      • `degrees`  : final desired rotation (deg).
      • `stepsize` : magnitude of each incremental step (deg).
      • `relative` : True → cumulative delta, False → absolute yaw goal.
    """
    # normalise target degrees for easier logic
    if relative:
        total = degrees
    else:
        total = norm_rad(math.radians(degrees)) * 180.0 / math.pi  # back to deg
        if total >  180: total -= 360
        if total <= -180: total += 360

    if relative:
        if abs(total) < 1e-2:   # already here
            rospy.loginfo("Step rotation skipped — negligible request.")
            return

    step_sign = 1 if total >= 0 else -1
    step_deg  = step_sign * abs(stepsize)
    segments  = []

    if relative:
        # e.g. 250° with 90° steps → 90,90,70
        n_full = int(abs(total) // abs(stepsize))
        segments = [step_deg] * n_full
        remainder = total - step_deg * n_full
        if abs(remainder) > 1e-2:
            segments.append(remainder)
    else:
        # Absolute: generate intermediate absolute yaws
        current_yaw_deg = math.degrees(yaw_from_pose(get_current_pose()))
        target_deg = total
        diff = target_deg - current_yaw_deg
        diff = (diff + 180) % 360 - 180   # shortest-direction diff

        n_full = int(abs(diff) // abs(stepsize))
        for i in range(1, n_full + 1):
            segments.append(current_yaw_deg + i * step_sign * abs(stepsize))
        segments.append(target_deg)
        # execute as absolute moves
        for yaw_abs in segments:
            rotateDrone(yaw_abs, relative=False)
        return

    # execute relative segments
    for seg in segments:
        rotateDrone(seg, relative=True)


# Fly over the car (for example (10,-8))
# Rotate around it, maybe with an altitude of 10m or less
# Get photos of the front and back
# Then read the plates
def readCarPlate(car_x, car_y, car_angle, save=False, behind=3, alt=10, index=-1):
    """
    Move the drone behind the car using car_x, car_y, and car_angle.
    A placeholder for reading the plate is returned.
    """
    altitude = alt
    behind_offset = behind

    deg_angle = math.degrees(car_angle) - 90
    behind_x = car_x - behind_offset * math.sin(car_angle)
    behind_y = car_y - behind_offset * math.cos(car_angle)
    # behind_x = car_x
    # behind_y = car_y
    rospy.loginfo(f"Moving behind ({behind}) the car at ({behind_x}, {behind_y}) with angle {car_angle}.")
    stepMove(Point(behind_x, behind_y, altitude))
    stepMove(Point(behind_x, behind_y, 0.7))
    stepRotateDrone(-1*deg_angle)

    # Try plate recognizing
    if PLATE:
        takePhoto("plate", front=True)
        plateText = plate.plateRecognizerDetect("images/plate.png")
    else:
        plateText = "DISABLED"

    if save:
        with open("plates.txt", "a") as f:
            f.write(f"{index},{plateText.upper()}\n")

    # Reset rotation
    stepRotateDrone(deg_angle)
    stepMove(Point(behind_x, behind_y, altitude))

    return plateText


def readPlates(abs_coords):
    """
    Reads plates starting from the closest coordinate to the drone's current position.

    Args:
        abs_coords (list): A list of dictionaries containing absolute coordinates with keys:
            - "x_global": X-coordinate of the detection.
            - "y_global": Y-coordinate of the detection.
            - "angle": Angle of the detection.
    """
    indexed_abs_coords = [(i, detection) for i, detection in enumerate(abs_coords)]
    while indexed_abs_coords:
        current_pose = get_current_pose()
        if current_pose is None:
            rospy.logerr("Current pose unavailable. Cannot determine closest coordinate.")
            return

        # Calculate distances and find the closest coordinate
        current_x, current_y = current_pose.position.x, current_pose.position.y
        i, closest = min(
            indexed_abs_coords,
            key=lambda detection: math.sqrt(
                (detection[1]["x_global"] - current_x) ** 2 +
                (detection[1]["y_global"] - current_y) ** 2
            )
        )

        # Process the closest coordinate
        readCarPlate(
            closest["x_global"],
            closest["y_global"],
            closest["angle"],
            save=True,
            behind = closest["w_global"] + 1.4,
            index=i
        )

        # Remove the processed coordinate from the list
        indexed_abs_coords.remove((i, closest))


def resetPlates():
    """
    Deletes the plates.txt file if it exists.
    """
    filename = "plates.txt"
    if os.path.exists(filename):
        try:
            os.remove(filename)
            rospy.loginfo(f"Deleted {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {filename}: {e}")
    else:
        rospy.loginfo(f"No plates.txt file to delete.")


if __name__ == "__main__":

    rospy.init_node("navigation_node", anonymous=True)
    # rotateDrone(90)
    # simpleMove(Point(0, 0, 3))
    # print(box_is_clear(Point(9, 0, 1), 1, 1))
    
    # print(point_at_angular_offset(
    #     Point(0, 0, 1), 
    #     Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    #     2, {"up": 90, "left": 90}
    # ))
    
    # fallback_angles = [
    #     {'left': 30},                     # 30° left
    #     {'right': 30},                    # 30° right
    #     {'left': 45, 'up': 15},           # up-left diagonal
    #     {'right': 45, 'up': 15},          # up-right diagonal
    # ]

    # safe_point = first_open_point(Point(2, 0, 1), fallback_angles)
    # simpleMove(safe_point)
    
    object_avoiding_navigation(Point(5, 0, 1))
    
    # stepRotateDrone(-180, relative=False)
    # stepMove(Point(0, 0, 10))

    # resetPlates()
    # resetPhotos()
    # MAX_DIST = 10.0
    # simpleNavigation([Point(0, 0, 80)], image=True)
    # simpleNavigation([Point(0, 0, 10)], image=False)
    # MAX_DIST = 1.0
    # # detect.init()

    # for f in os.listdir("images"):
    #     detect.draw_detections_on_image(f, f"output/det_{f}")
    #     abs_coords = detect.inference_with_absolute_coordinates(f, remove_repetations=True)

    # # for detection in abs_coords:
    # #     readCarPlate(
    # #         detection["x_global"], 
    # #         detection["y_global"], 
    # #         detection["angle"],
    # #         save=True)

    # # simpleNavigation([Point(-16, -4, 20)])
    # # readCarPlate(-14.6, -1.74, 1.51, save=True)

    # readPlates(abs_coords)

# {'x_global': 24.170509596565196, 'y_global': 6.748032890355546, 'altitude': 79.8, 'angle': 0.5816325, 'score': 0.8759265, 'w_global': 1.9124779614028897, 'h_global': 0.8567711158815933}
    # readCarPlate(24, 6.7, 0.581, save=False, behind=0, alt=5)
