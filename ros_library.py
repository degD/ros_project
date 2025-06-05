import detect
import plate
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
import math
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler

EPS = 0.5
MAX_DIST = 10.0  # Maximum distance for each step (adjust as needed)
INFERENCE_THRESHOLD = 0.5
INFERENCE_Z = 20
VEHICLE_EPS = 5


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


def simpleMove(point: Point):
    """
    Publishes a PoseStamped message to a ROS topic and waits until the robot arrives at the target point.

    Args:
        point (Point): The target point to move to.
    """
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position = point
    pose_stamped.pose.orientation.w = 1.0

    rospy.loginfo(f"Publishing target pose:\n{point}")

    latest_pose = {"pose": None}
    def pose_callback(msg):
        latest_pose["pose"] = msg

    sub = rospy.Subscriber('/firefly/ground_truth/pose', Pose, pose_callback)
    rate = rospy.Rate(10)
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
            rospy.sleep(1)
            takePhoto()


# Subscribe to the topic '/firefly/image_raw' and save the RGB image into 'images' dir.
# With the drone position (x,y,z) as file name.
def takePhoto(filename=None):
    bridge = CvBridge()
    latest_image = {"image": None}
    latest_pose = get_current_pose()

    if latest_pose is None:
        rospy.logerr("Current pose unavailable. Cannot take photo.")
        return

    def image_callback(msg):
        latest_image["image"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    sub = rospy.Subscriber('/firefly/image_raw', Image, image_callback)
    rate = rospy.Rate(10)
    timeout = rospy.Time.now() + rospy.Duration(2.0)  # 2 seconds timeout

    while not rospy.is_shutdown() and latest_image["image"] is None and rospy.Time.now() < timeout:
        rate.sleep()
    sub.unregister()

    if latest_image["image"] is None:
        rospy.logwarn("Could not capture image.")
        return

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


def rotateDrone(degrees):
    """
    Rotates the drone in place by 'degrees' (relative to its current heading).
    No persistence in future commands (once complete, orientation remains changed).
    """
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)
    current_pose = get_current_pose()
    if current_pose is None:
        rospy.logerr("Current pose unavailable. Cannot rotate.")
        return

    # Convert current orientation to Euler angles (roll, pitch, yaw)
    q = (current_pose.orientation.x,
         current_pose.orientation.y,
         current_pose.orientation.z,
         current_pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion(q)

    # Calculate new orientation
    new_yaw = yaw + math.radians(degrees)
    new_q = quaternion_from_euler(roll, pitch, new_yaw)

    # Build a PoseStamped with same position but updated orientation
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position = current_pose.position
    pose_stamped.pose.orientation.x = new_q[0]
    pose_stamped.pose.orientation.y = new_q[1]
    pose_stamped.pose.orientation.z = new_q[2]
    pose_stamped.pose.orientation.w = new_q[3]

    # Publish until the drone is close to the desired yaw
    rate = rospy.Rate(10)
    arrived = False
    rospy.loginfo(f"Rotating by {degrees} degrees...")

    while not rospy.is_shutdown() and not arrived:
        pub.publish(pose_stamped)
        updated_pose = get_current_pose()
        if updated_pose:
            uq = (updated_pose.orientation.x,
                  updated_pose.orientation.y,
                  updated_pose.orientation.z,
                  updated_pose.orientation.w)
            _, _, current_yaw = euler_from_quaternion(uq)
            if abs(current_yaw - new_yaw) < 0.05:  # Radians threshold
                arrived = True
        rate.sleep()

    rospy.loginfo("Drone rotation complete.")


def stepRotateDrone(degrees, step=90):
    if abs(degrees) < abs(step):
        rotateDrone(degrees)
        return
    s = 0
    while abs(s) < abs(degrees):
        rotateDrone((degrees / abs(degrees)) * step)
        s += step


# Fly over the car (for example (10,-8))
# Rotate around it, maybe with an altitude of 10m or less
# Get photos of the front and back
# Then read the plates
def readCarPlate(car_x, car_y, car_angle, save=False):
    """
    Move the drone behind the car using car_x, car_y, and car_angle.
    A placeholder for reading the plate is returned.
    """
    altitude = 3
    behind_offset = 4.0

    rad_angle = math.radians(car_angle)
    behind_x = car_x - behind_offset * math.cos(rad_angle)
    behind_y = car_y - behind_offset * math.sin(rad_angle)
    rospy.loginfo(f"Moving behind the car at ({behind_x}, {behind_y}) with angle {car_angle}.")
    stepMove(Point(behind_x, behind_y, altitude))
    stepRotateDrone(car_angle)

    # Try plate recognizing
    takePhoto("plate")
    plateText = plate.plateRecognizerDetect("images/plate.png")

    if save:
        with open("plates.csv", "a") as f:
            f.write(f"{car_x},{car_y},{car_angle},{plateText}\n")

    return plateText


def resetPlates():
    """
    Deletes the plates.csv file if it exists.
    """
    filename = "plates.csv"
    if os.path.exists(filename):
        try:
            os.remove(filename)
            rospy.loginfo(f"Deleted {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to delete {filename}: {e}")
    else:
        rospy.loginfo(f"No plates.csv file to delete.")


if __name__ == "__main__":

    rospy.init_node("navigation_node", anonymous=True)

    resetPlates()
    resetPhotos()
    simpleNavigation([Point(0, 0, 100)], image=True)

    for f in os.listdir("images"):
        if os.path.isfile(os.path.join("images", f)):
            image_path = os.path.join("images", f) 
            abs_coords = detect.inference_with_absolute_coordinates(image_path)

    for detection in abs_coords:
        readCarPlate(
            detection["x_global"], 
            detection["y_global"], 
            detection["angle"],
            save=True)
        