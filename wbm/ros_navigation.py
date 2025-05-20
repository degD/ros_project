import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
import math


EPS = 0.5
MAX_DIST = 10.0  # Maximum distance for each step (adjust as needed)


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
        rospy.loginfo(f"Step {i}/{steps}: Moving to {intermediate}")
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


def simpleNavigation(waypoints: list):
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


if __name__ == "__main__":

    rospy.init_node("navigation_node", anonymous=True)
    simpleNavigation([Point(30, -10, 20)])
    rospy.spin()