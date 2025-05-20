import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped


EPS = 0.5


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
        print("aaa")
        pub.publish(pose_stamped)
        pose = latest_pose["pose"]
        if pose is not None:
            rospy.loginfo(pose)
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
        simpleMove(wp)