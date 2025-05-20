import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped


EPS = 0.5


def simpleMove(point: Point):
    """
    Publishes a PoseStamped message to a ROS topic and waits until the robot arrives at the target point.

    Args:
        point (Point): The target point to move to.
    """
    # Create a publisher for the topic
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)

    # Create a PoseStamped message
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "map"  # Adjust the frame_id as needed
    pose_stamped.pose.position = point
    pose_stamped.pose.orientation.w = 1.0  # Neutral orientation

    # Publish the message
    rospy.loginfo(f"Publishing target pose: {point}")

    # Wait for the robot to arrive
    rate = rospy.Rate(10)  # 10 Hz
    arrived = False

    rospy.loginfo("Waiting for the robot to arrive...")
    while not rospy.is_shutdown() and not arrived:
        pub.publish(pose_stamped)
        rate.sleep()

        pose: Pose = rospy.wait_for_message('/firefly/ground_truth/pose', Pose)
        if (
            abs(pose.position.x - point.x) < EPS and 
            abs(pose.position.y - point.y) < EPS and 
            abs(pose.position.z - point.z) < EPS
        ):
            arrived = True

    rospy.loginfo("Robot has arrived at the target point.")


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