import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class OctomapHeightFilter:
    def __init__(self):
        # ---------- Parameters ----------
        self.input_topic   = rospy.get_param("~input_cloud",  "/firefly/octomap_point_cloud_centers")
        self.output_topic  = rospy.get_param("~output_cloud", "/filtered_octomap")
        self.base_frame    = rospy.get_param("~base_frame",   "firefly/base_link")
        self.world_frame   = rospy.get_param("~world_frame",  "world")
        self.clearance     = rospy.get_param("~clearance",    0.10)  # m below the vehicle to keep
        self.max_keep_above= rospy.get_param("~max_keep",     20.0)  # optional upper crop (relative)

        # ---------- TF listener ----------
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ---------- ROS I/O ----------
        self.pub  = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub  = rospy.Subscriber(self.input_topic, PointCloud2,
                                     self.cloud_callback, queue_size=1)

        rospy.loginfo("octomap_height_filter: listening on %s, publishing on %s",
                      self.input_topic, self.output_topic)

    # ---------------------------------------------------------
    def cloud_callback(self, cloud_msg: PointCloud2) -> None:
        """
        Filter incoming cloud by Z relative to base_frame, republish result.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rospy.Time(0),             # latest available
                rospy.Duration(0.05))      # timeout
            current_z = tf.transform.translation.z
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            rospy.logwarn_throttle(5.0, "octomap_height_filter: TF lookup failed")
            return

        min_z = current_z - self.clearance
        max_z = current_z + self.max_keep_above

        # Iterate & keep points
        kept_xyz = []
        for x, y, z in pc2.read_points(cloud_msg,
                                       field_names=("x", "y", "z"),
                                       skip_nans=True):
            if min_z <= z <= max_z:
                kept_xyz.append((x, y, z))

        # Build new cloud (XYZ32 is sufficient for costmap_2d)
        header = Header(frame_id=self.world_frame, stamp=cloud_msg.header.stamp)
        filtered_msg = pc2.create_cloud_xyz32(header, kept_xyz)
        self.pub.publish(filtered_msg)

        rospy.logdebug("octomap_height_filter: in=%d  out=%d  z=%.2f m",
                       cloud_msg.width * cloud_msg.height,
                       len(kept_xyz),
                       current_z)

# =============================================================
if __name__ == "__main__":
    rospy.init_node("octomap_height_filter")
    OctomapHeightFilter()
    rospy.spin()