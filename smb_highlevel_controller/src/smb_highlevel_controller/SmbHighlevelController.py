import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2


class SmbHighlevelController:
    def __init__(self, scan_topic="/scan", subscriber_queue_size=10):
        self._scan_topic = scan_topic
        self._subscriber_queue_size = subscriber_queue_size
        if not self._read_parameters():
            rospy.logerr("Could not read parameters")
            rospy.signal_shutdown()

        self._scan_subscriber = rospy.Subscriber(
            self._scan_topic, LaserScan, self._scan_callback, queue_size=self._subscriber_queue_size
        )
        self._pcl_subscriber = rospy.Subscriber(
            "/rslidar_points", PointCloud2, self._pcl_callback, queue_size=self._subscriber_queue_size
        )

    def on_shutdown(self):
        self._scan_subscriber.unregister()
        self._pcl_subscriber.unregister()

    def _read_parameters(self):
        try:
            self._scan_topic = rospy.get_param(
                "/smb_highlevel_controller/scan_subscriber_topic_name", self._scan_topic
            )
            self._subscriber_queue_size = rospy.get_param(
                "/smb_highlevel_controller/scan_subscriber_queue_size",
                self._subscriber_queue_size,
            )
            return True
        except KeyError:
            return False

    def _scan_callback(self, msg: LaserScan):
        minimum = min(msg.ranges)
        rospy.loginfo_throttle(2.0, f"Minimum range [m]: {minimum}")

    def _pcl_callback(self, msg: PointCloud2):
        size = msg.height * msg.row_step
        rospy.loginfo_throttle(2.0, f"Number of points in 3D cloud: {size}")


def main():
    rospy.init_node("smb_highlevel_controller")
    smb_highlevel_controller = SmbHighlevelController()
    rospy.on_shutdown(smb_highlevel_controller.on_shutdown)
    rospy.spin()
