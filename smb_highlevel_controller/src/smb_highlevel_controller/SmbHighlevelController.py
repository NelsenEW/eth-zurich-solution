import rospy
from math import atan2, degrees, sin, cos
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class SmbHighlevelController:
    def __init__(self, scan_topic="/scan", subscriber_queue_size=10):
        self._tf_buffer = tf2_ros.Buffer()
        self._tflistener = tf2_ros.TransformListener(self._tf_buffer)
        self._scan_topic = scan_topic
        self._subscriber_queue_size = subscriber_queue_size
        self._x_vel = 0.0
        self._kp_ang = 0.0
        self._service_name = ""
        self._is_start = False
        if not self._read_parameters():
            rospy.logerr("Could not read parameters")
            rospy.signal_shutdown()

        self._scan_subscriber = rospy.Subscriber(
            self._scan_topic, LaserScan, self._scan_callback, queue_size=self._subscriber_queue_size
        )
        self._pcl_subscriber = rospy.Subscriber(
            "/rslidar_points", PointCloud2, self._pcl_callback, queue_size=self._subscriber_queue_size
        )
        self._cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._marker_publisher = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )
        self._start_server = rospy.Service(
            self._service_name, SetBool, self._start_callback
        )

    def on_shutdown(self):
        self._scan_subscriber.unregister()
        self._pcl_subscriber.unregister()
        self._cmd_publisher.unregister()
        self._marker_publisher.unregister()

    def _read_parameters(self):
        try:
            self._scan_topic = rospy.get_param(
                "/smb_highlevel_controller/scan_subscriber_topic_name", self._scan_topic
            )
            self._subscriber_queue_size = rospy.get_param(
                "/smb_highlevel_controller/scan_subscriber_queue_size",
                self._subscriber_queue_size,
            )
            self._x_vel = rospy.get_param(
                "/smb_highlevel_controller/x_vel", self._x_vel
            )
            self._kp_ang = rospy.get_param(
                "/smb_highlevel_controller/p_gain_ang", self._kp_ang
            )
            self._service_name = rospy.get_param(
                "/smb_highlevel_controller/service_name", self._service_name
            )
            self._is_start = rospy.get_param(
                "/smb_highlevel_controller/start_robot", self._is_start
            )
            return True
        except KeyError:
            return False

    def _scan_callback(self, msg: LaserScan):
        goal_pose_lidar = self._get_goal_pose(msg)
        goal_pose_odom = self._transform_odom(goal_pose_lidar)
        self._move_to_goal(goal_pose_lidar)
        self._vis_marker_publish(goal_pose_odom)

    def _get_goal_pose(self, msg: LaserScan):
        goal_pose = PoseStamped()
        distance = min(msg.ranges)
        min_index = msg.ranges.index(distance)
        angle = msg.angle_min + msg.angle_increment * min_index
        goal_pose.header.frame_id = "rslidar"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = distance * cos(angle)
        goal_pose.pose.position.y = distance * sin(angle)
        goal_pose.pose.position.z = 0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        rospy.loginfo_throttle(2.0, f"Minimum range [m]: {distance}")
        rospy.loginfo_throttle(2.0, f"Angle from the robot [degree]: {degrees(angle)}")
        return goal_pose

    def _transform_odom(self, source_pose: PoseStamped):
        target_pose = PoseStamped()
        try:
            target_pose: PoseStamped = self._tf_buffer.transform(
                source_pose, "odom", rospy.Duration(0.1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(f"{e}")
        return target_pose

    def _move_to_goal(self, goal_pose: PoseStamped):
        vel_msg = Twist()
        if self._is_start:
            vel_msg.linear.x = self._x_vel
            vel_msg.angular.z = self._kp_ang * atan2(
                goal_pose.pose.position.y, goal_pose.pose.position.x
            )
        rospy.logdebug_throttle(2, f"{vel_msg.linear.x = }")
        rospy.logdebug_throttle(2, f"{vel_msg.angular.z = }")
        self._cmd_publisher.publish(vel_msg)

    def _vis_marker_publish(self, goal_pose: PoseStamped):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "smb_highlevel_controller"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = goal_pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self._marker_publisher.publish(marker)

    def _pcl_callback(self, msg: PointCloud2):
        size = msg.height * msg.row_step
        rospy.loginfo_throttle(2.0, f"Number of points in 3D cloud: {size}")

    def _start_callback(self, req: SetBoolRequest):
        resp = SetBoolResponse()
        resp.success = True
        if self._is_start != req.data:
            self._is_start = req.data
            resp.message = "Set SMB to " + "start" if req.data else "stop"
            rospy.logwarn(f"SMB {self._service_name} service called: {resp.message}")
        return resp


def main():
    rospy.init_node("smb_highlevel_controller")
    smb_highlevel_controller = SmbHighlevelController()
    rospy.on_shutdown(smb_highlevel_controller.on_shutdown)
    rospy.spin()
