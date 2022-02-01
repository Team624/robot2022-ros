#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, String, Float32MultiArray
import threading
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from tf.transformations import quaternion_from_euler

# Creates proxy node
rospy.init_node('rviz_data')

class RvizData:

    def __init__(self):

        self.update_rate = rospy.get_param('~rate', 30)

        # A Dictionary of wanted topic name and data type
        self.input_data = rospy.get_param('~input_data', [])
        self.output_data = rospy.get_param('~output_data', [])

        # Collection of ROS subscribers and actual data
        self._data = {}

        # List of ROS publishers and subscribers to avoid calling one twice
        self._publishers = {}
        self._subscribers = []

        self.poses = []

        rospy.Subscriber("/auto/paths", GoalPath, self._on_new_path)

        # rospy.Subscriber("/auto/robot_set_pose", Float32MultiArray, self._on_reset_pose)

    def subscribe(self, topic_name, data_type):
        """ This sets up the ros subscribers for incoming data """
        # Checks if subscriber exists, if not create one
        if topic_name in self._subscribers:
            return
        rospy.Subscriber(topic_name, data_type, self._on_new_data)
        self._subscribers.append(topic_name)

    def publish(self, topic_name, data_type, data, queue = 10, latching = False):
        """ This publishes ros data """
        # Check if publisher exists, if not create and publish data
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, data_type, queue_size=queue, latch=latching)
        self._publishers[topic_name].publish(data)

    def get_data(self, topic_name, simple_data = True):
        """ This gets the subscribed ros data """

        if topic_name in self._data:
            if simple_data:
                return self._data[topic_name].data
            else:
                return self._data[topic_name]
        return None

    def _on_new_path(self, msg):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        b = msg
        if (b is not None):
            max_ind = b.number_of_paths
            print(max_ind)
            for a in b.goals:
                pose = PoseStamped()
                pose.pose = a.pose
                self.poses.append(pose)
            if (b.path_index == max_ind - 1):
                path_msg.poses = self.poses
                self.publish("/rviz/paths", Path, path_msg)
                self.poses = []

    def _on_new_data(self, msg):
        """ This is the callback function for the subscribers """
        self._data[str(msg._connection_header["topic"])] = msg

    def _on_reset_pose(self, msg):
        pass

    def main(self):
        """ This is the main loop """
        # Set how many times this should run per second
        r = rospy.Rate(self.update_rate)

        self.subscribe("/pose/x", Float32)
        self.subscribe("/pose/y", Float32)
        self.subscribe("/pose/th", Float32)
        while not rospy.is_shutdown():
            odom = Odometry()
            odom.header.frame_id = "map"

            if ((self.get_data("/pose/x") or self.get_data("/pose/x") or self.get_data("/pose/x")) is not None):
                odom.pose.pose.position.x = self.get_data("/pose/x")
                odom.pose.pose.position.y = self.get_data("/pose/y")
                quat = quaternion_from_euler(0,0,self.get_data("/pose/th"))
                odom.pose.pose.orientation.x = quat[0]
                odom.pose.pose.orientation.y = quat[1]
                odom.pose.pose.orientation.z = quat[2]
                odom.pose.pose.orientation.w = quat[3]

            self.publish("/rviz/odom", Odometry, odom)
            # Sleeps to meet specified rate
            r.sleep()

rviz_node = RvizData()
rviz_node.main()