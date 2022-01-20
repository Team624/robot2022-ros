import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray

from auton_modules.state import SetIdle, State, StartPath, Intake, Shooter, Turret, Hood, Flywheel

# The id of the auton, used for picking auton
auton_id = 4
auton_title = "Auton Center"

class Idle(SetIdle):
    """
    The state which waits for autonomous to start
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.setRobotPose()
        self.setIdle()

    def tick(self):
        return RotateTurret(self.ros_node)

class RotateTurret(Turret):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.rotate_turret(-90.0)

    def tick(self):
        return FlyWheelSpin(self.ros_node)

class FlyWheelSpin(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_spin_up(0.5)

    def tick(self):
        if self.check_timer(1.3):
            return Prime(self.ros_node)
        return self

class Prime(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.idle_flywheel()
        self.start_prime()

    def tick(self):
        if self.check_timer(1.3):
            return Shoot(self.ros_node)
        return self

class Shoot(Turret):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_shoot()

    def tick(self):
        if self.check_timer(1.5):
            self.idle_turret()
            return StopShoot(self.ros_node)
        return self

class StopShoot(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.idle()
        self.start_spin_up(0.5)

    def tick(self):
        return StartFirstPath(self.ros_node)


class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 01")

    def tick(self):
        return DeployIntake(self.ros_node)

class DeployIntake(Intake):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        if self.ros_node.get_data("/diff_drive/path_achieved") and self.check_timer(0.5):
            return StartSecondPath(self.ros_node)
        return self

class StartSecondPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 11")

    def tick(self):
        if self.ros_node.get_data("/diff_drive/path_achieved") and self.check_timer(0.5):
            return StartThirdPath(self.ros_node)
        return self

class StartThirdPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 21")

    def tick(self):
        return FlyWheelSpin2(self.ros_node)

class FlyWheelSpin2(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_spin_up(0.5)

    def tick(self):
        if self.check_timer(1) and self.ros_node.get_data("/diff_drive/path_achieved"):
            return Prime2(self.ros_node)
        return self

class Prime2(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.check_timer(1.5):
            return Shoot2(self.ros_node)
        return self

class Shoot2(Turret):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_shoot()

    def tick(self):
        if self.check_timer(2.0):
            return StopShoot2(self.ros_node)
        return self

class StopShoot2(Flywheel):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.idle()
        self.start_spin_up(0.5)

    def tick(self):
        return Final(self.ros_node)

class Final(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        rospy.loginfo("END OF AUTON")

    def tick(self):
        return self

class Shutdown(SetIdle):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        pass

    def execute_action(self):
        self.setIdle()

    def tick(self):
        return self

def start(ros_node):
    # Pick which topics to subscribe to
    ros_node.subscribe("/diff_drive_go_to_goal/distance_to_goal", Float32)
    ros_node.subscribe("/diff_drive/waypoints_achieved", BoolArray)
    ros_node.subscribe("/diff_drive/path_achieved", Bool)

    ros_node.subscribe("/auto/turret/current/angle", Float32)
    ros_node.subscribe("/auto/flywheel/current/rpm", Float32)
    ros_node.subscribe("/auto/hood/current/angle", Float32)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown