import rospy
from std_msgs.msg import Float32, Float64, Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray

from auton_modules.state import SetIdle, State, StartPath, Intake, Shooter, Hood, Flywheel, Color

# The id of the auton, used for picking auton
auton_id = 8
auton_title = "Auton Three Ball"

# Start of our states
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
        return DisableColor(self.ros_node)

class DisableColor(Color):
    """
    The state which waits for the second waypoint of the path.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.disable_color()

    def tick(self):
        return DeployIntake(self.ros_node)

class DeployIntake(Intake):
    """
    The state which waits for the second waypoint of the path.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        return StartFirstPath(self.ros_node)

class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(0)

    def tick(self):
        if self.check_timer(0.5) and self.get_point() > 12:
            return Prime(self.ros_node)
        return self

class Prime(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.check_timer(0.5) and self.finished_path(0):
            return Shoot(self.ros_node)
        return self

class Shoot(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if self.check_timer(0.4):
            self.start_shoot()
        if self.get_ball_count() == 0:
            self.idle()
            return StartSecondPath(self.ros_node)
        return self


class StartSecondPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(1)

    def tick(self):
        if self.finished_path(1) or self.get_ball_count() == 1:
            return RetractIntake(self.ros_node)
        return self

class RetractIntake(Intake):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.retract_intake()

    def tick(self):
        if self.finished_path(1):
            return EnableColor(self.ros_node)
        return self

class EnableColor(Color):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.enable_color()

    def tick(self):
        if self.finished_path(1):
            return Hide(self.ros_node)
        return self

class Hide(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.hide_poop()

    def tick(self):
        if self.check_timer(1) and self.get_ball_count() == 0:
            self.idle()
            return StartThirdAndFourthPath(self.ros_node)
        return self

class StartThirdAndFourthPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(3)

    def tick(self):
        if self.get_ball_count() >= 1:
            return Prime2(self.ros_node)
        return self

class Prime2(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.check_timer(0.5) and self.finished_path(3):
            return Shoot2(self.ros_node)
        return self

class Shoot2(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if self.check_timer(0.5):
            self.start_shoot()
        if self.get_ball_count() == 0:
            self.idle()
            return Final(self.ros_node)
        return self

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

    ros_node.subscribe("/pathTable/status/path", Float32)
    ros_node.subscribe("/pathTable/status/point", Float32)
    ros_node.subscribe("/auto/numBall", Float32)
    ros_node.subscribe("/pathTable/status/finishedPath", String)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown