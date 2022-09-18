import rospy
from std_msgs.msg import Float32, Float64, Bool, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from .auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray

from .auton_modules.state import SetIdle, State, StartPath, Intake, Shooter, Hood, Flywheel

# The id of the auton, used for picking auton
auton_id = 6
auton_title = "Auton Six Ball"

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
        return Prime1(self.ros_node)

class Prime1(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.check_timer(0.6) and self.finished_path(0):
            return Shoot1(self.ros_node)
        return self

class Shoot1(Shooter):
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
            return StartSecondThirdPath(self.ros_node)
        return self

class StartSecondThirdPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(2)

    def tick(self):
        if self.finished_path(2):
            return StartFourthPath(self.ros_node)
        return self

class StartFourthPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if (self.check_timer(0.2)):
            self.start_path(3)
        if self.get_path() == 3:
            return StartFifthPath(self.ros_node)
        return self

class StartFifthPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if (self.check_timer(0.5)):
            self.start_path(4)
        if self.get_path() == 4:
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
        if self.finished_path(4):
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
        if self.check_timer(1.0):
            self.start_shoot()
        if self.get_ball_count() == 0:
            self.idle()
            return StartSixthSeventhPath(self.ros_node)
        return self

class StartSixthSeventhPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(6)

    def tick(self):
        if self.get_path() == 6:
            return Prime3(self.ros_node)
        return self

class Prime3(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.finished_path(6):
            return Shoot3(self.ros_node)
        return self

class Shoot3(Shooter):
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


    ros_node.subscribe("/pathTable/status/path", Float32)
    ros_node.subscribe("/pathTable/status/point", Float32)
    ros_node.subscribe("/pathTable/status/finishedPath", String)
    ros_node.subscribe("/auto/numBall", Float32)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown