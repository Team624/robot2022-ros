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
auton_id = 9
auton_title = "Auton Five Ball B"

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
        return DeployIntake1(self.ros_node)

class DeployIntake1(Intake):
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
        if self.check_timer(0.5):
            return Prime1(self.ros_node)
        return self

class Prime1(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.check_timer(0.5) and self.finished_path(0):
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
        if self.check_timer(0.7):
            self.start_shoot()
            if self.get_ball_count() == 0:
                self.idle()
                return RetractIntake1(self.ros_node)
        return self

class RetractIntake1(Intake):
   
    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.retract_intake()

    def tick(self):
        return StartSecondPath(self.ros_node)
        

class StartSecondPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(1)

    def tick(self):
        if self.finished_path(1):
            return DeployIntake2(self.ros_node)
        return self

class DeployIntake2(Intake):
    """
    The state which waits for the second waypoint of the path.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        return StartThirdPath(self.ros_node)

class StartThirdPath(StartPath):
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
        self.start_path(3)

    def tick(self):
        if self.finished_path(3):
            return StartFifthpath(self.ros_node)
        return self





class StartFifthpath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(4)

    def tick(self):
        if self.check_timer(3.5):
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
            return Shoot1(self.ros_node)
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
        if self.check_timer(0.7):
            self.start_shoot()
            if self.get_ball_count() == 0:
                self.idle()
                return StartSixthpath(self.ros_node)
        return self



class StartSixthpath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(5)
    def tick(self):
        return Prime3(self.ros_node)
    

class Prime3(Shooter):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.finished_path(5):
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
    ros_node.subscribe("/pathTable/status/finishedPath", String)
    ros_node.subscribe("/auto/numBall", Float32)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown