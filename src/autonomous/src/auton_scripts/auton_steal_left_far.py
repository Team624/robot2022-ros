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
auton_id = 7
auton_title = "Auton Steal Left Far"

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
        return FlyWheelSpin(self.ros_node)

class FlyWheelSpin(Flywheel): #Pre-revving (While it's following firstPath)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_spin_up(0.5)

    def tick(self):
        return StartFirstPath(self.ros_node)

class StartFirstPath(StartPath): # Moving towards the left side's two balls (2.5 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 01")

    def tick(self):
        return DeployIntake(self.ros_node)

class DeployIntake(Intake): # This steals the left side's two balls (2 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        if self.ros_node.get_data("/diff_drive/path_achieved") and self.check_timer(1.0):
            return Pause(self.ros_node)
        return self

class Pause(Intake): # This steals the left side's two balls (2 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if self.check_timer(0.5):
            self.retract_intake()
            return Turn(self.ros_node)
        return self

class Turn(StartPath): #Moving towards the goal with a total of 5 balls (4 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.turn(-1.047)

    def tick(self):
        if self.check_timer(1.0):
            self.stop_turn()
            return StartSecondPath(self.ros_node)
        return self

class StartSecondPath(StartPath): #Moving towards the goal with a total of 5 balls (4 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 11")

    def tick(self):
        return RotateTurret(self.ros_node)

class RotateTurret(Turret):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.rotate_turret(-35.0)

    def tick(self):
        if self.ros_node.get_data("/diff_drive/path_achieved") and self.check_timer(0.5):
            return DeployIntakeShoot(self.ros_node)
        return self

class DeployIntakeShoot(Intake): # This steals the left side's two balls (2 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        return Prime(self.ros_node)

class Prime(Flywheel): #Moves hood and turns on lights to get ready to shoot (2 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.idle_flywheel()
        self.start_prime()

    def tick(self):
        if self.check_timer(2.0):
            return Shoot(self.ros_node)
        return self

class Shoot(Turret): #Fires 5 balls (4.5 seconds)
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_shoot()

    def tick(self):
        if self.check_timer(4.5):
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


