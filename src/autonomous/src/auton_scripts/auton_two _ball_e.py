import rospy
from std_msgs.msg import Float32, Bool, String
from diff_drive.msg import BoolArray
from .auton_modules.state import SetIdle, State, StartPath, Intake, Shooter

auton_id = 18
auton_title = "Auton Two Ball E"

class Idle(SetIdle):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.setRobotPose()
        self.setIdle()

    def tick(self):
        return DeployIntake(self.ros_node)

class DeployIntake(Intake):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        return StartFirstPath(self.ros_node)

class StartFirstPath(StartPath):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(0)

    def tick(self):
        if self.check_timer(1):
            return Prime(self.ros_node)
        return self

class Prime(Shooter):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        #wait till ready to shoot
        if self.check_timer(1) and self.finished_path(0):
            return Shoot(self.ros_node)
        return self

class Shoot(Shooter):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if self.check_timer(1):
            self.start_shoot()
            self.idle()
            return StartSecondPath(self.ros_node)
        return self

class StartSecondPath(StartPath):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(1)

    def tick(self):
        if self.finished_path(1):
            return StartThirdPath(self.ros_node)
        return self

class StartThirdPath(StartPath):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_path(2)

    def tick(self):
        return Hide(self.ros_node)

class Hide(Shooter):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.hide_poop()

    def tick(self):
        if self.check_timer(1) and self.finished_path(2):
            return RetractIntake(self.ros_node)
        return self


class RetractIntake(Intake):
   
    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.retract_intake()

    def tick(self):
        return Final(self.ros_node)


class Final(State):

    def initialize(self):
        self.log_state()

    def execute_action(self):
        rospy.loginfo("END OF AUTON")

    def tick(self):
        return self

class Shutdown(SetIdle):

    def initialize(self):
        pass

    def execute_action(self):
        self.setIdle()

    def tick(self):
        return self

def start(ros_node):
    
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

    return Idle, Shutdown


