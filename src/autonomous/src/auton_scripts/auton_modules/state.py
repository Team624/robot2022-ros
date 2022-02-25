from datetime import datetime
import json
import rospy
from std_msgs.msg import Float32, Float64, Bool, String, Float32MultiArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from path import AutoGoal, AutoPath, Autons
import time
import rospkg 
import math

global data
data = []

def read_json():
    """ This reads the auton data and saves it to a list to be used """
    try:
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('autonomous') + "/src/auton_scripts/auton_modules/path-editor/data.txt"
        with open(file_path) as json_file:
            json_data = json.load(json_file)

            new_data = []
            for d in json_data:
                a = Autons(len(new_data))
                a.deserialize_json(d)
                new_data.append(a)

            global data
            data = new_data
    except:
        read_json()
        
read_json()

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.initialize()
        self.action_executed = False

        self.start_time = time.time()

    def log_state(self):
        """ Logs the name of the State """
        rospy.loginfo("STATE: %s   [%s]" %(self.__class__.__name__, 15 - self.ros_node.get_time()))
    
    # Counts the amount of time 
    def start_timer(self):
        """ Used to start a timer that counts time within a state """
        self.start_time = time.time()

    def check_timer(self, wanted_time):
        """ Checks if the amount of time given has passed """
        if time.time() - self.start_time >= wanted_time:
            return True
        return False

    # Function for working with wrapping angles
    def wrap_angle(self, angle):
        if angle < 0.0:
            return (math.pi * 2) + angle
        elif angle >= (math.pi * 2):
            return angle - (math.pi * 2)

        return angle
    
    # Get data
    def get_path(self):
        return self.ros_node.get_data("/pathTable/status/path")

    def get_point(self):
        return self.ros_node.get_data("/pathTable/status/point")

    def finished_path(self):
        return self.ros_node.get_data("/pathTable/status/finishedPath")

    # This runs in the child class when created
    def initialize(self):
        pass

    # This runs once in the child class
    def execute_action(self):
        pass

    # This runs in a loop in the child class
    def tick(self):
        pass

    # This makes it so that the functions created in the child class act as they should
    def update(self):
        if not self.action_executed:
            self.execute_action()
            self.action_executed = True
        return self.tick()

class SetIdle(State):

    def setRobotPose(self):
        global data
        msg = Float32MultiArray()
        for auton in data:
            if auton.title == self.ros_node.auton_title:
                msg.data = auton.start_pose
                self.ros_node.publish('/robot_set_pose', Float32MultiArray, msg, latching = True)
                rospy.loginfo("Reset Robot Pose")

    def setIdle(self):
        # Retract intake
        intake_state = String()
        intake_state.data = "retract"
        self.ros_node.publish("/auto/intake/state", String, intake_state, latching = True)
        rospy.loginfo("Retracted Intake")

        # Shooter idle
        shooter_state = String()
        shooter_state.data = "idle"
        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Idle")

        # Flywheel idle
        flywheel_state = String()
        flywheel_state.data = "idle"
        self.ros_node.publish("/auto/flywheel/state", String, flywheel_state, latching = True)
        rospy.loginfo("Flywheel Idle")

        # Hood idle
        hood_state = String()
        hood_state.data = "idle"
        self.ros_node.publish("/auto/hood/state", String, hood_state, latching = True)
        rospy.loginfo("Hood Idle")

        # Path Idle
        self.ros_node.publish("/pathTable/startPathIndex", Float32, -1, latching = True)



class StartPath(State):

    # Actions
    def start_path(self, index):
        """ This gets the path data from the json file and publishes to diff_drive """
        # Checks for updated data
        self.ros_node.publish("/pathTable/startPathIndex", Float32, index, latching = True)
        

# This is ROBOT SPECIFIC
class Intake(State):

    # Actions
    def deploy_intake(self):
        """ This publishes a msg to deploy the intake """
        intake_state = String()
        intake_state.data = "deploy"

        self.ros_node.publish("/auto/intake/state", String, intake_state, latching = True)
        rospy.loginfo("Deployed Intake")

    def retract_intake(self):
        """ This publishes a msg to retract the intake """
        intake_state = String()
        intake_state.data = "retract"

        self.ros_node.publish("/auto/intake/state", String, intake_state, latching = True)
        rospy.loginfo("Retracted Intake")

class Shooter(State):

    # This puts the shooter in idle mode and allows other sub systems to do specific functions
    def idle(self):
        """ This puts the shooter in idle mode """
        shooter_state = String()
        shooter_state.data = "idle"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Idle")
    
    # Overrides the other states because it needs to control all three subsystems 
    def start_hide(self):
        """ This starts the turret tracking, adjusting rpm, and hood angle """
        shooter_state = String()
        shooter_state.data = "hide"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Hide")

    def start_hide_shoot(self):
        """ This starts the turret tracking, adjusting rpm, and hood angle """
        shooter_state = String()
        shooter_state.data = "hide_shoot"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Hide Shoot")

    def start_prime(self):
        """ This starts the turret tracking, adjusting rpm, and hood angle """
        shooter_state = String()
        shooter_state.data = "prime"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Prime")

    def start_shoot(self):
        """ This makes the turret begin to shoot """
        shooter_state = String()
        shooter_state.data = "shoot"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
        rospy.loginfo("Shooter Shooting")

class Flywheel(Shooter):

    # Conditions
    def reached_rpm(self, rpm):
        """ Checks if the fly wheel has reached the wanted rpm """
        if self.ros_node.get_data("/auto/flywheel/current/rpm") == rpm:
            return True
        return False

    # Actions (Only works if Shooter is in idle)
    def idle_flywheel(self):
        """ This puts the shooter into idle mode """
        flywheel_state = String()
        flywheel_state.data = "idle"

        self.ros_node.publish("/auto/flywheel/state", String, flywheel_state, latching = True)
        rospy.loginfo("Flywheel Idle")

    def start_spin_up(self, rpm):
        """ This starts the robot's spin up to a specific rpm """
        flywheel_state = String()
        flywheel_state.data = "spin_up"

        flywheel_rpm = Float32()
        flywheel_rpm.data = rpm

        self.ros_node.publish("/auto/flywheel/state", String, flywheel_state, latching = True)
        self.ros_node.publish("/auto/flywheel/wanted/rpm", Float32, flywheel_rpm, latching = True)
        rospy.loginfo("Flywheel Spinup")

class Hood(Shooter):

    # Actions (Only works if Shooter is in idle)
    def idle_hood(self):
        """ This puts the hood into idle mode """
        hood_state = String()
        hood_state.data = "idle"

        self.ros_node.publish("/auto/hood/state", String, hood_state, latching = True)
        rospy.loginfo("Hood Idle")

    def actuate_hood(self, angle):
        """ This adjusts the hood to the given angle """
        hood_state = String()
        hood_state.data = "actuate"

        self.ros_node.publish("/auto/hood/state", String, hood_state, latching = True)
        rospy.loginfo("Hood Actuate")
