# import tf
# from std_msgs.msg import Float32, Float64, Bool
# from geometry_msgs.msg import Pose
# #from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from datetime import datetime
import json

class Autons:

    def __init__(self, auto_id, title="N/A", num_balls="N/A", start_pose=[0,0,0], description="N/A", paths=[], date_created=str(datetime.utcnow())):
        # Header Info
        self.id = auto_id
        self.title = title
        self.num_balls = num_balls
        self.start_pose = start_pose
        self.description = description
        self.date_created = date_created[0:date_created.index(' ')]
        
        # Main data
        self.paths = paths

    def serialize_json(self):
        """ This formats data to send to the client with json """
        json_string = json.dumps(self.__dict__, default=str)
        json_data = json.loads(json_string)

        # Loops through all the paths and coverts to json
        a = []
        for path in self.paths:
            json_string_path = json.dumps(path.__dict__, default=str)
            json_data_path = json.loads(json_string_path)

            # Loops through all the goals within the path and converts them
            b = []
            for goal in path.goals:
                json_string_goal = json.dumps(goal.__dict__, default=str)
                json_data_goal = json.loads(json_string_goal)
                b.append(json_data_goal)

            json_data_path["goals"] = b
            a.append(json_data_path)

        json_data["paths"] = a
        return json_data

    def deserialize_json(self, json_data):
        """ This formats data to be saved to this server """
        self.id = json_data["id"] 
        self.title = json_data["title"]
        self.num_balls = json_data["num_balls"]
        self.start_pose = json_data["start_pose"]
        self.description = json_data["description"]
        self.date_created = json_data["date_created"]

        # Loop through all the paths and update the class
        a = []
        for path in json_data["paths"]:
            b = []
            for goal in path["goals"]:
                c = AutoGoal(goal["x"], goal["y"], goal["t"])
                b.append(c)
            d = AutoPath(b, len(a), path["name"], path["control_points"], forward_movement_only=path["forward_movement_only"], end_of_path_stop=path["end_of_path_stop"])

            d.set_constants(kP=path["constants_kP"], kA=path["constants_kA"], kB=path["constants_kB"])
            d.set_linear(max_lin_speed=path["max_linear_speed"], min_lin_speed=path["min_linear_speed"], max_lin_accel=path["max_linear_acceleration"], lin_tolerance_outer=path["linear_tolerance_outer"], lin_tolerance_inner=path["linear_tolerance_inner"])
            d.set_angular(max_ang_speed=path["max_angular_speed"], min_ang_speed=path["min_angular_speed"], max_ang_accel=path["max_angular_acceleration"], ang_tolerance_outer=path["angular_tolerance_outer"], ang_tolerance_inner=path["angular_tolerance_inner"], ignore_ang_tolerance=path["ignore_angular_tolerance"])
            a.append(d)
            
        self.paths = a

class AutoPath:

    def __init__(self, goals, path_id, name, control_points, forward_movement_only = False, end_of_path_stop = True):
        # Returns a tuple converts to list
        self.id = path_id
        self.name = name

        self.constants_kP = 1.0
        self.constants_kA = 6.0
        self.constants_kB = -0.8

        self.max_linear_speed = 1.1
        self.min_linear_speed = 0.1
        self.max_linear_acceleration = 1E9
        self.linear_tolerance_outer = 0.3
        self.linear_tolerance_inner = 0.1

        self.max_angular_speed = 2.0
        self.min_angular_speed = 1.0
        self.max_angular_acceleration = 1E9
        self.angular_tolerance_outer = 0.2
        self.angular_tolerance_inner = 0.1
        self.ignore_angular_tolerance = False
        
        self.forward_movement_only = forward_movement_only
        self.end_of_path_stop = end_of_path_stop

        # An array of 4 points that control the curve
        self.control_points = control_points

        self.goals = goals

    def set_constants(self, kP = 1.0, kA = 6.0, kB = -0.8):
        self.constants_kP = kP
        self.constants_kA = kA
        self.constants_kB = kB

    def set_linear(self, max_lin_speed = 1.1, min_lin_speed = 0.1, max_lin_accel = 1E9, lin_tolerance_outer = 0.3, lin_tolerance_inner = 0.1):
        self.max_linear_speed = max_lin_speed
        self.min_linear_speed = min_lin_speed
        self.max_linear_acceleration = max_lin_accel
        self.linear_tolerance_outer = lin_tolerance_outer
        self.linear_tolerance_inner = lin_tolerance_inner

    def set_angular(self, max_ang_speed = 2.0, min_ang_speed = 1.0, max_ang_accel = 1E9, ang_tolerance_outer = 0.2, ang_tolerance_inner = 0.1, ignore_ang_tolerance = False):
        self.max_angular_speed = max_ang_speed
        self.min_angular_speed = min_ang_speed
        self.max_angular_acceleration = max_ang_accel
        self.angular_tolerance_outer = ang_tolerance_outer
        self.angular_tolerance_inner = ang_tolerance_inner
        self.ignore_angular_tolerance = ignore_ang_tolerance

    # def get_path(self):
    #     # Put list into msg
    #     goal_path = GoalPath()
    #     goal_path.goals = self.goals

    #     constants = Constants()
    #     constants.kP = self.constants_kP
    #     constants.kA = self.constants_kA
    #     constants.kB = self.constants_kB

    #     linear = Linear()
    #     linear.max_linear_speed = self.max_linear_speed
    #     linear.min_linear_speed = self.min_linear_speed
    #     linear.max_linear_acceleration = self.max_linear_acceleration
    #     linear.linear_tolerance_outer = self.linear_tolerance_outer
    #     linear.linear_tolerance_inner = self.linear_tolerance_inner

    #     angular = Angular()
    #     angular.max_angular_speed = self.max_angular_speed
    #     angular.min_angular_speed = self.min_angular_speed
    #     angular.max_angular_acceleration = self.max_angular_acceleration
    #     angular.angular_tolerance_outer = self.angular_tolerance_outer
    #     angular.angular_tolerance_inner = self.angular_tolerance_inner
    #     angular.ignore_angular_tolerance = self.ignore_angular_tolerance
        
    #     goal_path.forward_movement_only = self.forward_movement_only

    #     goal_path.end_of_path_stop = self.end_of_path_stop

    #     goal_path.constants = constants
    #     goal_path.linear = linear
    #     goal_path.angular = angular

    #     return goal_path

class AutoGoal:

    def __init__(self, x, y, t):
        self.x = x
        self.y = y

        self.t = t

    # def get_goal(self):
    #     goal = Goal()

    #     pose = Pose()
    #     pose.position.x = self.position_x
    #     pose.position.y = self.position_y

    #     quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th, 'sxyz')

    #     pose.orientation.z = quaternion[2]
    #     pose.orientation.w = quaternion[3]

    #     goal.pose = pose
    #     return goal