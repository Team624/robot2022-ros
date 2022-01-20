"""
Path tracking simulation with Stanley steering control and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import cubic_spline_planner
from diff_drive.pose import Pose
import math

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, wheel_base=0.5842):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.wheel_base = wheel_base
        self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw))

    def update(self, x, y, yaw, v, is_backward = 1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        if is_backward == -1:
            self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw-math.pi))
            self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw-math.pi))
        else:
            self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw))
            self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class StanleySteering:
    def __init__(self, ax = [0, 1], ay = [0, 1], k = 0.1, Kp = 1.0, dt = 0.5, Ka = 0.4, L = 0.5842, look_ahead = 0.1):
        self.k = k  # control gain
        self.Kp = Kp  # speed proportional gain
        self.Ka = Ka
        self.wheel_base = L  # [m] Wheel base of vehicle

        self.start_th = 0

        self.look_ahead = look_ahead

        #  target course
        self.cx = ax
        self.cy = ay

        # self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        #     self.ax, self.ay, ds=0.1)

        self.target_course = None

        # Initial state
        self.state = State()

        self.last_idx = len(self.cx) - 1
        
        self.target_idx = 0

        self.is_at_goal = False
        self.direction = 1

        # Parameters
        self.max_linear_speed = 1.1
        self.min_linear_speed = 0.1
        self.max_angular_speed = 2.0
        self.min_angular_speed = 1.0
        self.max_linear_acceleration = 1E9
        self.max_angular_acceleration = 1E9
        self.linear_tolerance_outer = 0.3
        self.linear_tolerance_inner = 0.1
        self.angular_tolerance_outer = 0.2
        self.angular_tolerance_inner = 0.1
        self.ignore_angular_tolerance = False
        self.forward_movement_only = False
        self.end_of_path_stop = True
        self.within_linear_tolerance = False
        self.within_angular_tolerance = False

        self.test_alpha = 0
        self.test_delta = 0
        self.test_theta_vel = 0
        self.test_robot_yaw = 0
        self.test_goal_yaw = 0

    def set_path(self, pose, vel, ax = [0,1], ay = [0,1]):
        self.is_at_goal = False
        self.state.update(pose.x,pose.y,pose.theta,vel)
        self.start_th = pose.theta

        self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=0.1)

        #  target course
        # self.cx = ax
        # self.cy = ay

        # self.cx.append(self.cx[len(self.cx)-1])
        # self.cy.append(self.cy[len(self.cy)-1])
        # self.cyaw.append(self.cyaw[len(self.cyaw)-1]

        self.last_idx = len(self.cx) - 1

        #  target course
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_idx, _, _ = self.target_course.search_target_index(self.state, self.k, self.linear_tolerance_outer, self.look_ahead)
    # Parameters
    def set_constants(self, Kp, k, Ka):
        self.Kp = Kp
        self.Ka = Ka
        self.k = k

    def set_max_linear_speed(self, speed):
        self.max_linear_speed = speed

    def set_min_linear_speed(self, speed):
        self.min_linear_speed = speed

    def set_max_angular_speed(self, speed):
        self.max_angular_speed = speed

    def set_min_angular_speed(self, speed):
        self.min_angular_speed = speed

    def set_max_linear_acceleration(self, accel):
        self.max_linear_acceleration = accel

    def set_max_angular_acceleration(self, accel):
        self.max_angular_acceleration = accel

    def set_linear_tolerance_outer(self, tolerance):
        self.linear_tolerance_outer = tolerance
    
    def set_linear_tolerance_inner(self, tolerance):
        self.linear_tolerance_inner = tolerance

    def set_angular_tolerance_outer(self, tolerance):
        self.angular_tolerance_outer = tolerance

    def set_angular_tolerance_inner(self, tolerance):
        self.angular_tolerance_inner = tolerance
    
    def set_ignore_angular_tolerance(self, ignore):
        self.ignore_angular_tolerance = ignore

    def set_forward_movement_only(self, forward_only):
        self.forward_movement_only = forward_only
        if (forward_only):
            self.direction = 1
        else:
            self.direction = -1
            self.max_linear_speed = -self.max_linear_speed

    def set_end_of_path_stop(self, end_of_path_stop):
        self.end_of_path_stop = end_of_path_stop
    
    def reset_within_tolerance(self):
        self.within_linear_tolerance = False
        self.within_angular_tolerance = False

    def reset(self):
        if self.end_of_path_stop:
            return True
        return False

    def get_goal_distance(self):
        diffX = self.state.x - self.cx[len(self.cx) -1]
        diffY = self.state.y - self.cy[len(self.cy) -1]
        return math.sqrt(diffX*diffX + diffY*diffY)

    def at_goal(self):
        return self.is_at_goal

    def pid_control(self, target, current):
        """
        Proportional control for the speed.
        :param target: (float)
        :param current: (float)
        :return: (float)
        """

        return self.Kp * (target - current)

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        angle = angle % (math.pi*2)
        angle = (angle + 2* math.pi)% (math.pi*2)

        if angle > math.pi:
            angle -= math.pi * 2

        return angle

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        ind, Lf, self.is_at_goal = trajectory.search_target_index(state, self.k, self.linear_tolerance_outer, self.look_ahead)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
        # How much it needs to turn in radians
        #if self.direction == -1 and ((abs(self.start_th) > math.pi/2.0 and abs(self.state.yaw) < math.pi/2.0) or (abs(self.start_th) < math.pi/2.0 and abs(self.state.yaw) > math.pi/2.0)):
        if (self.direction == -1):
            alpha = self.normalize_angle(self.normalize_angle(math.atan2(ty - state.rear_y, tx - state.rear_x)) - self.normalize_angle(state.yaw - math.pi))
            self.test_robot_yaw = tx
            self.test_goal_yaw = ty
        else:
            alpha = self.normalize_angle(self.normalize_angle(math.atan2(ty - state.rear_y, tx - state.rear_x)) - self.normalize_angle(state.yaw))
            self.test_robot_yaw = tx
            self.test_goal_yaw = ty

        delta = math.atan2(2.0 * self.wheel_base * math.sin(alpha) / self.Ka, 1.0)

        self.test_delta = delta
        self.test_alpha = alpha


        return delta, ind

    def get_velocity(self, pose, vel):
        self.state.update(pose.x,pose.y,pose.theta,vel, is_backward = self.direction)

        desired = Pose()
        if (not self.is_at_goal):
            # Calc control input
            ai = self.pid_control(self.max_linear_speed, self.state.v)
            di, self.target_idx = self.pure_pursuit_steer_control(
                self.state, self.target_course, self.target_idx)

            desired.xVel = self.state.v + ai

    
            desired.thetaVel = abs(self.state.v) / self.wheel_base * np.tan(di)
            self.test_theta_vel = abs(self.state.v) / self.wheel_base * np.tan(di)
        else:
            if (self.ignore_angular_tolerance):
                ai = self.pid_control(self.max_linear_speed, self.state.v)
                desired.xVel = self.state.v + ai

        return desired

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state, k, linear_tol, look_ahead):

        # To speed up nearest point search, doing it at only first time.
        print("old", self.old_nearest_point_index)
        if self.old_nearest_point_index is None:
            ind = 0
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            if (ind < len(self.cx)-1):
                distance_this_index = state.calc_distance(self.cx[ind],
                                                        self.cy[ind])
                while True:
                    distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                            self.cy[ind + 1])
                    if distance_this_index < distance_next_index:
                        break
                    ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                    distance_this_index = distance_next_index
                    if (ind + 1) >= len(self.cx):
                        break  # not exceed goal
                self.old_nearest_point_index = ind
            else:
                ind = len(self.cx)-1

        Lf = k * abs(state.v) + look_ahead  # update look ahead distance

        #search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        if ind >= len(self.cx):
            ind = len(self.cx) -1

        if state.calc_distance(self.cx[len(self.cx)-1], self.cy[len(self.cx)-1]) < linear_tol and ind >= len(self.cx)/2.0:
            is_at_goal = True
        else:
            is_at_goal = False

        self.old_nearest_point_index = ind
        return ind, Lf, is_at_goal

