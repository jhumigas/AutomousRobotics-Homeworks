#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True

    def speed_steering_distribution(self, twist, drive_cfg):
        """
        Computing steering and radial speed from velocity if we 
        group wheels by pairs. 

        Args:
            twist(Twist): Twist command, converted for example from joystick
        
        Returns:
            Dictionnary for each wheel steering and speed
        """
        vx_r = twist.linear.x    
        vy_r = twist.linear.y + drive_cfg["RL"].x*twist.angular.z
        vx_c = vx_r                  
        vy_c = twist.linear.y + drive_cfg["CL"].x*twist.angular.z
        vx_f = vx_r
        vy_f = twist.linear.y + drive_cfg["FL"].x*twist.angular.z
        wheelcommand = {'steering':{}, 'speed':{}}
        wheelcommand['steering']['RL'] = atan2(vy_r,vx_r)
        wheelcommand['steering']['RR'] = atan2(vy_r,vx_r)
        wheelcommand['steering']['CL'] = atan2(vy_c,vx_c)
        wheelcommand['steering']['CR'] = atan2(vy_c,vx_c)
        wheelcommand['steering']['FL'] = atan2(vy_f,vx_f)
        wheelcommand['steering']['FR'] = atan2(vy_f,vx_f)
        wheelcommand['speed']['RL'] = 10*hypot(vy_r,vx_r)
        wheelcommand['speed']['RR'] = 10*hypot(vy_r,vx_r)
        wheelcommand['speed']['CL'] = 10*hypot(vy_c,vx_c)
        wheelcommand['speed']['CR'] = 10*hypot(vy_c,vx_c)
        wheelcommand['speed']['FL'] = 10*hypot(vy_f,vx_f)
        wheelcommand['speed']['FR'] = 10*hypot(vy_f,vx_f)
        return wheelcommand



    def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
        motors = RoverMotors()
        # comm = self.speed_steering_distribution(twist, drive_cfg)
        if skidsteer:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                vx = twist.linear.x - twist.angular.z*drive_cfg[k].y
                vy = twist.linear.y + twist.angular.z*drive_cfg[k].x
                # if -pi/2 <= atan2(vy, vx) <= pi/2:
                motors.steering[k] = atan2(vy, vx) # comm['steering'][k]
                motors.drive[k] = 10*hypot(vy, vx) # comm['speed'][k]
        else:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                vx = twist.linear.x - twist.angular.z*drive_cfg[k].y
                vy = twist.linear.y + twist.angular.z*drive_cfg[k].x
                if -pi/2 <= atan2(vy, vx) <= pi/2:
                    motors.steering[k] = atan2(vy, vx) # comm['steering'][k] # 
                    motors.drive[k] = 10*hypot(vy, vx)
                elif pi/2 < atan2(vy, vx) <= pi:
                    motors.steering[k] = atan2(vy, vx) + pi # comm['steering'][k] #
                    motors.drive[k] = -10*hypot(vy, vx)
                else:
                    motors.steering[k] = atan2(vy, vx) - pi
                    motors.drive[k] = -10*hypot(vy, vx)

                # motors.steering[k] = atan2(vy, vx) # comm['steering'][k] # 
                # motors.drive[k] = 10*hypot(vy, vx)

        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return self.X
        # Odometry code
        A_ = []
        B_ = []
        for k in drive_cfg.keys():
            s_k = 0
            beta_k = 0
            if abs(motor_state.drive[k] - self.motor_state.drive[k]) > 5e-5:
                s_k = motor_state.drive[k] - self.motor_state.drive[k]
            if abs(motor_state.steering[k] - self.motor_state.steering[k]) > 5e-3:
                beta_k = motor_state.steering[k] - self.motor_state.steering[k]
            A_.append([1, 0, -drive_cfg[k].y])
            B_.append([s_k*cos(beta_k)])
            A_.append([0, 1, drive_cfg[k].x])
            B_.append([s_k*sin(beta_k)])
        A = numpy.array(A_)
        B = numpy.array(B_)
        self.X += pinv(A).dot(B)
        # self.X[0,0] = 0.0
        # self.X[1,0] = 0.0
        # self.X[2,0] = 0.0

        self.motor_state.copy(motor_state)
        return self.X



