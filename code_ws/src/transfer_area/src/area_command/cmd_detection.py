#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import copy
import threading
import math
from command import Command
from transfer_area.msg import CommandReply,MotorCmd
from car_scanner.msg import CarState
from geometry_msgs.msg import Pose2D
from area_common import AreaParam

class DetectionCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.lock_rx_car_state = threading.Lock()
        self.car_state_data = CarState()
        self.Subscriber("car_state", CarState, self.rx_car_state)

        self.motor_cmd = MotorCmd()
        self.motor_cmd_pub = self.Publisher('motor_cmd', MotorCmd, queue_size=10)
        self.car_scanner_successful = False

        self.param = AreaParam()
        self.pose_in_map = self.param.pose_in_map
        

    def rx_car_state(self, data):
        with self.lock_rx_car_state:
            self.car_state_data = copy.deepcopy(data)
            self.car_scanner_successful = True
            
    def execute(self, cmd_dict):
        self.setState(CommandReply.STATE_RUNNING)
        self.reply_result()

    def transform_error(self, state_data):
        if self.car_scanner_successful == False:
            return 0xFF
        error_code = 0
        error_code += state_data.is_position_left * 2 ** 0
        error_code += state_data.is_position_right * 2 ** 1
        error_code += state_data.is_position_front * 2 ** 2
        error_code += state_data.is_position_back * 2 ** 3
        error_code += state_data.is_yaw_left * 2 ** 4
        error_code += state_data.is_yaw_right * 2 ** 5
        error_code += state_data.is_steering_left * 2 ** 6
        error_code += state_data.is_steering_right * 2 ** 7
        error_code += state_data.is_wheelbase_short * 2 ** 8
        error_code += state_data.is_wheelbase_long * 2 ** 9
        error_code += state_data.is_wheel_distance_short * 2 ** 10
        error_code += state_data.is_wheel_distance_long * 2 ** 11
        return error_code
    
    def getScanPose(self, data):
        # points in local transferArea coordinate {Ti}
        p_local_orig = Pose2D(x=0.0, y=0.0, theta=0.0)
        p_local_tire = Pose2D(x=data.aX, y=data.aY, theta=data.aYaw)
        # calculate car head tires center from transferArea local coordinate to global coordinate
        p_global_orig = Pose2D(x=self.pose_in_map[0], y=self.pose_in_map[1], theta=self.pose_in_map[2])
        p_global_tire = Pose2D(
            x = p_global_orig.x - (p_local_tire.y - p_local_orig.y),
            y = p_global_orig.y + (p_local_tire.x - p_local_orig.x),
            theta = p_global_orig.theta + p_local_tire.theta
        )
        # calculate the scan point for robot in global coordinate
        p_global_scan = Pose2D(
            x = p_global_tire.x + self.param.access_y_offset * math.cos(p_global_tire.theta),
            y = p_global_tire.y + self.param.access_y_offset * math.sin(p_global_tire.theta),
            theta = p_global_tire.theta
        )
        print(p_local_orig)
        print(p_local_tire)
        print(p_global_orig)
        print(p_global_tire)
        print(p_global_scan)
        return p_global_scan

    def reply_result(self):
        if self.car_scanner_successful == True and self.car_state_data.is_car_available == True:
            pose_in_map = self.getScanPose(self.car_state_data)
            message_text = "result: " + str(self.car_state_data.is_car_available)
            message_text += "\n" + "scanX: " + str(pose_in_map.x)
            message_text += "\n" + "scanY: " + str(pose_in_map.y)
            message_text += "\n" + "scanYaw: " + str(pose_in_map.theta)
            self.message = message_text
            self.Reply(state = CommandReply.STATE_FINISH)

            self.motor_cmd.cmd = self.motor_cmd.CMD_DOWN
            self.motor_cmd_pub.publish(self.motor_cmd)
        else:
            message_text = "result: " + str(False)
            self.message = message_text
            self.Reply(state = CommandReply.STATE_ERROR, error_code=self.transform_error(self.car_state_data))
        self.car_scanner_successful = False
