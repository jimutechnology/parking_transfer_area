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
        # pose_in_map: [2.59, 0.2, 1.57079633]
        # access_y_offset: 1.5
        p_delta = Pose2D(
            x = self.pose_in_map[0],
            y = self.pose_in_map[1],
            theta = self.pose_in_map[2])
        p_b = Pose2D(
            x = data.aX - self.param.access_y_offset * math.cos(data.aYaw),
            y = data.aY - self.param.access_y_offset * math.sin(data.aYaw),
            theta = data.aYaw)
        p_a = Pose2D()
        p_a.x = p_b.x * math.cos(p_delta.theta) - p_b.y * math.sin(p_delta.theta) + p_delta.x
        p_a.y = p_b.x * math.sin(p_delta.theta) + p_b.y * math.cos(p_delta.theta) + p_delta.y
        
        #print self.param.access_y_offset
        #p_a.y = p_a.y - self.param.access_y_offset * math.sin(p_b.theta)
        #p_a.x = p_a.x - self.param.access_y_offset * math.cos(p_b.theta)

        p_a.theta = p_b.theta + p_delta.theta
        print(p_delta)
        print(p_b)
        print(p_a)
        return p_a


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
