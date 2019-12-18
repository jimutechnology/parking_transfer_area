#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import copy
import threading

from command import Command
from transfer_area.msg import CommandReply
from car_scanner.msg import CarState

class DetectionCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.lock_rx_car_state = threading.Lock()
        self.car_state_data = CarState()
        self.Subscriber("car_state", CarState, self.rx_car_state)

        self.car_scanner_successful = False

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

    def reply_result(self):
        message_text = "result: " + str(self.car_state_data.is_car_available)
        if self.car_state_data.is_car_available == True and self.car_scanner_successful == True:
            message_text += "\n" + "scanX: " + str(self.car_state_data.aX)
            message_text += "\n" + "scanY: " + str(self.car_state_data.aY)
            message_text += "\n" + "scanYaw: " + str(self.car_state_data.aYaw)
            self.message = message_text
            self.Reply(state = CommandReply.STATE_FINISH)
        else:
            self.message = message_text
            self.Reply(state = CommandReply.STATE_ERROR, error_code=self.transform_error(self.car_state_data))
        self.car_scanner_successful == False
