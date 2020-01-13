#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
import rospy
import copy
from car_scanner.msg import CarState
from transfer_area.msg import CommandReply, StateCmd, UnderpanDetectionState, LightCurtainState, DoorCmd, InfoOut, DockState, Door, ScreenCmd
from transfer_area.srv import getAreaState, getDoorState, getGroundStatus

from area_common import ServiceNode, RunService, getHeader, running_state as RS, area_state as AS, screen_state as SS

PRIM_HZ = 2

class Detection_Car(ServiceNode):
    def __init__(self):
        self.lock_rx_car_state = threading.Lock()
        self.car_state_data = CarState()
        self.Subscriber("car_state", CarState, self.rx_car_state)
        self.car_scanner_successful = False
        self.dock_state = DockState()
        self.dock_status_pub = self.Publisher('dock_state', DockState, queue_size=1)

    def rx_car_state(self, data):
        with self.lock_rx_car_state:
            self.car_state_data = copy.deepcopy(data)
            self.car_scanner_successful = True
        
    def get_ground_status(self):
        try:
            rospy.wait_for_service('get_ground_status',timeout=0.1)
            state = rospy.ServiceProxy('get_ground_status', getGroundStatus)
            res = state()
            if res.is_ground_clear == True:
                self.dock_state.area_state = DockState.EMPTY
            else:
                self.dock_state.area_state = DockState.OCCUPY
        except:
            self.dock_state.area_state = DockState.ERROR

    def get_doorstate(self):
        self.dock_state.door_state = []
        try:
            rospy.wait_for_service('door_state',timeout=0.3)
            state = rospy.ServiceProxy('door_state', getDoorState)
            res = state()
            self.dock_state.door_state = copy.deepcopy(res.door_state)
        except rospy.ServiceException, e:
            return
    
    def execute(self):
        self.get_ground_status()
        self.get_doorstate()
        self.dock_state.header = copy.deepcopy(getHeader())
        self.dock_status_pub.publish(self.dock_state)
        if self.dock_state.area_state == DockState.EMPTY and self.car_scanner_successful == False:
            return False
        else:
            return True

class Detection_Sensor(ServiceNode):
    def __init__(self):
        # Subscribers
        self.UD_state_data = UnderpanDetectionState()
        self.UD_state_data.state = False
        self.lock_UD_state_rx = threading.Lock()
        self.Subscriber("underpan_detection_state", UnderpanDetectionState, self.rx_UD_state)
        
        self.LC_state_data = [False, False]  ## 0: 内 1: 外 False: 闭合 True: 断开
        self.lock_LC_state_rx = threading.Lock()
        self.Subscriber("light_curtain_state", LightCurtainState, self.rx_LC_state)

    def rx_UD_state(self, data):
        with self.lock_LC_state_rx:
            with self.lock_UD_state_rx:
                self.UD_state_data == data

    def rx_LC_state(self, data):
        with self.lock_UD_state_rx:
            with self.lock_LC_state_rx:
                if data.id == 0:
                    self.LC_state_data[0] = data.state
                elif data.id == 1:
                    self.LC_state_data[1] = data.state
    
    def execute(self):
        if self.UD_state_data.state == False and self.LC_state_data[0] == False and self.LC_state_data[1] == False:
            return False
        else:
            return True
        
class Area_Manager(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay
        self.area_state = AS['FREE']
        self.DS = Detection_Sensor()
        self.DC = Detection_Car()
        self.car_detection_state = False

        self.dock_state = DockState()
        self.dock_status_pub = self.Publisher('dock_state', DockState, queue_size=1)

        self.screen_state = ScreenCmd()
        self.last_screen_state = ScreenCmd()
        self.screen_state.id = 1
        self.last_screen_state.id = 1
        self.last_screen_state.state = 100
        self.screen_cmd_pub = self.Publisher('screen_cmd', ScreenCmd, queue_size=1)

    def loop(self):
        if self.DS.execute() == False and self.DC.execute() == False:
            self.screen_state.state = SS['FREE']
            self.car_detection_state = False
        elif self.car_detection_state == False:
            if self.DS.execute() == False and self.DC.execute() == True:
                if self.DC.car_scanner_successful == False:
                    self.screen_state.state = SS['LEAVE']
                elif self.DC.car_state_data.is_car_available == True:
                    self.screen_state.state = SS['OK']
                elif self.DC.car_state_data.is_car_available == False:
                    if self.DC.car_state_data.is_position_left == True:
                        self.screen_state.state = SS['LEFT']
                    if self.DC.car_state_data.is_position_right == True:
                        self.screen_state.state = SS['RIGHT']
                    if self.DC.car_state_data.is_position_front == True:
                        self.screen_state.state = SS['FORWARD']
                    if self.DC.car_state_data.is_position_back == True:
                        self.screen_state.state = SS['BACKWARD']
                    if self.DC.car_state_data.is_yaw_left == True or self.DC.car_state_data.is_yaw_right == True:
                        self.screen_state.state = SS['YAW']
                    if self.DC.car_state_data.is_steering_left == True or self.DC.car_state_data.is_steering_right == True:
                        self.screen_state.state = SS['STEERING']
                    if self.DC.car_state_data.is_wheelbase_short == True:
                        self.screen_state.state = SS['SHORT']
                    if self.DC.car_state_data.is_wheelbase_long == True:
                        self.screen_state.state = SS['LONG']
                    if self.DC.car_state_data.is_wheel_distance_short == True:
                        self.screen_state.state = SS['NARROW']
                    if self.DC.car_state_data.is_wheel_distance_long == True:
                        self.screen_state.state = SS['WIDE']
            else:
                if self.DS.LC_state_data[1] == True:
                    if self.DS.UD_state_data.state == True:
                        self.screen_state.state = SS['LOW']
                        self.car_detection_state = True
                    elif self.DS.LC_state_data[0] == True:
                        self.screen_state.state = SS['LONG']
                    else:
                        self.screen_state.state = SS['FORWARD']
                elif self.DS.LC_state_data[0] == True:
                    self.screen_state.state = SS['BACKWARD']
        if self.last_screen_state.state != self.screen_state.state:
            self.screen_state.header = copy.deepcopy(getHeader())
            self.screen_cmd_pub.publish(self.screen_state)
            self.last_screen_state.state = self.screen_state.state
        self.DC.car_scanner_successful = False


if __name__ == '__main__':
    time.sleep(5)
    RunService(Area_Manager)
