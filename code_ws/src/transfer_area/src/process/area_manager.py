#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
import rospy
import copy
from transfer_area.msg import CommandReply, StateCmd, UnderpanDetectionState, LightCurtainState, DoorCmd, InfoOut, DockState, Door, ScreenCmd
from transfer_area.srv import getAreaState, getDoorState, getGroundStatus

from area_common import ServiceNode, RunService, getHeader, running_state as RS, area_state as AS, screen_state as SS

PRIM_HZ = 2

class Status_Manager(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay
        self.area_state = AS['FREE']

        self.lock_loop_data = threading.Lock()
        self.forward_start_time = ''

        # Publishers
        self.error_out_data = InfoOut()
        self.warning_pub = self.Publisher('warning_info', InfoOut, queue_size=1)

        self.dock_state = DockState()
        self.dock_status_pub = self.Publisher('dock_state', DockState, queue_size=1)

        self.screen_state = ScreenCmd()
        self.screen_state.id = 1
        self.screen_state.state = SS['空闲']
        self.screen_cmd_pub = self.Publisher('screen_cmd', ScreenCmd, queue_size=1)

        # Subscribers
        self.UD_state_data = UnderpanDetectionState()
        self.lock_UD_state_rx = threading.Lock()
        self.Subscriber("underpan_detection_state", UnderpanDetectionState, self.rx_UD_state)
        
        self.LC_state_data = LightCurtainState()
        self.lock_LC_state_rx = threading.Lock()
        self.Subscriber("light_curtain_state", LightCurtainState, self.rx_LC_state)

        self.door_cmd_data = DoorCmd()
        self.lock_door_cmd_data_rx = threading.Lock()
        self.Subscriber("door_cmd", DoorCmd, self.rx_door_cmd)

    def rx_UD_state(self, data):
        with self.lock_LC_state_rx:
            with self.lock_UD_state_rx:
                self.UD_state_data == data
                if self.area_state == AS['FORWARD'] and self.UD_state_data.state == False:
                    self.area_state = AS['QUIT']
                    self.error_out_data.error_code = 0
                    self.error_out_data.message = "底盘高度不合格, 请退出"
                    self.warning_pub.publish(self.error_out_data)
                    self.screen_state.state = SS['底盘']
                    self.screen_cmd_pub.publish(self.screen_state)
                    

    def rx_LC_state(self, data):
        with self.lock_UD_state_rx:
            with self.lock_LC_state_rx:
                self.LC_state_data = data
                if self.area_state == AS['FREE']:
                    if self.LC_state_data.id == 1 and self.LC_state_data.state == True: #外光幕断开持续3秒
                        if self.forward_start_time == '':
                            self.forward_start_time = time.time()
                    else:
                        self.forward_start_time = ''
                elif self.area_state == AS['FORWARD']:
                    if self.LC_state_data.id == 1 and self.LC_state_data.state == False: #外光幕闭合
                        self.area_state = AS['FINISH']
                        self.error_out_data.error_code = 0
                        self.error_out_data.message = "车辆合格，请下车"
                        self.warning_pub.publish(self.error_out_data)
                        self.screen_state.state = SS['OK']
                        self.screen_cmd_pub.publish(self.screen_state)
                    elif self.LC_state_data.id == 0 and self.LC_state_data.state == True: #内光幕断开
                        self.area_state = AS['QUIT']
                        self.error_out_data.error_code = 0
                        self.error_out_data.message = "车辆超长，请退出"
                        self.warning_pub.publish(self.error_out_data)
                        self.screen_state.state = SS['超长']
                        self.screen_cmd_pub.publish(self.screen_state)
                elif self.area_state == AS['BACK']:
                    if self.LC_state_data.id == 0 and self.LC_state_data.state == False: #内光幕闭合
                        self.area_state = AS['FINISH']
                        self.error_out_data.error_code = 0
                        self.error_out_data.message = "车辆合格，请下车"
                        self.warning_pub.publish(self.error_out_data)
                        self.screen_state.state = SS['OK']
                        self.screen_cmd_pub.publish(self.screen_state)
                elif self.area_state == AS['FINISH']:
                    if self.LC_state_data.id == 1 and self.LC_state_data.state == True: #外光幕断开
                        self.area_state = AS['FORWARD']
                        self.error_out_data.error_code = 0
                        self.error_out_data.message = "请前进"
                        self.warning_pub.publish(self.error_out_data)
                        self.screen_state.state = SS['前进']
                        self.screen_cmd_pub.publish(self.screen_state)
                    elif self.LC_state_data.id == 0 and self.LC_state_data.state == True: #内光幕断开
                        self.area_state = AS['BACK']
                        self.error_out_data.error_code = 0
                        self.error_out_data.message = "请后退"
                        self.warning_pub.publish(self.error_out_data)
                        self.screen_state.state = SS['超长']
                        self.screen_cmd_pub.publish(self.screen_state)
                
    def rx_door_cmd(self, data):
        with self.lock_door_cmd_data_rx:
            self.door_cmd_data = data
            opens = self.door_cmd_data.door
            for i in range(len(opens)):
                if opens[i].id == Door.OUTSIDE and opens[i].position == Door.CLOSE and self.area_state == AS['FINISH']:
                    self.area_state == AS['RUNNING']
                    self.error_out_data.error_code = 0
                    self.error_out_data.message = "存车中"
                    self.warning_pub.publish(self.error_out_data)
                    break

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


    def loop(self):
        with self.lock_loop_data:
            self.get_doorstate()
            self.get_ground_status()
            self.dock_state.header = copy.deepcopy(getHeader())
            self.dock_status_pub.publish(self.dock_state)
            if self.area_state == AS['QUIT'] and self.area_state == AS['RUNNING']:
                if self.get_ground_status == True:
                    self.area_state == AS['FREE']
                    self.error_out_data.error_code = 0
                    self.error_out_data.message = "车库空闲"
                    self.warning_pub.publish(self.error_out_data)
                    self.screen_state.state = SS['空闲']
                    self.screen_cmd_pub.publish(self.screen_state)
                    
            if self.area_state == AS['FREE'] and self.forward_start_time != '':
                if time.time() - self.forward_start_time > 3.0:
                    self.area_state = AS['FORWARD']
                    self.forward_start_time = ''
                    self.error_out_data.error_code = 0
                    self.error_out_data.message = "请前进"
                    self.warning_pub.publish(self.error_out_data)
                    self.screen_state.state = SS['前进']
                    self.screen_cmd_pub.publish(self.screen_state)
                    




if __name__ == '__main__':
    time.sleep(5)
    RunService(Status_Manager)
