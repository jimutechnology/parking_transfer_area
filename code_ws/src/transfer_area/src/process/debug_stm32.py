#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
import rospy
from area_common import ServiceNode, RunService
from transfer_area.msg import Door, DoorCmd, CommandReply
from transfer_area.srv import getDoorState, getDoorStateResponse

PRIM_HZ = 2

class DebugStm32(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay

        self.door_state=[Door(),Door()]
        self.door_state[0].id=Door.OUTSIDE
        self.door_state[1].id=Door.INSIDE

        self.door_cmd_reply_data = CommandReply()
        self.door_cmd_reply_data.state = CommandReply.STATE_EMPTY
        self.door_cmd_data = DoorCmd()
        self.lock_door_cmd_data_rx = threading.Lock()
        self.Subscriber("door_cmd", DoorCmd, self.rx_door_cmd)

        self.reply_pub = self.Publisher('cmd_reply', CommandReply, queue_size=5)

        s = rospy.Service('door_state', getDoorState, self.check_state)

    def rx_door_cmd(self, data):
        with self.lock_door_cmd_data_rx:
            if self.door_cmd_reply_data.state == CommandReply.STATE_RUNNING:
                reply_data = CommandReply()
                reply_data.state = CommandReply.STATE_ERROR
                reply_data.command_id = data.header.frame_id
                self.reply_pub.publish(reply_data)
                return
            self.door_cmd_data = data
            self.door_cmd_reply_data.state = CommandReply.STATE_RUNNING
            self.door_cmd_reply_data.command_id = data.header.frame_id
            self.reply_pub.publish(self.door_cmd_reply_data)
    
    def execute_door_cmd(self):
        cmd = self.door_cmd_data.door
        for i in range(len(cmd)):
            if cmd[i].id < 3:
                for j in range(len(self.door_state)):
                    if self.door_state[j].id == cmd[i].id and self.door_state[j].position != cmd[i].position:
                        time.sleep(5)
                        self.door_state[j].position = cmd[i].position
                        break
        self.door_cmd_reply_data.state = CommandReply.STATE_FINISH
        self.reply_pub.publish(self.door_cmd_reply_data)

    def check_state(self, req):
        return getDoorStateResponse(self.door_state)
            
    def loop(self):
        if self.door_cmd_reply_data.state == CommandReply.STATE_RUNNING:
            self.execute_door_cmd()

if __name__ == '__main__':
    RunService(DebugStm32)
