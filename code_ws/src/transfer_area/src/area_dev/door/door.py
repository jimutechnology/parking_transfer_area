#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import rospy

from area_common import ServiceNode, RunService, running_state as RS
from transfer_area.msg import DoorCmd, CommandReply

class DoorNode(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(2)

        self.state = RS['STANDBY']
        self.loop_cnt = 0
        # Publishers
        self.reply_data = CommandReply()
        self.reply_data_last = CommandReply()

        self.reply_pub = self.Publisher('cmd_reply', CommandReply, queue_size=1)

        # Subscribers
        self.door_cmd_data = DoorCmd()

        self.Subscriber("door_cmd", DoorCmd, self.door_cmd_rx)

        self.lock_door_cmd_data = threading.RLock()

    def door_cmd_rx(self, data):
        with self.lock_door_cmd_data:
            if self.state == RS['STANDBY']:
                self.door_cmd_data = data
                self.reply_data.command_id = data.header.frame_id
                self.reply_data.state = CommandReply.STATE_RUNNING
                self.state = RS['RUNNING']
                print "door cmd rx"


    def update_cmd_reply(self):
        if self.reply_data_last.state != self.reply_data.state and self.reply_data.command_id != '':
            self.reply_data_last.state = self.reply_data.state
            self.reply_pub.publish(self.reply_data)

    def loop(self):
        if self.state == RS['RUNNING']:
            if self.loop_cnt < 10:
                self.loop_cnt += 1
            else:
                self.loop_cnt = 0
                self.reply_data.state = CommandReply.STATE_FINISH
                self.state = RS['STANDBY']
        self.update_cmd_reply()


if __name__ == '__main__':
    RunService(DoorNode)
