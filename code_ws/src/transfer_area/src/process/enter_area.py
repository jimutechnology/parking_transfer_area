#!/usr/bin/env python

import threading

from transfer_area.msg import CommandReply, StateCmd
from transfer_area.srv import getAreaState

from area_common import ServiceNode, RunService, running_state as RS

PRIM_HZ = 2

class EnterArea(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay
        self.state = RS['STANDBY']
       
        # Publishers
        self.reply_data = CommandReply()
        self.reply_data_last = CommandReply()

        self.reply_pub = self.Publisher('cmd_reply', CommandReply, queue_size=1)

        # Subscribers
        self.state_cmd_data = StateCmd()
        self.lock_cmd_state = threading.Lock()
        self.Subscriber("set_area_state", StateCmd, self.rx_state_cmd)

        self.lock_src_state = threading.Lock()
        self.Service("get_area_state", getAreaState, self.return_state)
        self.area_state = self.state_cmd_data.state

    def rx_state_cmd(self, data):
        with self.lock_cmd_state:
            if self.state == RS['STANDBY']:
                self.state_cmd_data = data
                self.reply_data.command_id = data.header.frame_id
                self.reply_data.state = CommandReply.STATE_RUNNING
                self.state = RS['RUNNING']
    
    def return_state(self, data):
        return self.area_state

    def update_cmd_reply(self):
        if self.reply_data_last.state != self.reply_data.state and self.reply_data.command_id != '':
            self.reply_data_last.state = self.reply_data.state
            self.reply_pub.publish(self.reply_data)

    def loop(self):
        self.update_cmd_reply()
        if self.state == RS['RUNNING']:
            self.area_state = self.state_cmd_data.state
            self.state = RS['STANDBY']
            self.reply_data.state = CommandReply.STATE_FINISH
            self.state = RS['STANDBY']

if __name__ == '__main__':
    RunService(EnterArea)