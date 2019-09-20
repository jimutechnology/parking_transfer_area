#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import rospy
from command import Command
from transfer_area.msg import CommandReply, StateCmd
from transfer_area.srv import getDoorState

class StateCommand(Command):
    def __init__(self):
        Command.__init__(self)
        #self.set_area_state_pub = self.Publisher('set_area_state', StateCmd, queue_size=10)
        #rospy.wait_for_service('get_area_state')

    def execute(self, cmd_dict):
        self.setState(CommandReply.STATE_RUNNING)
        duration = 30 * 1000
        self.setTimeout(duration)
        self.reply_result()
 
    def reply_result(self):
        door_state = self.get_doorstate()
        message_text = "area_state: " + str(1)
        message_text += "\n" + "door_state: " + str(door_state)
        self.message = str(message_text)
        self.Reply(state = CommandReply.STATE_FINISH)

    def get_doorstate(self):
        rospy.wait_for_service('door_state')
        try:
            state = rospy.ServiceProxy('door_state', getDoorState)
            res = state()
            return res.state
        except rospy.ServiceException, e:
            return [-1,-1]