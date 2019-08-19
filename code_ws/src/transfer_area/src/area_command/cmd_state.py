#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import rospy
from command import Command
from transfer_area.msg import CommandReply, StateCmd
from transfer_area.srv import getAraeState

class StateCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.set_area_state_pub = self.Publisher('set_area_state', StateCmd, queue_size=10)
        rospy.wait_for_service('get_area_state')

    def execute(self, cmd_dict):
        self.setState(CommandReply.STATE_RUNNING)
        operate = self.get(cmd_dict, "operate", 'GET')
        if operate == 'GET':
            self.get_state()
        else:
            stateCommand = StateCmd(
                header      = self.getHeader(cmd_dict),
                state       = self.get(cmd_dict, "area_state", 1)
            )
            duration = 30 * 1000
            self.setTimeout(duration)
            self.set_area_state_pub.publish(stateCommand)

    def get_state(self):
        try:
            get_area_state = rospy.ServiceProxy('get_area_state', getAraeState)
            res = get_area_state()
            self.message = "area_state: " + str(res.state)
            self.Reply(state = CommandReply.STATE_FINISH)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e