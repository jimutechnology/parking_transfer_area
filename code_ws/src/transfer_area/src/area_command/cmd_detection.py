#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
from command import Command

from transfer_area.msg import CommandReply

class DetectionCommand(Command):
    def __init__(self):
        Command.__init__(self)
        #self.arm_cmd_pub = self.Publisher('arm_cmd', ArmCmd, queue_size=10)

    def execute(self, cmd_dict):
        self.setState(CommandReply.STATE_RUNNING)
        duration = 30 * 1000
        self.setTimeout(duration)
        self.reply_result()

    def reply_result(self):
        message_text = "result: " + str(0)
        #message_text += "\n" +"error_code: " + str(0)
        self.message = str(message_text)
        self.Reply(state = CommandReply.STATE_FINISH)
