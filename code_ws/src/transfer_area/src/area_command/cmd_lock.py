#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import copy
from command import Command
from transfer_area.msg import MotorLock, CommandReply

class LockCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.motor_lock_cmd_pub = self.Publisher('motor_lock', MotorLock, queue_size=10)

    def execute(self, cmd_dict):
        motor_lock_data = MotorLock()
        motor_lock_data.enable_lock = cmd_dict['enable']
        self.motor_lock_cmd_pub.publish(motor_lock_data)
        self.reply_result()

    def reply_result(self):
            ret = True
            message_text = "result: " + str(ret)
            # message_text += "\n" + "scanX: 0.0" 
            # message_text += "\n" + "scanY: 0.0"
            # message_text += "\n" + "scanYaw: 0.0"
            self.message = message_text
            self.Reply(state = CommandReply.STATE_FINISH)