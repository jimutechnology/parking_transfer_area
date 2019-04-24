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
        duration = 30 * 1000
        self.setTimeout(duration)

