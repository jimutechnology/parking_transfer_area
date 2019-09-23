#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time

from command import Command
from transfer_area.msg import DoorCmd, CommandReply

class DoorCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.door_cmd_pub = self.Publisher('door_cmd', DoorCmd, queue_size=10)

    def execute(self, cmd_dict):
        doorCommand = DoorCmd(
            header      = self.getHeader(cmd_dict),
            id          = self.get(cmd_dict, "door_id", 0),
            action      = self.get(cmd_dict, "action", 0)
        )
        duration = 30 * 1000
        self.setTimeout(duration)
        self.setState(CommandReply.STATE_RUNNING)
        self.door_cmd_pub.publish(doorCommand)
