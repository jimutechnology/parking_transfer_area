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

        action_str = self.get(cmd_dict, "action", 'open')
        action_val = DoorCmd.OPEN_DOOR
        if action_str == 'open':
            action_val = DoorCmd.OPEN_DOOR
        else:
            action_val = DoorCmd.CLOSE_DOOR
        doorCommand = DoorCmd(
            header      = self.getHeader(cmd_dict),
            door_id     = self.get(cmd_dict, "door_id", 0),
            action      = action_val
        )
        duration = 30 * 1000
        self.setTimeout(duration)
        self.message = str(doorCommand)
        self.setState(CommandReply.STATE_RUNNING)
        self.door_cmd_pub.publish(doorCommand)
