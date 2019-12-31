#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import copy
from command import Command
from transfer_area.msg import Door, DoorCmd, CommandReply

class DoorCommand(Command):
    def __init__(self):
        Command.__init__(self)
        self.door_cmd_pub = self.Publisher('door_cmd', DoorCmd, queue_size=10)

    def execute(self, cmd_dict):
        duration = 30.0 * 1000
        self.setTimeout(duration)
        doorCommand = DoorCmd(
            header = self.getHeader(cmd_dict)
        )

        opens = self.get(cmd_dict, "door", [])
        for v in opens:
            if 'id' in v and 'position' in v:
                door = Door()
                door.id = v['id']
                door.position = v['position']
                doorCommand.door.append(door)

        if doorCommand.door:
            self.door_cmd_pub.publish(doorCommand)
