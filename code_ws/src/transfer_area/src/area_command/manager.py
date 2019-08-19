#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import time
from cmd_door  import DoorCommand
from cmd_detection  import DetectionCommand
from cmd_state  import StateCommand

from transfer_area.msg import CommandReply

class CommandManger:
    def __init__(self):
        self.cmds = {
            "DOOR"      : DoorCommand(),
            "STATE"     : StateCommand(),
            "DETECTION" : DetectionCommand(),

        }

    def execute(self, pydict):
        cmd_key = str(pydict["cmd"].upper())

        if cmd_key in self.cmds:
            cmd = self.cmds[cmd_key]
            cmd.prepare(pydict)
            cmd.execute(pydict)
            return cmd

    def UpdateCommand(self, reply_message):
        cmd_id = reply_message.command_id
        for _, cmd in self.cmds.items():
            if cmd.ID == cmd_id:
                cmd.setState(reply_message.state, reply_message.error_code, reply_message.message)
                return cmd
        return None

    def loop(self, reply_command_and_reset):
        for _, cmd in self.cmds.items():
            if cmd.getState() == CommandReply.STATE_RUNNING:
                if cmd.Timeout():
                    print "ID:", cmd.ID, "timeout"
                    cmd.setState(CommandReply.STATE_TIMEOUT)
                    reply_command_and_reset(cmd)
                else:
                    cmd.loop()
