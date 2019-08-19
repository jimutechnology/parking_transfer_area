#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import rospy
import threading
import time
from robot_msgs.msg import CommandReply
from std_msgs.msg import Header
from area_common import GetAreaID, Publisher, Subscriber, TimerDuration

DEFAULT_BUFF_SIZE = 65536

class Command:
    def __init__(self):
        self.state = CommandReply.STATE_EMPTY
        self.lock = threading.RLock()
        self.ID = ""
        self.type = ""
        self.Session = ""
        self.message = None
        self.error_code = 0
        self.text = ""
        self.debug_mode = False
        self.time_stamp = time.time()
        self.timer = TimerDuration()
        self.cmd_state_pub  = self.Publisher('cmd_reply', CommandReply, queue_size=1)

    def prepare(self, cmd_dict):
        self.ID         = str(self.get(cmd_dict, "id", ""))
        self.type       = str(self.get(cmd_dict, "cmd", ""))
        self.Session    = str(self.get(cmd_dict, "session", ""))
        self.time_stamp = self.get(cmd_dict, "time_stamp", self.time_stamp)

    def execute(self, cmd_dict):
        pass

    def cleanup(self):
        pass

    def Reply(self, state = CommandReply.STATE_FINISH, detail = ''):
        cmd_reply = CommandReply(
            command_id = self.ID,
            state = state,
            message = detail,
        )
        self.cmd_state_pub.publish(cmd_reply)

    def GetPublishMessage(self):
        if self.ID:
            message_text  = "id: " + str(self.ID)
            message_text += "\n" + "state: " + str(self.getState())
            message_text += "\n" + "error_code: " + str(self.error_code)
            if self.text:
                message_text += "\n" + "text: " + str(self.text)
            if self.message:
                message_text += "\n" + str(self.message)
            # set parameters to defaults
            self.text = ""
            self.message = None
            return message_text
        return None

    def reset(self):
        self.ID = ""
        self.cleanup()
        self.setState(CommandReply.STATE_READLY)

    def getHeader(self, cmd_dict):
        time_stamp = self.get(cmd_dict, "time_stamp", self.time_stamp)
        return Header(
            frame_id = str(self.get(cmd_dict, "id", "")),
            stamp = rospy.Time.from_sec(time_stamp / 1000.0),
        )

    def getState(self):
        with self.lock:
            return self.state

    def setState(self, state, error_code=0, text=""):
        with self.lock:
            self.state = state
            self.error_code = error_code
            self.text = text

    def setTimeout(self, timeout):
        self.timer.duration = timeout
        self.timer.Start()

    def stopTimeout(self):
        self.timer.Stop()

    def pauseTimeout(self):
        self.timer.Pause()

    def resumeTimeout(self):
        self.timer.Resume()

    def Timeout(self):
        return self.timer.Tick()

    def get(self, cmd_dict, key, default):
        if key in cmd_dict:
            return cmd_dict[key]
        return default

    def loop(self):
        pass

    def Publisher(self, name, data_class, subscriber_listener=None,
                tcp_nodelay=False, latch=False, headers=None, queue_size=None):
        return Publisher(name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)

    def Subscriber(self, name, data_class, callback=None, callback_args=None,
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        return Subscriber(name, data_class, callback, callback_args,
                 queue_size, buff_size, tcp_nodelay)
