#!/usr/bin/env python

import copy
import yaml
import threading
import manager
from transfer_area.msg import CommandReply
from area_common import GetAreaID, ServiceNode, RunService, MqttClient

PRIM_HZ = 200
DEBUG_SHOW_CMD = False

class AreaCommandMqtt(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay
        self.area_id = GetAreaID()
        print "area_id", self.area_id
        self.lock_cmd_reply = threading.Lock()

        self.mqtt_sub_cmd_topic  = "/area/command/{}".format(self.area_id)
        print self.mqtt_sub_cmd_topic
        self.mqtt_pub_cmd_reply_prefix  = "/area/command_reply/"

        self.cmdGroup = manager.CommandManger()
        self.cmd_reply_data = CommandReply()

        self.mqtt = MqttClient(self.on_connect)
        self.mqtt.client.message_callback_add(self.mqtt_sub_cmd_topic, self.on_message_area_command)
        self.mqtt.Connect()

        self.Subscriber("cmd_reply", CommandReply, self.rx_command_reply)

    def rx_command_reply(self, data):
        with self.lock_cmd_reply:
            self.cmd_reply_data = copy.deepcopy(data)
            command = self.cmdGroup.UpdateCommand(self.cmd_reply_data)
            if command:
                self.reply_command_and_reset(command)

    def reply_command_and_reset(self, command):
        reply_topic = self.mqtt_pub_cmd_reply_prefix + command.Session + "/" + self.area_id
        text = command.GetPublishMessage()
        if text:
            self.mqtt.publish(reply_topic, text)
            print "publish mqtt message"
        else:
            print('Invalid command reply msg: ', command.ID)
            
        if command.state in [CommandReply.STATE_FINISH, CommandReply.STATE_ERROR, CommandReply.STATE_TIMEOUT]:
            command.reset()

    def on_connect(self, client, userdata, flags, rc):
        self.mqtt.subscribe(self.mqtt_sub_cmd_topic)

    def check_message(self, msg_recv):
        try:
            msg_data = yaml.load(msg_recv.payload)
            if type(msg_data) is not dict:
                print("{}: Invalid Yaml dict.".format(msg_recv.payload))
                return False
        except:
            print("yaml.load() fail: {}".format(msg_recv.payload))
            return False

        return True

    def on_message_area_command(self, client, userdata, msg):
        if not self.check_message(msg):
            return
        msg_cmd = yaml.load(msg.payload)
        self.cmdGroup.execute(msg_cmd)

        if DEBUG_SHOW_CMD:
            cmd_info = CommandReply()
            cmd_info.command_id = str(msg_cmd["id"].upper())
            cmd_info.state = 0
            cmd_info.message = msg.payload
            self.cmd_info_pub.publish(cmd_info)

    def loop(self):
        self.mqtt.loop()

if __name__ == '__main__':
    RunService(AreaCommandMqtt)