#! /usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
from area_param import AreaParam
from area_common import GetAreaID

class MqttClient:
    def __init__(self, on_connect=None, on_message=None):
        self.client = mqtt.Client(client_id="area_" + GetAreaID())

        if on_connect:
            self.client.on_connect = on_connect
        if on_message:
            self.client.on_message = on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe  = self.on_subscribe

    def connected(self):
        return self.client._state == mqtt.mqtt_cs_connected

    def Connect(self):
        param = AreaParam()
        print param.mqtt_server
        self.client.connect(param.mqtt_server, 1883, 60)

    def publish(self, topic, text, qos=2):
        if self.client._state == mqtt.mqtt_cs_connected:
            self.client.publish(topic, text, qos)

    def subscribe(self, topic, qos=2):
        if self.client._state == mqtt.mqtt_cs_connected:
            self.client.subscribe(topic, qos)

    # def on_connect(self, client, userdata, flags, rc):
    #     print("Connected with result code "+str(rc))

    # def on_message(self, client, userdata, msg):
    #     pass

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("on_subscribe: mid: {}, granted_qos: {}".format(mid, granted_qos))

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected ... reconnecting")
        self.client.reconnect()

    def loop(self):
        self.client.loop()
