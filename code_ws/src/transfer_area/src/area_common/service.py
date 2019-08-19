#!/usr/bin/env python

import rospy

from robot_msgs.msg import InfoOut

DEFAULT_BUFF_SIZE = 65536

def Publisher(name, data_class, subscriber_listener=None,
            tcp_nodelay=False, latch=False, headers=None, queue_size=None):
    return rospy.Publisher(name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)


def Subscriber(name, data_class, callback=None, callback_args=None,
                queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):

    return rospy.Subscriber(name, data_class, callback, callback_args,
                queue_size, buff_size, tcp_nodelay)

class ServiceNode:
    def __init__(self):
        self.Name = self.__class__.__name__

        print "New ROS node: %s" % self.Name
        rospy.init_node(self.Name, anonymous=True)
        self.rate = rospy.Rate(10)
        self.warning_pub = self.Publisher('warning_info', InfoOut, queue_size=10)
        self.error_pub   = self.Publisher('error_info', InfoOut, queue_size=10)

        self.warnings = {
        }
        self.errors = {
        }

    def setRate(self, rate):
        if rate == 0:
            self.rate = None
        else:
            self.rate = rospy.Rate(rate)

    def Publisher(self, name, data_class, subscriber_listener=None,
                tcp_nodelay=False, latch=False, headers=None, queue_size=None):
        return rospy.Publisher(name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size)


    def Subscriber(self, name, data_class, callback=None, callback_args=None,
                 queue_size=None, buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        return rospy.Subscriber(name, data_class, callback, callback_args,
                 queue_size, buff_size, tcp_nodelay)

    def Service(self, name, data_class, callback=None):
        return rospy.Service(name, data_class, callback)

    def loop(self):
        pass

    def cleanup(self):
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            if self.rate:
                self.rate.sleep()

        self.cleanup()

    def out(self, error_code, message=None):
        info = InfoOut()
        info.header.frame_id = self.Name
        info.error_code = error_code
        if error_code in self.warnings:
            if message == None:
                info.message = self.warnings[error_code]
            else:
                info.message = message
            self.warning_pub.publish(info)
        elif error_code in self.errors:
            if message == None:
                info.message = self.errors[error_code]
            else:
                info.message = message
            self.error_pub.publish(info)
        else:
            rospy.logout("error_code: %d, message: %s" % (error_code, message))


def RunService(service_class):
    try:
        service_class().run()
    except rospy.ROSInterruptException:
        pass
