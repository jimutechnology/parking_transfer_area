#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from std_msgs.msg import Header
import uuid

def GetAreaID():
    '''
    返回交接区的 ID，如果设置了环境变量 AREA_ID，则使用该环境变量，否则返回 "0"
    '''
    area_id = os.environ.get('AREA_ID')
    if area_id is not None:
        return area_id
    return "0"


def getHeader():
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = str(uuid.uuid4())
    return header