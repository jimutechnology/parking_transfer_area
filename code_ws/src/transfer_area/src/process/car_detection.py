#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
from transfer_area.msg import CommandReply, StateCmd 
from transfer_area.srv import getAreaState

from area_common import ServiceNode, RunService, running_state as RS, area_state as AS

PRIM_HZ = 10

class Car_Detection(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(PRIM_HZ) # no delay

    def loop(self):
        with self.lock_loop_data:
            print("loop")
                    




if __name__ == '__main__':
    RunService(Car_Detection)