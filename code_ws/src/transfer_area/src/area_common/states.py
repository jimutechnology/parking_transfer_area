# IMPORTANT!!!!: state is encoded in ROS msg as uint8, so max value is 255!!!
# this is sent by robot to reflect internal state

running_state = {
    "STANDBY": 0,
    "RUNNING": 10,
    "FINISH": 20,
    "FAULT": 100,
}

area_state = {
    "FREE"       : 0,
    "RESERVED"   : 1,
    "BUSY"       : 2,
    "UNAVAILABLE": 3, 
    "ERROR"      : 4
}