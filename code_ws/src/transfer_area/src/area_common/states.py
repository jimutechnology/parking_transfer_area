# IMPORTANT!!!!: state is encoded in ROS msg as uint8, so max value is 255!!!
# this is sent by robot to reflect internal state

running_state = {
    "STANDBY": 0,
    "RUNNING": 10,
    "FINISH": 20,
    "FAULT": 100,
}

area_state = {
    "FREE"      : 0,
    "FORWARD"   : 1,
    "BACK"      : 2,
    "QUIT"      : 3,
    "FINISH"    : 4,
    "RUNNING"   : 5
}