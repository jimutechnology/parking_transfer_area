import rospy

class AreaParam:
    def __init__(self, **kwargs):
        try:
            self.__dict__['area_param'] = rospy.get_param("area_param")
        except:
            # we are not setting default values any longer
            # self.default()
            raise Exception("No found area param")

    def Update(self):
        self.__dict__['area_param']  = rospy.get_param("area_param")

    def __setattr__(self, name, value):
        # print "set:", name, value
        self.__dict__['area_param'][name] = value
        try:
            rospy.set_param("area_param/" + name, value)
        except:
            pass

    def __getattr__(self, name):
        # print "get:", name
        if name in self.__dict__['area_param']:
            return self.__dict__['area_param'].get(name)
        else:
            raise Exception("No found attribute: " + name)

    def default(self):
        # motor clamp parameters
        area_param = {}

        self.__dict__['area_param'] = area_param


if __name__ == "__main__":
    robot = AreaParam()
