import rospy
import math
import mavros
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix, TimeReference
from std_msgs.msg import Header, Float64
from mavros_msgs.srv import *
from mavros.param import param_set, param_get_all, param_get

import random

from pymavlink import mavutil
from pymavlink.mavutil import location

class ArducopterRL():
    def __init__(self, ):
    
    def run(self):


if __name__ == '__main__':
    rospy.init_node("ArducopterRL", anonymous=True)
    print("Welcome")
    patroler = ArducopterRL()
    try:
        ArducopterRL.run()
    except rospy.ROSInterruptException:
        pass
