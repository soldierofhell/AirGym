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

# environment variables: ARDUPILOT_PATH

class AirGymSITLRL():
    def __init__(self, ):
        self._launch_sitl()        
    
    def _launch_apm(self):
        sim_vehicle_sh = str(os.environ["ARDUPILOT_PATH"]) + "/Tools/autotest/sim_vehicle.sh"
        subprocess.Popen(["xterm","-e",sim_vehicle_sh,"-j4","-f","quad","-v","ArduCopter"])    
    
    def run(self):


if __name__ == '__main__':
    rospy.init_node("ArducopterRL", anonymous=True)
    print("Welcome")
    patroler = ArducopterRL()
    try:
        ArducopterRL.run()
    except rospy.ROSInterruptException:
        pass
