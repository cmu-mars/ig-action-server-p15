import rospy
import rosnode
import subprocess
import psutil
from brass_gazebo_plugins.srv import *

class CP3_Instructions(object):
    NODE_MAP = {"aruco" : ["aruco_marker_publisher_front", "aruco_marker_publisher_back", "marker_manager", "marker_pose_publisher"],
                "amcl" : ["amcl"],
                "mrpt" : ["mrpt_localization_node"]}

    LAUNCH_MAP = {"aruco" : "cp3-aruco.launch",
                  "amcl" : "cp3-amcl.launch",
                  "mrpt" : "cp3-mrpt.launch"}

    SENSORS = ["kinect", "lidar", "cameras", "camera", "headlamp"]

    launched = None

    def __init__(self):
        self.launched = None

    def kill_launch(self, cmd):
        for proc in psutil.process_iter():
            s = [str(item) for item in proc.cmdline()]
            j = " ".join(s)
            if len(proc.cmdline()) > 0 and j.endswith(cmd):
                proc.terminate()


    def kill_nodes(self,config_id):

        if self.launched is not None:
            self.kill_launch(self.launched)
        self.launched = None
        config_id = config_id.lower()
        if config_id not in self.NODE_MAP.keys():
            return True, "Illegal config passed in: %s" %config_id 
        nodes = self.NODE_MAP[config_id]

        if nodes is None or len(nodes) == 0:
            return False, "Nothing to kill"

        rosnode.kill_nodes(nodes)

        start = rospy.Time.now()
        end = start + rospy.Duration(30)

        nodes1 = ['/' + s for s in nodes]


        current = rosnode.get_node_names() 
        while rospy.Time.now() < end and any(x in current for x in nodes1):
            rospy.sleep(1)
            current = rosnode.get_node_names()
        if any(x in current for x in nodes1):
            return False, "Nodes were not killed"


        return True, None

    def start_nodes(self,config_id):
        config_id = config_id.lower()
        if config_id not in self.LAUNCH_MAP.keys():
            return False, "Illegal config passed in: %s" %config_id

        launch = self.LAUNCH_MAP[config_id]

        launch_cmd = "roslaunch cp3_base %s" %launch
        print("Launching %s" %launch_cmd)
        roslaunch = subprocess.Popen(launch_cmd, shell=True)
        self.launched = launch_cmd
        nodes = self.NODE_MAP[config_id]

        end = rospy.Time.now() + rospy.Duration(30)
        nodes1 = ['/' + s for s in nodes]
        current = rosnode.get_node_names()
        while rospy.Time.now() < end and not all(x in current for x in nodes1):
            rospy.sleep(1)
            current = rosnode.get_node_names()

        if not all(x in current for x in nodes1):
            return False, 'Not all nodes started'

        rospy.sleep(30) #Let's give things a chance to settle down

        return True, None


    def set_sensor(self, sensor, enablement):
        sensor = sensor.lower()

        if not sensor in self.SENSORS:
            return False, "Unknown sensor: %s" %sensor
        if not isinstance(enablement, bool):
            if enablement in ["True", "true", "on"]:
                enablement = True
            elif enablement in ["False", "false", "off"]:
                enablement = False
            else:
                return False, "Uninterpretable enablement passed in: %s" %str(enablement)

        result = False
        if sensor == "kinect":
            set_kinect_srv = rospy.ServiceProxy('/mobile_base/kinect/mode', SetKinectMode)
            result = set_kinect_srv(1 if enablement else 0)
        elif sensor == "lidar":
            set_lidar_srv = rospy.ServiceProxy('/mobile_base/lidar/mode', SetLidarMode)
            result = set_lidar_srv(enablement)
        elif sensor == "cameras" or sensor=="camera":
            set_kinect_srv = rospy.ServiceProxy('/mobile_base/kinect/mode', SetKinectMode)
            result = set_kinect_srv(2 if enablement else 0)
        elif sensor == "headlamp":
            set_headlamp_srv = rospy.ServiceProxy("/mobile_base/headlamp", ToggleHeadlamp)
            result = set_headlamp_srv(enablement)
        else:
            return False, "Something weird happened in set_sensor"

        return result, None if result else "set_sensor(%s,%s) failed" %(sensor,str(enablement))