import rospy
import math
from mavros_msgs.srv import CommandInt
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
import time

def publish_target_coordinates(latitude, longitude, altitude):
    rospy.init_node('target_coordinates_publisher', anonymous=True)
    pub = rospy.Publisher('/drone4/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
      # Publish at 10 Hz

    while not rospy.is_shutdown():
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        msg.type_mask = GlobalPositionTarget.IGNORE_VX + GlobalPositionTarget.IGNORE_VY + GlobalPositionTarget.IGNORE_VZ + GlobalPositionTarget.IGNORE_AFX + GlobalPositionTarget.IGNORE_AFY + GlobalPositionTarget.IGNORE_AFZ + GlobalPositionTarget.IGNORE_YAW_RATE
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        pub.publish(msg)
        