import rospy
from sensor_msgs.msg import NavSatFix

def callback(data):
    rospy.loginfo("Received coordinates: lat=%s, lon=%s",data.latitude,data.longitude)

def listener():
    rospy.init_node('coordinates_listener', anonymous=True)
    rospy.Subscriber('get_coordinates', NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
