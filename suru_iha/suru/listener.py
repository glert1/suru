import rospy
from geometry_msgs.msg import Point

def callback(data):
    rospy.loginfo("Received square position: x=%f, y=%f, z=%f" % (data.x, data.y, data.z))

if __name__ == '__main__':
    rospy.init_node('square_positions_sub')
    rospy.Subscriber('/square_positions', Point, callback)
    rospy.spin()
