import rospy
from mavros_msgs.srv import CommandInt
import random
import time

def takeoff_and_maintain_formation(drones, target_altitude):
    try:
        rospy.init_node('send_goto_position_node')

        # Take off and maintain formation at target altitude
        for drone in drones:
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/takeoff', CommandInt)
            response = command_int(altitude=target_altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo("Takeoff command sent to %s" % drone)

        # Wait for all drones to take off
        time.sleep(5)

        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def land(drones):
    try:
        # Land all drones
        for drone in drones:
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/land', CommandInt)
            response = command_int(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo("Land command sent to %s" % drone)

        # Wait for all drones to land
        time.sleep(5)

        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    # Example usage
    rospy.init_node('send_goto_position_node')
    
    # List of drones
    drones = ['/drone1', '/drone2', '/drone3', '/drone4']

    target_altitude = 50  # in meters

    # Take off and maintain formation
    takeoff_and_maintain_formation(drones, target_altitude)

    # Wait for a while at the target altitude
    time.sleep(10)

    # Land all drones
    land(drones)
