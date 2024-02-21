import rospy
import math
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition

def return_launch(target_latitude, target_longitude, target_altitude, drone_name):
    try:
        
        command_int = rospy.ServiceProxy(drone_name + '/mavros/cmd/command_int', CommandInt)
        response = command_int(
            frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            command=192,  # MAV_CMD_NAV_WAYPOINT
            current=2,  # Not used, but set to 2 according to MAVLink docs
            autocontinue=0,  # Don't continue to next waypoint after this
            param1=60,  # Hold time in seconds
            param2=0,  # Acceptance radius, not used here
            param3=0,  # Pass through, not used here
            param4=0,  # Pass through, not used here
            x=int(target_latitude * 1e7),  # Latitude in degrees * 1e7
            y=int(target_longitude * 1e7),  # Longitude in degrees * 1e7
            z=target_altitude  # Altitude in meters
        )
        rospy.loginfo("Position command sent to %s" % drone_name)
        
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed for %s: %s" % (drone_name, e))

if __name__ == '__main__':
    rospy.init_node('return_launch_node')

    drones = ['drone1', 'drone2', 'drone3', 'drone4']  # Add more drones as needed
    spacing_meters = 0.001  # Adjust spacing in meters
   
    drone_to_return_home = 'drone4'
    home_latitude = -35.3632622
    home_longitude = 149.1652375
    home_altitude = 10.0
    return_launch(home_latitude,home_longitude,home_altitude,drone_to_return_home)
