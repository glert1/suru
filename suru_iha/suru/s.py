import rospy
from mavros_msgs.srv import CommandInt
import math

def takeoff_and_land(drones, target_latitude, target_longitude, target_altitude):
    try:
        rospy.init_node('takeoff_and_land_node')

        for drone in drones:
            # Takeoff command
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/command_int', CommandInt)
            takeoff_response = command_int(
                frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=22,  # MAV_CMD_NAV_TAKEOFF
                current=2,  # Not used, but set to 2 according to MAVLink docs
                autocontinue=0,  # Don't continue to next waypoint after this
                param1=0,  # Takeoff altitude type (0 for default)
                param2=0,  # Takeoff pitch angle (not used for default takeoff)
                param3=0,  # Takeoff heading (not used for default takeoff)
                param4=0,  # Empty (not used for default takeoff)
                x=int(target_latitude * 1e7),  # Latitude in degrees * 1e7
                y=int(target_longitude * 1e7),  # Longitude in degrees * 1e7
                z=target_altitude  # Altitude in meters
            )
            rospy.loginfo("Takeoff command sent to %s" % drone)
        
        rospy.sleep(5)  # Wait for drones to reach takeoff altitude
        
        # Land command
        for drone in drones:
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/command_int', CommandInt)
            landing_response = command_int(
                frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=21,  # MAV_CMD_NAV_LAND
                current=2,  # Not used, but set to 2 according to MAVLink docs
                autocontinue=0,  # Don't continue to next waypoint after this
                param1=0,  # Empty (not used for default landing)
                param2=0,  # Empty (not used for default landing)
                param3=0,  # Empty (not used for default landing)
                param4=0,  # Landing target number (not used for default landing)
                x=int(target_latitude * 1e7),  # Latitude in degrees * 1e7
                y=int(target_longitude * 1e7),  # Longitude in degrees * 1e7
                z=0  # Altitude in meters (land at current altitude)
            )
            rospy.loginfo("Landing command sent to %s" % drone)
        
        rospy.sleep(5)  # Wait for drones to land

        return takeoff_response.success and landing_response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    # Example usage
    rospy.init_node('takeoff_and_land_node')
    
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    drones = ['/drone3', '/drone2', '/drone1']
    
    takeoff_and_land(drones, target_latitude, target_longitude, target_altitude)
