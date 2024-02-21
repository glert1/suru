import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
import math

def suru_iha_custom_lineup_angles(target_latitude, target_longitude, target_altitude, drones, angles_degrees):
    try:
        rospy.init_node('send_goto_position_node')
        
        # Define custom positions for each drone in a lineup
        drone_positions = {}
        for i, drone in enumerate(drones):
            angle = angles_degrees[i]
            adjusted_longitude = target_longitude + math.cos(math.radians(angle)) * 0.0003
            adjusted_latitude = target_latitude + math.sin(math.radians(angle)) * 0.0003
            drone_positions[drone] = (adjusted_latitude, adjusted_longitude, target_altitude)
        
        for drone in drones:
            latitude, longitude, altitude = drone_positions[drone]
            
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/command_int', CommandInt)
            response = command_int(
                frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=192,  # MAV_CMD_NAV_WAYPOINT
                current=2,  # Not used, but set to 2 according to MAVLink docs
                autocontinue=0,  # Don't continue to next waypoint after this
                param1=60,  # Hold time in seconds
                param2=0,  # Acceptance radius, not used here
                param3=0,  # Pass through, not used here
                param4=0,  # Pass through, not used here
                x=int(latitude * 1e7),  # Latitude in degrees * 1e7
                y=int(longitude * 1e7),  # Longitude in degrees * 1e7
                z=altitude  # Altitude in meters
            )
            rospy.loginfo("Position command sent to %s" % drone)
        
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    # Example usage
    rospy.init_node('send_goto_position_node')
    
    # Example usage
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    # List of drones
    drones = ['/drone3', '/drone2', '/drone1']
    
    # Specify the angles for each drone in degrees
    angles_degrees = [90, 0, -90]  # Drone 1 at 90 degrees, Drone 2 at 0 degrees, Drone 3 at -90 degrees
    
    suru_iha_custom_lineup_angles(target_latitude, target_longitude, target_altitude, drones, angles_degrees)
