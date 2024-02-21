import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
import math

def find_middle_drone(drones):
    num_drones = len(drones)
    middle_drone_index = num_drones // 2
    return drones[middle_drone_index]

def calculate_reverse_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters):
    # Calculate the angle between each drone
    angle_between_drones = 360.0 / len(drones)
    
    positions = []
    
    # Calculate positions for each drone
    for i, _ in enumerate(drones):
        angle = 180 + i * angle_between_drones  # Offset by 180 degrees for reverse triangle
        # Calculate the position offset with specified distance
        offset_latitude = math.cos(math.radians(angle)) * spacing_meters
        offset_longitude = math.sin(math.radians(angle)) * spacing_meters
        # Calculate the latitude and longitude for the current drone
        latitude = middle_drone_latitude + offset_latitude
        longitude = middle_drone_longitude + offset_longitude
        positions.append((latitude, longitude))
    
    return positions

def suru_iha_reverse_triangle(target_latitude, target_longitude, target_altitude, drones, spacing_meters):
    try:
        rospy.init_node('send_goto_position_node')
        
        middle_drone = find_middle_drone(drones)
        
        # Assuming target latitude and longitude are the same for all drones
        middle_drone_latitude = target_latitude
        middle_drone_longitude = target_longitude
        
        positions = calculate_reverse_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters)
        
        for i, (latitude, longitude) in enumerate(positions):
            command_int = rospy.ServiceProxy(drones[i] + '/mavros/cmd/command_int', CommandInt)
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
                z=target_altitude  # Altitude in meters
            )
            rospy.loginfo("Position command sent to drone %d" % (i+1))
            rospy.sleep(3)
        
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    # Example usage
    rospy.init_node('send_goto_position_node')
    
    # Example usage
    leader_position = NavSatFix()
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    # List of drones, add more as needed
    drones = ['/drone1', '/drone2', '/drone3']  # Add more drones as needed
    spacing_meters = 0.001  # Adjust spacing in meters
    
    suru_iha_reverse_triangle(target_latitude, target_longitude, target_altitude, drones, spacing_meters)
