import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
import math

def find_middle_drone(drones, coordinates):
    # Ensure coordinates list is not empty
    if not coordinates:
        rospy.logerr("Coordinates list is empty. Please make sure to call get_coordinates first.")
        return None
    
    # Calculate the middle drone's latitude based on the coordinates
    num_drones = len(drones)
    middle_drone_index = num_drones // 2
    middle_drone_latitude = coordinates[middle_drone_index][0]  # Latitude is the first element in the tuple
    return middle_drone_latitude

def calculate_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters):
    # Calculate the angle between each drone
    angle_between_drones = 360.0 / len(drones)
    
    positions = []
    
    # Calculate positions for each drone
    for i, _ in enumerate(drones):
        angle = i * angle_between_drones
        # Calculate the position offset with specified distance
        offset_latitude = math.cos(math.radians(angle)) * spacing_meters
        offset_longitude = math.sin(math.radians(angle)) * spacing_meters
        # Calculate the latitude and longitude for the current drone
        latitude = middle_drone_latitude + offset_latitude
        longitude = middle_drone_longitude + offset_longitude
        positions.append((latitude, longitude))
    
    return positions

def suru_iha_equilateral_triangle(target_latitude, target_longitude, target_altitude, drones, spacing_meters):
    try:
        rospy.init_node('send_goto_position_node')
        
        coordinates = get_coordinates()
        
        middle_drone_latitude = find_middle_drone(drones, coordinates)
        
        if middle_drone_latitude is None:
            rospy.logerr("Unable to find middle drone latitude. Aborting mission.")
            return False
        
        middle_drone_longitude = target_longitude  # Assuming middle drone stays at the same longitude as the target
        
        positions = calculate_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters)
        
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
        return False
        
def callback(data, coordinates):
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    coordinates.append((latitude, longitude, altitude))

def get_coordinates():
    coordinates = []

    sub_drone1 = rospy.Subscriber('drone1/mavros/global_position/global', NavSatFix, callback, callback_args=([coordinates],))
    sub_drone2 = rospy.Subscriber('drone2/mavros/global_position/global', NavSatFix, callback, callback_args=([coordinates],))
    sub_drone3 = rospy.Subscriber('drone3/mavros/global_position/global', NavSatFix, callback, callback_args=([coordinates],))
    rospy.sleep(5)  # Wait for the subscribers to get registered and data to be collected
    rospy.spin()  # Keep the node running until it's shut down

    return coordinates


if __name__ == '__main__':
    # Example usage
    rospy.init_node('send_goto_position_node')
    
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    drones = ['/drone1', '/drone2', '/drone3']  # Add more drones as needed
    spacing_meters = 0.001  # Adjust spacing in meters
    
    suru_iha_equilateral_triangle(target_latitude, target_longitude, target_altitude, drones, spacing_meters)
