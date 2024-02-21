import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
import math

def find_middle_drone(drones):
    num_drones = len(drones)
    middle_drone_index = num_drones // 2
    return drones[middle_drone_index]

def calculate_lineup_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters):
    # Calculate positions for each drone in a line
    positions = []
    
    for i in range(len(drones)):
        latitude = middle_drone_latitude
        longitude = middle_drone_longitude + i * spacing_meters
        positions.append((latitude, longitude))
    
    return positions

def calculate_equilateral_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters):
    # Calculate the angle between each drone
    angle_between_drones = 360.0 / len(drones)
    
    positions = []
    
    # Calculate positions for each drone
    for i, _ in enumerate(drones):
        angle = 360.0 / len(drones) * i
        offset_latitude = math.cos(math.radians(angle)) * spacing_meters
        offset_longitude = math.sin(math.radians(angle)) * spacing_meters
        latitude = middle_drone_latitude + offset_latitude
        longitude = middle_drone_longitude + offset_longitude
        positions.append((latitude, longitude))
    
    return positions

def calculate_reverse_triangle_positions(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters):
    # Calculate the angle between each drone
    angle_between_drones = 360.0 / len(drones)
    
    positions = []
    
    # Calculate positions for each drone
    for i, _ in enumerate(drones):
        angle = 180 + i * angle_between_drones  # Offset by 180 degrees for reverse triangle
        offset_latitude = math.cos(math.radians(angle)) * spacing_meters
        offset_longitude = math.sin(math.radians(angle)) * spacing_meters
        latitude = middle_drone_latitude + offset_latitude
        longitude = middle_drone_longitude + offset_longitude
        positions.append((latitude, longitude))
    
    return positions

def send_command(position, target_altitude, drone_index, drones, next_function):
    try:
        latitude, longitude = position
        command_int = rospy.ServiceProxy(drones[drone_index] + '/mavros/cmd/command_int', CommandInt)
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
        rospy.loginfo("Position command sent to drone %d" % (drone_index+1))
        
        if response.success:
            # If the command was successful, proceed to the next step
            next_function()
        else:
            rospy.logerr("Command failed for drone %d" % (drone_index+1))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def lineup_then_equilateral_then_reverse(target_latitude, target_longitude, target_altitude, drones, spacing_meters):
    try:
        rospy.init_node('send_goto_position_node')
        
        middle_drone = find_middle_drone(drones)
        middle_drone_latitude = target_latitude
        middle_drone_longitude = target_longitude
        
        # Define the sequence of maneuvers
        maneuvers = [
            (calculate_lineup_positions, middle_drone_latitude, middle_drone_longitude),
            (calculate_equilateral_triangle_positions, middle_drone_latitude, middle_drone_longitude),
            (calculate_reverse_triangle_positions, middle_drone_latitude, middle_drone_longitude)
        ]
        
        # Start the sequence
        execute_maneuvers(maneuvers, 0, target_altitude, drones, spacing_meters)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error occurred: %s" % str(e))

def execute_maneuvers(maneuvers, index, target_altitude, drones, spacing_meters):
    if index >= len(maneuvers):
        rospy.loginfo("All maneuvers completed successfully.")
        return
    
    # Get the current maneuver parameters
    maneuver_function, middle_drone_latitude, middle_drone_longitude = maneuvers[index]
    
    # Calculate positions for the current maneuver
    positions = maneuver_function(middle_drone_latitude, middle_drone_longitude, drones, spacing_meters)
    
    # Define the callback function for the next maneuver
    if index + 1 < len(maneuvers):
        next_function = lambda: execute_maneuvers(maneuvers, index + 1, target_altitude, drones, spacing_meters)
    else:
        next_function = lambda: rospy.loginfo("All maneuvers completed successfully.")
    
    # Send commands for the current maneuver
    for i, position in enumerate(positions):
        send_command(position, target_altitude, i, drones, next_function)

if __name__ == '__main__':
    # Example usage
    rospy.init_node('send_goto_position_node')
    
    leader_position = NavSatFix()
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    # List of drones, add more as needed
    drones = ['/drone1', '/drone2', '/drone3']  # Add more drones as needed
    spacing_meters = 0.001  # Adjust spacing in meters
    
    lineup_then_equilateral_then_reverse(target_latitude, target_longitude, target_altitude, drones, spacing_meters)
