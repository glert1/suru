import rospy
import math
from mavros_msgs.srv import CommandInt
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
import time


lat = 0.0
lon = 0.0
alt = 0.0

def calculate_square_positions(center_latitude, center_longitude, spacing_meters):
    angles = [45, 135, 225, 315]  # Angles for each corner
    positions = []
    for angle in angles:
        offset_latitude = math.cos(math.radians(angle)) * spacing_meters
        offset_longitude = math.sin(math.radians(angle)) * spacing_meters
        latitude = center_latitude + offset_latitude
        longitude = center_longitude + offset_longitude
        positions.append((latitude, longitude))

    return positions

def send_goto_position(target_latitude, target_longitude, target_altitude, drone_name, latitude, longitude):
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
            x=int(latitude * 1e7),  # Latitude in degrees * 1e7
            y=int(longitude * 1e7),  # Longitude in degrees * 1e7
            z=target_altitude  # Altitude in meters
        )
        rospy.loginfo("Position command sent to %s" % drone_name)
        rospy.sleep(3)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed for %s: %s" % (drone_name, e))



        
        

    
    
    
if __name__ == '__main__':
    
    # Example usage
    target_latitude = -35.362905
    target_longitude = 149.164135
    target_altitude = 100  # in meters

    # List of drones, add more as needed
    drones = ['drone1', 'drone2', 'drone3', 'drone4']  # Add more drones as needed
    spacing_meters = 0.001  # Adjust spacing in meters

    positions = calculate_square_positions(target_latitude, target_longitude, spacing_meters)

    for i, (latitude, longitude) in enumerate(positions):
        send_goto_position(target_latitude, target_longitude, target_altitude, drones[i], latitude, longitude)


    
    
    
    
  
    
    
    
      
    
