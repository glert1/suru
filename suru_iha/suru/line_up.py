import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix

def suru_yaw(target_latitude, target_longitude, target_altitude, drones):
    try:
        rospy.init_node('send_goto_position_node')
        
        num_drones = len(drones)
        spacing = 0.0004489  # Adjust this value based on the desired spacing between drones
        
        for i, drone in enumerate(drones):
            adjusted_longitude = target_longitude + (i - num_drones / 2) * spacing
            
            command_int = rospy.ServiceProxy(drone + '/mavros/cmd/command_int', CommandInt)
            response = command_int(
                frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=201,  # Set ROI
                current=2,  # Not used, but set to 2 according to MAVLink docs
                autocontinue=0,  # Don't continue to next waypoint after this
                param1=target_latitude,  # Latitude of the ROI
                param2=adjusted_longitude,  # Longitude of the ROI
                param3=0,  # Not used
                param4=0,  # Not used
                x=0,  # Not used
                y=0,  # Not used
                z=target_altitude  # Altitude in meters
            )
            rospy.loginfo("Position command sent to drone %d" % (i+1))
        
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        
def suru_iha_cizgi_formasyonu(target_latitude, target_longitude, target_altitude, drones):
    try:
        rospy.init_node('send_goto_position_node')
        
        num_drones = len(drones)
        spacing = 0.0004489  # Adjust this value based on the desired spacing between drones
        
        for i, drone in enumerate(drones):
            adjusted_longitude = target_longitude + (i - num_drones / 2) * spacing
            
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
                x=int(target_latitude * 1e7),  # Latitude in degrees * 1e7
                y=int(adjusted_longitude * 1e7),  # Adjusted longitude in degrees * 1e7
                z=target_altitude  # Altitude in meters
            )
            rospy.loginfo("Position command sent to drone %d" % (i+1))
        
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
    
    suru_iha_cizgi_formasyonu(target_latitude, target_longitude, target_altitude, drones)
    suru_yaw(target_latitude, target_longitude, target_altitude, drones)
