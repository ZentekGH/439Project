#!/usr/bin/env python3
import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439WaypointXY, ME439PathSpecs 
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

# set the waypoint tolerance, how close the bot must be for the waypoint to be reached
waypoint_tolerance = rospy.get_param('/waypoint_tolerance') 

# this is a global variable that holds the current waypoint, init to NaN
waypoint = ME439WaypointXY()
waypoint.x = np.nan 
waypoint.y = np.nan

# this is a global variable that stores whether the current waypoint is reached or not
waypoint_complete = Bool()
waypoint_complete.data = False    

# two publishers, one for path segments specs and another for waypoint complete
pub_segment_specs = rospy.Publisher('/path_segment_spec', ME439PathSpecs, queue_size=1)
pub_waypoint_complete = rospy.Publisher('/waypoint_complete', Bool, queue_size=1)

# start the node and create some subscribers
def talker(): 
    # Actually launch a node called "waypoint_seeker"
    rospy.init_node('waypoint_seeker', anonymous=False)
    
    # here's a subscriber to the estimated robot pose
    sub_robot_pose_estimated = rospy.Subscriber('/robot_pose_estimated', Pose2D, set_path_to_waypoint)

    # here's the subscriber to the current waypoint
    sub_waypoint = rospy.Subscriber('/waypoint_xy', ME439WaypointXY, set_waypoint)

    # Prevent the node from exiting
    rospy.spin()    


# this function is called whenever the robot's estimated position is set
# as of right now, this sets the path to a straight line to the waypoint from its position
def set_path_to_waypoint(pose_msg_in):
    # get access to globals and assign the estimated pose to a variable
    global estimated_pose, waypoint, pub_segment_specs, waypoint_complete
    estimated_pose = pose_msg_in
        
    # find the x and y distances from robot's current position to the waypoint
    dx = waypoint.x - estimated_pose.x     # distance in X coords
    dy = waypoint.y - estimated_pose.y     # distance in Y coords
    
    # set the path specs to a straight line from the current location to the waypoint
    path_segment_spec = ME439PathSpecs()
    path_segment_spec.x0 = estimated_pose.x
    path_segment_spec.y0 = estimated_pose.y
    path_segment_spec.theta0 = np.arctan2(-dx, dy)    
    path_segment_spec.Radius = np.inf
    path_segment_spec.Length = np.sqrt(dx**2 + dy**2)
    
    # path segment will be NaN if we haven't gotten a waypoint yet
    if not np.isnan(path_segment_spec.Length):
        # publish the path specs to get to the waypoint
        pub_segment_specs.publish(path_segment_spec)
    
    # if we're at the waypoint, send the waypoint complete message to load next waypoint
    if (path_segment_spec.Length < waypoint_tolerance) and not waypoint_complete.data: 
        waypoint_complete.data = True
        pub_waypoint_complete.publish(waypoint_complete)

    
# callback for when we get a new waypoint, grabs the waypoint, resets values
def set_waypoint(waypoint_msg_in): 
    global waypoint, waypoint_complete
    waypoint = waypoint_msg_in
    waypoint_complete.data = False
    # might need to comment this out
    pub_waypoint_complete.publish(waypoint_complete)

# ignore
if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
