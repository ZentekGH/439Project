#!/usr/bin/env python3
import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439WaypointXY
from std_msgs.msg import Bool

# set user-defined waypoints here
waypoints = np.array([[20.0, 0.0]])

# some globals
waypoint_number = 0  # index of the current waypoint
path_complete = Bool() # the path_complete message
path_complete.data = False # initialize the path complete message to false

# sets up publishers, subscribers, and sets the current waypoint on topic /waypoint_xy
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
    # launch this node with the name 'set_waypoints'
    rospy.init_node('set_waypoints', anonymous=False)
    
    # here's the waypoint message that is used in the waypoint follower node
    msg_waypoint = ME439WaypointXY()
    
    # publisher to the topic /waypoint_xy to set the waypoint the bot is currently trying to get to
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)
    
    # publisher to the topic /path_complete to tell the robot when the path is complete
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # subscriber that listens on waypoint_complete that tells when the waypoint is reached
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)   

    # set up a rate basis to keep it on schedule
    r = rospy.Rate(10) # N Hz
    
    # main loop
    try: 
        while not rospy.is_shutdown():
            # publish whether or not the path is complete
            pub_path_complete.publish(path_complete)
            # if the path is complete, we're done here
            if path_complete.data:
                break
            # if we still have more to go, get the next waypoint and publish it
            else:
                msg_waypoint.x = waypoints[waypoint_number,0]
                msg_waypoint.y = waypoints[waypoint_number,1]
                pub_waypoint_xy.publish(msg_waypoint) 
            
            # sleep for the rate described above
            r.sleep()

    # just handles exceptions
    except Exception:
        traceback.print_exc()
        pass

# this just increments to the next waypoint if the current one is reached
# this also publishes whether or not path is complete
def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
    # if the current waypoint is complete, increment to the next waypoint in the path
    if msg_in.data:  
        waypoint_number = waypoint_number + 1
    
    # if the last waypoint is done, the path is done
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
    else:
        path_complete.data = False
        
    # publish whether or not the path is complete
    pub_path_complete.publish(path_complete)

# ignore
if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass

