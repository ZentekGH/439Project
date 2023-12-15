#!/usr/bin/env python3
import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439WaypointXY, WallDetection, ME439SensorsRaw
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose2D

# set user-defined waypoints here
waypoints = np.array([[0, 4.0]])
end_wp_copy = [0,4.0]

# some globals
waypoint_number = 0  # index of the current waypoint
avoiding = False # true when navigating an obstacle
path_complete = Bool() # the path_complete message
path_complete.data = False # initialize the path complete message to false
sub_wall_front = None 
sub_wall_left = None
pub_path_complete = None
sub_robot_pose_est = None
curr_pose = Pose2D()
waypoint_dist = 0.7 # the distance for each new avoidance waypoint
pub_override = None
turn_num = 0
wall_at_left = False
wall_at_front = False
pub_free = rospy.Publisher('/test', Float32, queue_size=1)
sensing = False
# sets up publishers, subscribers, and sets the current waypoint on topic /waypoint_xy
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_path_complete, sub_wall_front, sub_wall_left, curr_pose, pub_override, sensing
    
    # launch this node with the name 'set_waypoints'
    rospy.init_node('set_waypoints', anonymous=False)
    
    # here's the subscriber for the wall detection topic
    sub_wall_front = rospy.Subscriber('/wall_detected_front', Bool, wall_detected_forward)
    sub_wall_left = rospy.Subscriber('/wall_detected_left', Bool, wall_detected_left)

    # subscribe to wait for sensors
    sub_sensors = rospy.Subscriber('/sensors_data_raw', ME439SensorsRaw, set_sensing)

    # a subscriber for the robot pose
    sub_robot_pose_est = rospy.Subscriber('/robot_pose_estimated', Pose2D, get_robot_pose)

    # here's the waypoint message that is used in the waypoint follower node
    msg_waypoint = ME439WaypointXY()
    
    # publisher to the topic /waypoint_xy to set the waypoint the bot is currently trying to get to
    pub_waypoint_xy = rospy.Publisher('/waypoint_xy', ME439WaypointXY, queue_size=1)
    
    # publisher to the topic /path_complete to tell the robot when the path is complete
    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    # subscriber that listens on waypoint_complete that tells when the waypoint is reached
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)   

    # publisher for waypoint override
    pub_override = rospy.Publisher('/override_waypoint_complete', Bool, queue_size=1)
    

    # set up a rate basis to keep it on schedule
    r = rospy.Rate(10) # N Hz
    
    while (not sensing):
        continue

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

# set the sensing variable
def set_sensing(msg_in):
    global sensing
    sensing = True

# this is the callback for when a wall is detected either to left or front of bot
def wall_detected_forward(msg_in):
    global avoiding, pub_path_complete, path_complete, pub_override, waypoints, turn_num, wall_at_front
    wall_at_front = msg_in.data

    # if we're not currently avoiding an obstacle and there's no wall at front, ignore the callback
    if (not avoiding) and (not msg_in.data):
        return
    
    # if we're not currently in avoidance and we detect a wall at front, we have to say the path is complete and increment to next waypoint
    if (not avoiding) and (msg_in.data):
        #path_complete.data = True
        avoiding = True
        turn_num = turn_num + 1
        # calculate the new waypoints
        new_waypoint = get_next_waypoint_avoidance("right") #right affinity
        waypoints = np.append(waypoints, np.array([new_waypoint]), axis=0)
        msg_override = Bool()
        msg_override.data = True
        pub_override.publish(msg_override)
        return

# wall detected at left
def wall_detected_left(msg_in):
    global avoiding, wall_at_left
    wall_at_left = msg_in.data
    if not avoiding:
        return

# sets robot current pose for use elsewhere
def get_robot_pose(msg_in):
    curr_pose.x = msg_in.x
    curr_pose.y = msg_in.y
    curr_pose.theta = msg_in.theta

# gets the next waypoint when in avoidance
# direction is a string, either "left", "forward", or "right"
def get_next_waypoint_avoidance(direction):
    global curr_pose, waypoint_dist

    # the waypoint needs to be to the right of the current pose
    theta = 0
    new_x = 0
    new_y = 0
    if (direction == "right"):
        theta = curr_pose.theta + 90
        new_x = curr_pose.x + (waypoint_dist * np.sin(np.radians(theta)))
        new_y = curr_pose.y + (waypoint_dist * np.cos(np.radians(theta)))
    elif (direction == "center"):
        theta = curr_pose.theta
    elif (direction == "left"):
        theta = curr_pose.theta - 90
        new_x = curr_pose.x - (waypoint_dist * np.cos(np.radians(theta)))
        new_y = curr_pose.y - (waypoint_dist * np.sin(np.radians(theta)))
    else:
        exit()
    
    return np.array([new_x, new_y])


# this just increments to the next waypoint if the current one is reached
# this also publishes whether or not path is complete
def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete, avoiding, turn_num, wall_at_left, wall_at_front, waypoints, pub_free

    msg = Float32()
    msg.data = float(turn_num)
    pub_free.publish(msg)

    # if the current waypoint is complete, increment to the next waypoint in the path
    if msg_in.data:  
        waypoint_number = waypoint_number + 1
    
    # if the last waypoint is done, the path is done
    if waypoint_number >= waypoints.shape[0] and not avoiding:
        path_complete.data = True
    elif waypoint_number >= waypoints.shape[0] and avoiding and msg_in.data:
        # if we're here, the robot is still in obstacle avoidance but there is not another waypoint, so we need to make one
        path_complete.data = False
        if (turn_num == 1 or turn_num == 2):
            if (wall_at_left):
                # this means that there's a wall at the left and we're in the first avoidance mode, so we need to generate a center waypoint
                new_waypoint = get_next_waypoint_avoidance("center") #center affinity
                waypoints = np.append(waypoints, np.array([new_waypoint]), axis=0)
                #msg_override = Bool()
                #msg_override.data = True
                #pub_override.publish(msg_override)
            else:
                # this means that there's no wall and we're in first avoidance mode, so generate left waypoint
                turn_num = turn_num + 1
                new_waypoint = get_next_waypoint_avoidance("left") #center affinity
                waypoints = np.append(waypoints, np.array([new_waypoint]), axis=0)
                #msg_override = Bool()
                #msg_override.data = True
                #pub_override.publish(msg_override)
        else:
            # can just load the waypoint now
            # testing
            turn_num = 0
            avoiding = False
            waypoints = np.append(waypoints, np.array([end_wp_copy]), axis=0)
            #msg_override = Bool()
            #msg_override.data = True
            #pub_override.publish(msg_override)
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

