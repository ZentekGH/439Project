#!/usr/bin/env python3
import rospy
import numpy as np
import traceback 
from geometry_msgs.msg import Pose2D
from mobrob_util.msg import ME439WheelDisplacements

# get parameters
wheel_width = rospy.get_param('/wheel_width_model')
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0

# Global variables for the robot's position
r_center_world_estimated = np.array([0.,0.])    # Position (r) of the robot in the World frame. 
theta_estimated = 0.                            # heading angle (theta) of the robot relative to the World frame. 

# Global variables for the robot's wheel displacements (to keep knowledge of it from one step to the next)
d_left_previous = 0.
d_right_previous = 0.

# Rate to set how often the estimated "pose" is published
f = 10.     # Hz 

def listener(): 
    global r_center_world_estimated, theta_estimated
    
    # start the node
    rospy.init_node('dead_reckoning', anonymous=False)
    
    
    # subscribe to wheel displacements
    sub_wheel_disps = rospy.Subscriber('/robot_wheel_displacements', ME439WheelDisplacements, dead_reckoning)  

    
    # subscribe to robot position override (?)
    sub_position_override = rospy.Subscriber('/robot_position_override', Pose2D, set_pose)  
    
    # publisher for the final robot pose estimated from dead reckoning
    pub_robot_pose_estimated = rospy.Publisher('/robot_pose_estimated', Pose2D, queue_size = 1)
    robot_pose_estimated_message = Pose2D()

    # set the rate of publishing
    r = rospy.Rate(f)
    
    #  loop to publish the estimated pose
    while not rospy.is_shutdown():
        # Publish the pose
        robot_pose_estimated_message.x = r_center_world_estimated[0]
        robot_pose_estimated_message.y = r_center_world_estimated[1]
        robot_pose_estimated_message.theta = theta_estimated
        pub_robot_pose_estimated.publish(robot_pose_estimated_message)
        
        # Log the info to the ROS log. 
        rospy.loginfo(pub_robot_pose_estimated)
        
        r.sleep()
        
# calculates the robot's position in the world frame
def dead_reckoning(msg_in): 
    # These global variables hold this node's estimate of robot pose. 
    global r_center_world_estimated, theta_estimated
    # This parameter is necessary. 
    global wheel_width
    # More globals to store the previous values of the wheel displacements    
    global d_left_previous, d_right_previous
    
    
    # grab wheel displacements
    d_left = msg_in.d_left
    d_right = msg_in.d_right
    
    # compute the change in displacements
    diff_left = d_left - d_left_previous
    diff_right = d_right - d_right_previous
    
    # update previous values to current
    d_left_previous = d_left
    d_right_previous = d_right
    
    # get change in path length and change in wheel angle
    diff_pathlength = (diff_left+diff_right)/2
    diff_theta = (diff_right-diff_left)/wheel_width

    # compute the average of the headings 
    theta_avg = theta_estimated + diff_theta/2.
    
    # compute change in position and heading
    r_center_world_estimated[0] += diff_pathlength * -np.sin(theta_avg)     # x-direction position
    r_center_world_estimated[1] += diff_pathlength * np.cos(theta_avg)      # y-direction position
    theta_estimated += diff_theta                                           # angle

# sets the robot pose to something
def set_pose(msg_in): 
    global r_center_world_estimated, theta_estimated
    r_center_world_estimated[0] = msg_in.x
    r_center_world_estimated[1] = msg_in.y
    theta_estimated = msg_in.theta
    
    
if __name__ == '__main__':
    try: 
        listener()
    except rospy.ROSInterruptException: 
        pass
#        traceback.print_exc()
