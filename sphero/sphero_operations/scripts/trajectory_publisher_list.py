#!/usr/bin/env python3

import rospy
import math
import sys
from math import sin, cos, sqrt, atan2
from sensor_msgs.msg import JointState

def generate_js(filename):
    js_list = []
    fh = open(filename, 'r')
    for line in fh:
        data = line.strip()
        data = line.split()
        x = float(data[0]) # x coordinate
        y = float(data[1]) # y coordinate
        th = float(data[2]) #radians
        j = JointState()
        j.header.stamp = rospy.Time.now()
        j.name = ['x', 'y', 'theta']
        j.position = [x,y,th]
        js_list.append(j)
    return js_list
 
def publish_js_lin_vel(js_list, max_vel, pub, rate):
    for j_idx in range(1,len(js_list)):
        start_configuration = js_list[j_idx-1].position
        goal_configuration = js_list[j_idx].position
        # Get current time
        start_time = rospy.get_rostime()
        del_x = goal_configuration[0] - start_configuration[0]
        del_y = goal_configuration[1] - start_configuration[1]
        dist_sqr = del_x * del_x + del_y * del_y
        dist = sqrt(dist_sqr)
        del_theta = goal_configuration[2] - start_configuration[2]
        traj_tf = dist / max_vel
        
        # Log data
        rospy.loginfo("Total distance: %f", dist)
        rospy.loginfo("Trajectory time: %f", traj_tf)
        cos_angle = del_x / dist
        sin_angle = del_y / dist
        
        # initialize message type
        new_state = js_list[j_idx-1] 
        # -- while loop through the list of joint states
        while not rospy.is_shutdown():
            time_elapsed = (rospy.get_rostime() - start_time)
            time_secs = time_elapsed.secs + time_elapsed.nsecs/1e9
            if(time_secs <= traj_tf):
                pt =  max_vel * time_secs
                new_state.header.stamp = rospy.get_rostime()

                # Write position of the robot to the message data
                # Note: traj_tf = 0 can cause a problem
                if traj_tf < 1e-9:
                    new_state.position = [start_configuration[0] + pt * cos_angle, start_configuration[1] + pt * sin_angle, goal_configuration[2]]
                else:
                    new_state.position = [start_configuration[0] + pt * cos_angle, start_configuration[1] + pt * sin_angle, start_configuration[2] + time_secs * (del_theta)/traj_tf]

            else:
                if j_idx == len(js_list) -1 : # last index
                    new_state.header.stamp = rospy.get_rostime()
                    new_state.position = goal_configuration
                else:
                    break

            # Send the message
            pub.publish(new_state);
            rate.sleep()
        

def talker(filename):
    # -- define maximum velocity
    max_vel = 0.5
    
    # -- initialize node
    rospy.init_node('trajectory_publisher', anonymous=True)
    
    # -- create ROS publisher
    pub = rospy.Publisher('/publish_joint_states', JointState, queue_size=10)
    rate = rospy.Rate(30) # 30hz
    
    js_list = generate_js(filename)
    
    publish_js_lin_vel(js_list, max_vel, pub, rate)
    
    return

if __name__ == '__main__':
    try:
        talker(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
