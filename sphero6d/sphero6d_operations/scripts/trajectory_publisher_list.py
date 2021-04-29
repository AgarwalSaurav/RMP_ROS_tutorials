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
        z = float(data[2]) # y coordinate
        th = float(data[3]) #radians
        j = JointState()
        j.header.stamp = rospy.Time.now()
        j.name = ['x', 'y', 'z', 'ang_x', 'ang_y', 'ang_z']
        j.position = [x, y, z, 0, 0, th]
        js_list.append(j)
    return js_list
 
def publish_js_lin_vel(js_list, max_vel, pub, rate):
    start_state = js_list[0]
    for j_idx in range(1,len(js_list)):
        start_configuration = js_list[j_idx-1].position
        goal_configuration = js_list[j_idx].position

        # Get current time
        start_time = rospy.get_rostime()

        del_x = goal_configuration[0] - start_configuration[0]
        del_y = goal_configuration[1] - start_configuration[1]
        del_z = goal_configuration[2] - start_configuration[2]

        dist_xy_sqr = del_x * del_x + del_y * del_y
        dist_sqr = dist_xy_sqr + del_z * del_z

        dist = sqrt(dist_sqr)
        dist_xy = sqrt(dist_xy_sqr)
        dist_sqr = del_x * del_x + del_y * del_y

        # Change in orientation
        del_yaw = goal_configuration[5] - start_configuration[5]
        traj_tf = dist / max_vel

        cos_angle_z = dist_xy / dist
        sin_angle_z = del_z / dist

        if dist_xy < 1e-9:
            cos_angle_xy = 1
            sin_angle_xy = 0
        else:
            cos_angle_xy = del_x / dist_xy
            sin_angle_xy = del_y / dist_xy

        # initialize message type
        new_state = js_list[j_idx-1] 
        # -- while loop through the list of joint states
        while not rospy.is_shutdown():
            time_elapsed = (rospy.get_rostime() - start_time)
            time_secs = time_elapsed.secs + time_elapsed.nsecs/1e9
            if(time_secs <= traj_tf):
                pt =  max_vel * time_secs
                new_state.header.stamp = rospy.get_rostime()

                # Write configuration of the robot to the message data
                # Note: traj_tf = 0 can cause a problem
                new_state.position = [start_configuration[0] + pt * cos_angle_z * cos_angle_xy, start_configuration[1] + pt * cos_angle_z * sin_angle_xy, start_configuration[2] + pt * sin_angle_z, goal_configuration[3], goal_configuration[4], start_configuration[5] +  time_secs * (del_yaw)/traj_tf]

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
