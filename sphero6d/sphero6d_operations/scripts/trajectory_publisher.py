#!/usr/bin/env python3
# Create a node that publishes location of the robot
# Visualize the robot in rviz to see how the location is changing
# In this example a cubic trajectory is used to travel between two points
# This is for a 3 DoF planar robot
import rospy
import math
from math import sin, cos, sqrt, atan2
from sensor_msgs.msg import JointState

def talker():

    # publish_joint_states is the topic to which the data needs to be published
    # This is the topic the robot will subscribe to and change its configuration
    pub = rospy.Publisher('/publish_joint_states', JointState, queue_size=10)
    # Name of the node
    rospy.init_node('trajectory_publisher', anonymous=True)
    # Frequency at which the data will be published
    rate = rospy.Rate(30) # 30hz

    # Get the configuration of the robot
    start_state = rospy.wait_for_message('/joint_states', JointState)
    start_configuration = start_state.position;

    # Message type
    new_state = start_state

    # sample goal location
    goal_configuration = [5, 4, 5, 0, 0, math.pi]

    del_x = goal_configuration[0] - start_configuration[0]
    del_y = goal_configuration[1] - start_configuration[1]
    del_z = goal_configuration[2] - start_configuration[2]
    dist_xy_sqr = del_x * del_x + del_y * del_y
    dist_sqr = dist_xy_sqr + del_z * del_z

    dist = sqrt(dist_sqr)
    dist_xy = sqrt(dist_xy_sqr)
    max_vel = 0.5

    # Change in orientation
    del_yaw = goal_configuration[5] - start_configuration[5]

    # Calculations for constant velocity trajectory
    traj_a = 0
    traj_b = 0
    traj_c = max_vel
    traj_d = 0
    traj_tf = dist / max_vel

    # Calculations for cubic trajectory
    # max_vel_sqr = max_vel * max_vel
    # traj_a = -(16 * max_vel * max_vel_sqr)/(27 * dist_sqr)
    # traj_b = (4 * max_vel_sqr)/(3 * dist)
    # traj_c = 0
    # traj_d = 0
    # traj_tf = (3 * dist) / (2 * max_vel)

    # Log data
    rospy.loginfo("Total distance: %f", dist)
    rospy.loginfo("Trajectory time: %f", traj_tf)

    cos_angle_z = dist_xy / dist
    sin_angle_z = del_z / dist

    if dist_xy < 1e-9:
        cos_angle_xy = 1
        sin_angle_xy = 0
    else:
        cos_angle_xy = del_x / dist_xy

    # Get current time
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown():
        time_elapsed = (rospy.get_rostime() - start_time)
        time_secs = time_elapsed.secs + time_elapsed.nsecs/1e9
        time_secs_sqr = time_secs * time_secs
        if(time_secs <= traj_tf):
            pt = traj_a * time_secs * time_secs_sqr + traj_b * time_secs_sqr + traj_c * time_secs + traj_d
            new_state.header.stamp = rospy.get_rostime()

            # Write configuration of the robot to the message data
            # Note: traj_tf = 0 can cause a problem
            new_state.position = [start_configuration[0] + pt * cos_angle_z * cos_angle_xy, start_configuration[1] + pt * cos_angle_z * sin_angle_xy, start_configuration[2] + pt * sin_angle_z, goal_configuration[3], goal_configuration[4], start_configuration[5] +  time_secs * (del_yaw)/traj_tf]

        else:
            new_state.header.stamp = rospy.get_rostime()
            new_state.position = goal_configuration

        # Send the message
        pub.publish(new_state);
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
