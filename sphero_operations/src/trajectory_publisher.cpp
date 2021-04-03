#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <fstream>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_publisher_cpp");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Publisher pub;
	pub = n.advertise <sensor_msgs::JointState> ("/publish_joint_states", 5);
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"x", "y", "theta"};
	new_state.header.stamp = ros::Time::now();
	new_state.position = {0, 0, 0};
	auto time_elapsed = (ros::Time::now() - start);
	double time_secs = 0;

	double start_position[] = {0, 0};
	double goal_position[] = {3, 4};
	double del_x = goal_position[0] - start_position[0];
	double del_y = goal_position[1] - start_position[1];
	double dist_sqr = del_x * del_x + del_y * del_y;

	double dist = sqrt(dist_sqr);
	double max_vel = 0.5;
	double max_vel_sqr = max_vel * max_vel;
	double traj_a = -(16 * max_vel * max_vel_sqr)/(27 * dist_sqr);
	double traj_b = (4 * max_vel_sqr)/(3 * dist);
	double traj_c = 0;
	double traj_d = 0;
	double traj_tf = (3 * dist) / (2 * max_vel);

	double cos_angle = del_x / dist;
	double sin_angle = del_y / dist;
	while (ros::ok())
	{

		time_elapsed = (ros::Time::now() - start);
		time_secs = time_elapsed.toSec();
		if(time_secs < traj_tf) {
			double time_secs_sqr = time_secs * time_secs;
			new_state.header.stamp = ros::Time::now();
			double pt = traj_a * time_secs * time_secs_sqr + traj_b * time_secs_sqr + traj_c * time_secs + traj_d;
			new_state.position[0] = pt * cos_angle;
			new_state.position[1] = pt * sin_angle;
			new_state.position[2] = time_secs * atan2(del_x, del_y)/traj_tf;
			pub.publish(new_state);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
