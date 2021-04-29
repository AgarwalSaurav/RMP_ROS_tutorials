/* Create a node that publishes location of the robot */
/* Visualize the robot in rviz to see how the location is changing */
/* In this example a cubic trajectory is used to travel between two points */
/* This is for a 3 DoF planar robot */

#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <fstream>

int main(int argc, char **argv)
{
	/* Name of the node */
	ros::init(argc, argv, "trajectory_publisher_cpp");
	ros::NodeHandle n;
	/* Frequency at which the data will be published */
	ros::Rate loop_rate(30);

	/* publish_joint_states is the topic to which the data needs to be published */
	/* This is the topic the robot will subscribe to and change its position */
	ros::Publisher pub;
	pub = n.advertise <sensor_msgs::JointState> ("/publish_joint_states", 5);

	/* Message type */
	sensor_msgs::JointState new_state;
	new_state.name = {"x", "y", "theta"};
	new_state.header.stamp = ros::Time::now();
	new_state.position = {0, 0, 0};

	/* Get starting position of the robot */
	std::string joint_state_topic = "/joint_states";
	auto start_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic);
	auto start_configuration = start_state_ptr->position;

	/* Sample goal location */
	double goal_configuration[] = {5, 0, M_PI};

  /* Change in orientation */
	double del_theta = goal_configuration[2] - start_configuration[2];

	double del_x = goal_configuration[0] - start_configuration[0];
	double del_y = goal_configuration[1] - start_configuration[1];
	double dist_sqr = del_x * del_x + del_y * del_y;
	double dist = sqrt(dist_sqr);
	double max_vel = 0.5;

	/* Constant velocity */
	/* */
	double traj_a = 0;
	double traj_b = 0;
	double traj_c = max_vel;
	double traj_d = 0;
	double traj_tf = dist / max_vel;
	/* */

	/* Cubic trajectory */
	/*
	double max_vel_sqr = max_vel * max_vel;
	double traj_a = -(16 * max_vel * max_vel_sqr)/(27 * dist_sqr);
	double traj_b = (4 * max_vel_sqr)/(3 * dist);
	double traj_c = 0;
	double traj_d = 0;
	double traj_tf = (3 * dist) / (2 * max_vel);
  */

	double cos_angle = del_x / dist;
	double sin_angle = del_y / dist;

	/* Get current time */
	ros::Time start = ros::Time::now();
	auto time_elapsed = (ros::Time::now() - start);
	double time_secs = 0;

	while (ros::ok())
	{
		time_elapsed = (ros::Time::now() - start);
		time_secs = time_elapsed.toSec();
		new_state.header.stamp = ros::Time::now();
		if(time_secs <= traj_tf) {
			double time_secs_sqr = time_secs * time_secs;
			double pt = traj_a * time_secs * time_secs_sqr + traj_b * time_secs_sqr + traj_c * time_secs + traj_d;
			/* Write position of the robot to the message data */
			new_state.position[0] = start_configuration[0] + pt * cos_angle;
			new_state.position[1] = start_configuration[1] + pt * sin_angle;
			new_state.position[2] = start_configuration[2] + time_secs * del_theta/traj_tf;
		}
		else {
			/* Write position of the robot to the message data */
			new_state.position[0] = goal_configuration[0];
			new_state.position[1] = goal_configuration[1];
			new_state.position[2] = goal_configuration[2];
		}
		/* Send the message */
		pub.publish(new_state);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
