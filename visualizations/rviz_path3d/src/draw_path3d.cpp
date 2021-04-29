#include <array>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <string>

visualization_msgs::Marker GetDefaultMarker() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::LINE_STRIP;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.6;

	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();

	return marker;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "draw_path");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_path", 1);

	std::string path_filename = argv[1];
	auto marker = GetDefaultMarker();

	std::ifstream infile(path_filename);
	geometry_msgs::Point pt;
	while(infile >> pt.x >> pt.y >> pt.z) {
		marker.points.push_back(pt);
	}
	infile.close();

	while (ros::ok())
	{
		marker.header.stamp = ros::Time::now();

		// Publish the marker_polygon_
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub.publish(marker);

		r.sleep();
	}
}
