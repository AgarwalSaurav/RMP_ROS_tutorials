#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

visualization_msgs::Marker GetMarker(const YAML::Node& object, const int id = 0, const std::string frame_id = "base_link", const std::string ns = "basic_shapes") {
	visualization_msgs::Marker object_marker;
	object_marker.header.frame_id = frame_id;
	object_marker.ns = ns;
	object_marker.id = id;
	if(object["type"].as<std::string>() == "SPHERE") {
		object_marker.type = visualization_msgs::Marker::SPHERE;
	}
	if(object["type"].as<std::string>() == "CYLINDER") {
		object_marker.type = visualization_msgs::Marker::CYLINDER;
	}
	if(object["type"].as<std::string>() == "CUBE") {
		object_marker.type = visualization_msgs::Marker::CUBE;
	}

	auto scale = object["scale"];
	object_marker.scale.x = scale["x"].as<double>();
	object_marker.scale.y = scale["y"].as<double>();
	object_marker.scale.z = scale["z"].as<double>();

	auto position = object["position"];
	object_marker.pose.position.x = position["x"].as<double>();
	object_marker.pose.position.y = position["y"].as<double>();
	object_marker.pose.position.z = position["z"].as<double>();

	auto orientation = object["orientation"];
	object_marker.pose.orientation.x = orientation["x"].as<double>();
	object_marker.pose.orientation.y = orientation["y"].as<double>();
	object_marker.pose.orientation.z = orientation["z"].as<double>();
	object_marker.pose.orientation.w = orientation["w"].as<double>();

	auto color = object["color"];
	object_marker.color.r = color["r"].as<double>();
	object_marker.color.g = color["g"].as<double>();
	object_marker.color.b = color["b"].as<double>();
	object_marker.color.a = color["a"].as<double>();

	object_marker.action = visualization_msgs::Marker::ADD;
	object_marker.lifetime = ros::Duration();

	return object_marker;

}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "draw_environment3d");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_environment3d", 1);

	std::string file_name = argv[1];
  YAML::Node node = YAML::LoadFile(file_name);
	visualization_msgs::MarkerArray marker_array;

	int count = 1;

	auto objects = node["objects"];
  for (size_t i = 0; i < objects.size(); ++i) {
		auto object_marker = GetMarker(objects[i], i);
		marker_array.markers.push_back(object_marker);
	}

	while (ros::ok())
	{
		for(auto &marker:marker_array.markers) {
			marker.header.stamp = ros::Time::now();
		}

		// Publish the object_marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub.publish(marker_array);

		r.sleep();
	}
}
