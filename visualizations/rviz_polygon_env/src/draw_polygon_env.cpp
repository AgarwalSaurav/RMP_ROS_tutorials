#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_polygon_env/env.h>

struct PolygonMarker {
	public:
	visualization_msgs::Marker marker_polygon_;
	PolygonMarker(const int id = 0, const std::string frame_id = "base_link", const std::string ns = "basic_shapes") {
		uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker_polygon_.header.frame_id = frame_id;

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker_polygon_.ns = ns;
		marker_polygon_.id = id;

		marker_polygon_.type = shape;

		marker_polygon_.scale.x = 0.05;
		marker_polygon_.pose.position.x = 0;
		marker_polygon_.pose.position.y = 0;
		marker_polygon_.pose.position.z = 0;
		marker_polygon_.pose.orientation.x = 0.0;
		marker_polygon_.pose.orientation.y = 0.0;
		marker_polygon_.pose.orientation.z = 0.0;
		marker_polygon_.pose.orientation.w = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		marker_polygon_.color.r = 0.0f;
		marker_polygon_.color.g = 0.0f;
		marker_polygon_.color.b = 1.0f;
		marker_polygon_.color.a = 1.0;

		// Set the marker_polygon action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker_polygon_.action = visualization_msgs::Marker::ADD;

		marker_polygon_.lifetime = ros::Duration();
	}

	void SetPoints(const Polygon &poly) {
		marker_polygon_.points = poly;
		marker_polygon_.points.push_back(marker_polygon_.points.front());
	}

	void SetRGBA(float r, float g, float b, float a = 1.0f) {
		marker_polygon_.color.r = r;
		marker_polygon_.color.g = g;
		marker_polygon_.color.b = b;
		marker_polygon_.color.a = a;
	}

};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "draw_polygon_env");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_env_polygons", 1);

	std::string filename = argv[1];
	Env env;
	env.ReadPolygonFromFile(filename);

	visualization_msgs::MarkerArray marker_array;

	PolygonMarker marker;
	marker.SetRGBA(0, 0, 0);
	marker.SetPoints(env.outer_polygon_);
	marker_array.markers.push_back(marker.marker_polygon_);

	int count = 1;
	for(const auto &obstacle:env.obstacles_) {
		PolygonMarker marker(count++);
		marker.SetPoints(obstacle);
		marker.SetRGBA(1, 0, 0);
		marker_array.markers.push_back(marker.marker_polygon_);
	}

	while (ros::ok())
	{
		for(auto &marker:marker_array.markers) {
			marker.header.stamp = ros::Time::now();
		}

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
		marker_pub.publish(marker_array);

		r.sleep();
	}
}
