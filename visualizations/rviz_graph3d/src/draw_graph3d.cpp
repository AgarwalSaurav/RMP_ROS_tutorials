#include <array>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <rviz_graph3d/graph3d.h>

visualization_msgs::Marker GetDefaultMarker() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::POINTS;

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
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();

	return marker;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "draw_graph");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub_vertices = n.advertise<visualization_msgs::Marker>("visualization_graph/vertices", 1);
	ros::Publisher marker_pub_edges = n.advertise<visualization_msgs::Marker>("visualization_graph/edges", 1);

	std::string vertex_filename = argv[1];
	std::string edge_filename = argv[2];
	Graph graph(vertex_filename, edge_filename);
	std::vector<geometry_msgs::Point> vertex_list;
	std::vector<geometry_msgs::Point> edge_list;
	graph.GetVertexList(vertex_list);
	graph.GetEdgeList(edge_list);

	auto marker_vertices = GetDefaultMarker();
	auto marker_edges = GetDefaultMarker();
	marker_edges.type = visualization_msgs::Marker::LINE_LIST;

	marker_vertices.points = vertex_list;
	marker_edges.points = edge_list;
	marker_edges.scale.x = 0.02;
	marker_edges.scale.y = 0.02;


	while (ros::ok())
	{
		marker_vertices.header.stamp = ros::Time::now();

		// Publish the marker_polygon_
		while (marker_pub_vertices.getNumSubscribers() < 1 or marker_pub_edges.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub_vertices.publish(marker_vertices);
		marker_pub_edges.publish(marker_edges);

		r.sleep();
	}
}
