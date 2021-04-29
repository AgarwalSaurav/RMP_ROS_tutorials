#ifndef RVIZ_GRAPH__GRAPH_H
#define RVIZ_GRAPH__GRAPH_H


#include <string>
#include <fstream>
#include <unordered_map>
#include <geometry_msgs/Point.h>

struct Vertex {
	size_t id;
	double x;
	double y;
	double z;

	Vertex() { id = 0; x = 0; y = 0; z = 0; }
	Vertex(size_t id_in, double x_in, double y_in, double z_in) : id(id_in), x(x_in), y(y_in), z(z_in) {}
	Vertex(double x_in, double y_in, double z_in) : id(0), x(x_in), y(y_in), z(z_in) {}
};

struct Edge {
	size_t u_id, v_id;
	Edge() = default;
	Edge(const size_t &u_in, const size_t &v_in) : u_id(u_in), v_id(v_in) {}
};

class Graph {
	std::vector <Vertex> vertex_list_;
	std::vector <Edge> edge_list_;
	std::unordered_map <size_t, size_t> vertex_map_; /*! Stores a map of the index of vertices to actual ID of the node <ID, index>*/

	public:
	Graph(const std::string &vertex_list_filename, const std::string &edge_list_filename) {
		VertexParser(vertex_list_filename);
		EdgeParser(edge_list_filename);
	}

	void VertexParser (const std::string &vertex_list_filename) {
		std::ifstream infile(vertex_list_filename);
		size_t id;
		double x, y, z;
		while(infile >> id >> x >> y >> z) {
			Vertex new_vertex(id, x, y, z);
			vertex_list_.push_back(new_vertex);
			vertex_map_[new_vertex.id] = vertex_list_.size() - 1;
		}
		infile.close();
	}

	bool EdgeParser (const std::string &edge_list_filename) {
		std::ifstream infile(edge_list_filename);
		size_t u, v;
		while(infile >> u >> v) {
			Edge new_edge(u, v);
			edge_list_.push_back(new_edge);
			if(IsVertexInGraph(u) == false or IsVertexInGraph(v) == false) {
				infile.close();
				std::cerr << "Vertex not found\n";
				return 1;
			}
		}
		infile.close();
		return 0;
	}


	bool IsVertexInGraph (const size_t ID) const {
		auto search_vertex = vertex_map_.find(ID);
		if(search_vertex == vertex_map_.end())
			return false;
		else
			return true;
	}

	void GetVertexList(std::vector<geometry_msgs::Point> &point_list) {
		point_list.clear();
		for(const auto &v:vertex_list_) {
			geometry_msgs::Point pt;
			pt.x = v.x;
			pt.y = v.y;
			pt.z = v.z;
			point_list.push_back(pt);
		}
	}

	void GetEdgeList(std::vector<geometry_msgs::Point> &edge_list) {
		for(const auto &e:edge_list_) {
			geometry_msgs::Point pt_u, pt_v;
			auto u = vertex_list_[vertex_map_[e.u_id]];
			auto v = vertex_list_[vertex_map_[e.v_id]];

			pt_u.x = u.x;
			pt_u.y = u.y;
			pt_u.z = u.z;
			pt_v.x = v.x;
			pt_v.y = v.y;
			pt_v.z = v.z;
			edge_list.push_back(pt_u);
			edge_list.push_back(pt_v);
		}
	}
};

#endif /* RVIZ_GRAPH__GRAPH_H */
