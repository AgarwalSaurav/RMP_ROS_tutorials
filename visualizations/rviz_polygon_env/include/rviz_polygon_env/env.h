#ifndef RVIZ_POLYGON_ENV__ENV_H_
#define RVIZ_POLYGON_ENV__ENV_H_

#include <geometry_msgs/Point.h>
#include <vector>
#include <fstream>
#include <ros/ros.h>

typedef std::vector<geometry_msgs::Point> Polygon;

struct Env {
	public:
		Polygon outer_polygon_;
		std::vector <Polygon> obstacles_;
		void ReadPolygonFromFile(const std::string &filename) {

			std::ifstream infile (filename);
			if(infile.is_open() == false) {
				std::cerr << "Cannot open file " << filename << ". Is the path correct?" << std::endl;
				return;
			}
			geometry_msgs::Point pt;
			pt.z = 0;
			std::string s;
			while(std::getline(infile, s)) {
				if(s.empty())
					break;
				std::istringstream tmp(s);
				tmp >> pt.x >> pt.y;
				outer_polygon_.push_back(pt);
			}

			Polygon obstacle;
			bool flag = 0;
			while (std::getline(infile, s)) {
				if (s.empty() && flag == 1) {
					obstacles_.push_back(obstacle);
					obstacle.clear();
					flag = 0;
				}
				else {
					std::istringstream tmp(s);
					tmp >> pt.x >> pt.y;
					obstacle.push_back(pt);
					flag = 1;
				}
			}
			obstacles_.push_back(obstacle);
			infile.close();
		}
};

#endif /* RVIZ_POLYGON_ENV__ENV_H_ */
