/*
 * 	A star 路径规划
 * 	spray0 2019.11.22
 */
#ifndef A_STAR_H
#define A_STAR_H

#include <cmath>
#include <vector>
#include <iostream>

class A_star_path {
public:
	bool NoPath = true;
	float path_cost = 0;
	float path_length=0;
	int GridMap_width = 0;
	int GridMap_height = 0;
	unsigned char OCCUPIED = 100;
	unsigned char IDLE = 0;

	struct XY_t {
		unsigned int x;
		unsigned int y;
	}Start,Target,Node,MAX;
	struct index_t{
		XY_t node;
		unsigned int jump_width;
	};
	struct open_t {
		unsigned char flag = 1;
		unsigned int xval;
		unsigned int yval;
		unsigned int parent_xval;
		unsigned int parent_yval;
		float hn;
		float gn;
		float fn;
	};
	struct expand_t {
		unsigned int x;
		unsigned int y;
		float hn;
		float gn;
		float fn;
	};

	XY_t XY(unsigned int x,unsigned int y);
	index_t XYW(unsigned int x, unsigned int y,unsigned w);
	std::vector<signed char> GridData;
	std::vector<open_t> open_list;
	std::vector<expand_t> expand_list;
	std::vector<XY_t> mypath_list;
	std::vector<XY_t> path_list;
	friend signed char& operator>>(A_star_path::index_t index, std::vector<signed char> &data);
	bool Path_Calc_Raw(XY_t start,XY_t target,std::vector<signed char> griddata,int gridmap_width,int gridmap_height);
	bool Path_Calc_Optimize(XY_t start,XY_t target,std::vector<signed char> griddata,int gridmap_width,int gridmap_height);
private:
	bool isOccupied(XY_t node);
	void expand_array(float hn, XY_t node, XY_t target, XY_t max, std::vector<signed char> &map);
	float distance_between(XY_t nodef, XY_t nodet);
	double Calc_Theta_a2b(XY_t a,XY_t b);
	bool isOccupied_a2b(A_star_path::XY_t a,A_star_path::XY_t b);
	open_t& min_fn(std::vector<open_t> &open_list, XY_t Target);
};

#endif
