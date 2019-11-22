/*
 * 	A star 路径规划
 * 	spray0 2019.11.22
 */

#include "A_star.h"

//返回节点坐标类型
A_star_path::XY_t A_star_path::XY(unsigned int x, unsigned int y) {
	return {x,y};
}
//返回索引坐标类型
A_star_path::index_t A_star_path::XYW(unsigned int x, unsigned int y, unsigned int w) {
	return {XY(x,y),w};
}

//重载　用于索引一维到二维
signed char& operator>>(A_star_path::index_t index, std::vector<signed char> &data) {
	return data[index.node.x + index.node.y * index.jump_width];
}
//重载　用于比较两个节点
bool operator!=(A_star_path::XY_t a, A_star_path::XY_t b) {
	return ((a.x == b.x) && (a.y == b.y)) ? false : true;
}

//查询这个节点是否被占用
bool A_star_path::isOccupied(A_star_path::XY_t node) {
	return ((XYW(node.x, node.y, this->GridMap_width) >> this->GridData) == this->OCCUPIED) ? true : false;
}
//计算两个节点之间的直线距离
float A_star_path::distance_between(A_star_path::XY_t nodef, A_star_path::XY_t nodet) {
	return (float) sqrt((nodef.x - nodet.x) * (nodef.x - nodet.x) + (nodef.y - nodet.y) * (nodef.y - nodet.y));
}

//OPEN表插入项
A_star_path::open_t insert_open(unsigned char flag, unsigned int x, unsigned int y, unsigned int px, unsigned int py, float hn, float gn, float fn) {
	return {flag, x, y, px, py, hn, gn, fn};
}

//计算此节点期望表
void A_star_path::expand_array(float hn, A_star_path::XY_t node, A_star_path::XY_t target, A_star_path::XY_t max, std::vector<signed char> &map) {
	std::vector<A_star_path::expand_t> list;
	A_star_path::expand_t temp;
	int nowx = 0;
	int nowy = 0;
	for (char w = -1; w < 2; ++w) {
		for (char h = -1; h < 2; ++h) {
			if ((w != h) || (w != 0)) {
				nowx = w + node.x;
				nowy = h + node.y;
				if ((nowx >= 0) && (nowx <= max.x) && (nowy >= 0) && (nowy <= max.y)) {
					if (isOccupied(XY(nowx, nowy)) == false) {
						temp.x = nowx;
						temp.y = nowy;
						temp.hn = hn + distance_between(node, XY(nowx, nowy));
						temp.gn = distance_between(target, XY(nowx, nowy));
						temp.fn = temp.hn + temp.gn;
						list.push_back(temp);
					}
				}
			}
		}
	}
	A_star_path::expand_list.clear();
	A_star_path::expand_list = list;
	list.clear();
}

//返回open表内的最小项的引用
A_star_path::open_t& A_star_path::min_fn(std::vector<A_star_path::open_t> &open_list, A_star_path::XY_t Target) {
	int cnt = 0;
	float temp_fn = 0;
	int index = 0;
	for (auto open : open_list) {
		if (open.flag == 1) {
			if ((open.xval == Target.x) && (open.yval == Target.y)) {
				open_list[cnt].flag = 0;
				return open_list[cnt];
			}
			if ((open.fn < temp_fn) || (temp_fn == 0)) {
				temp_fn = open.fn;
				index = cnt;
			}
		}
		++cnt;
	}
	if (temp_fn == 0) {
		NoPath = false;
		return open_list[0];
	}
	open_list[index].flag = 0;
	return open_list[index];
}

//路径计算
bool A_star_path::Path_Calc(A_star_path::XY_t start, A_star_path::XY_t target, std::vector<signed char> griddata, int gridmap_width, int gridmap_height) {

	this->Start = start;
	this->Target = target;
	this->GridData = griddata;
	this->GridMap_width = gridmap_width;
	this->GridMap_height = gridmap_height;

	printf("[A*]	From [%d,%d] to [%d,%d].\n", this->Start.x, this->Start.y, this->Target.x, this->Target.y);
	NoPath = true;
	path_cost = 0;
	path_length = 0;

	Node = Start;
	float goal_distance = distance_between(Node, Target);
	printf("[A*]	LinearDistance: %.2f\n", goal_distance);

	open_list.clear();
	open_list.push_back(insert_open(0, Node.x, Node.y, Node.x, Node.y, path_cost, goal_distance, goal_distance));
	(XYW(Node.x, Node.y, this->GridMap_width) >> GridData) = OCCUPIED;

	MAX.x = GridMap_width - 1;
	MAX.y = GridMap_height - 1;

	//路径计算循环
	printf("[A*]	Start to calc...\n");
	while ((Node != Target) && NoPath) {
		//创建这个节点的期望表
		expand_array(path_cost, Node, Target, MAX, GridData);
		//对于期望表内的每一项
		for (auto expand : expand_list) {
			bool had = false;
			//对于open表内的每一项
			for (auto &open : open_list) {
				if ((expand.x == open.xval) && (expand.y == open.yval)) {
					open.fn = (open.fn < expand.fn) ? open.fn : expand.fn;
					if (open.fn == expand.fn) {
						open.parent_xval = Node.x;
						open.parent_yval = Node.y;
						open.hn = expand.hn;
						open.gn = expand.gn;
					}
					had = true;
				}
			}
			if (had == false)
				open_list.push_back(insert_open(1, expand.x, expand.y, Node.x, Node.y, expand.hn, expand.gn, expand.fn));

		}

		//更新
		open_t min_open = min_fn(open_list, Target);
		if (NoPath == true) {
			Node.x = min_open.xval;
			Node.y = min_open.yval;
			path_cost = min_open.hn;
			(XYW(Node.x, Node.y, this->GridMap_width) >> GridData) = OCCUPIED;
		} else {
			printf("No path!");
			return false;
		}

		//提取规划的路径长度
		if ((Node != Target) == false)
			path_length = min_open.fn;
	}

	//提取路径
	printf("[A*]	Analyse the path...\n");
	std::vector<XY_t> temp_path;
	temp_path.push_back(Target);
	XY_t lookup = Target;
	while (lookup != Start) {
		for (auto data : open_list) {
			//ROS_INFO("|%d|%3d|%3d|%3d|%3d|%.2f|%.2f|%.2f|", data.flag, data.xval, data.yval, data.parent_xval, data.parent_yval, data.hn, data.gn, data.fn);
			if ((data.xval == lookup.x) && (data.yval == lookup.y) && (data.flag == 0)) {
				//更新
				lookup.x = data.parent_xval;
				lookup.y = data.parent_yval;
				temp_path.push_back(lookup);
			}
		}
	}

	//将存储的路径转成正序
	mypath_list.clear();
	std::vector<XY_t>::iterator iter_end = temp_path.end();
	std::vector<XY_t>::iterator iter_begin = temp_path.begin();
	while (iter_end != iter_begin) {
		--iter_end;
		mypath_list.push_back(*iter_end);
	}

	printf("[A*]	Path length: %.2f steps: %ld\n", path_length, mypath_list.size());
	printf("[A*]	Path calc done.\n");
	return true;
}

