/*
 * 	A star 路径规划
 * 	spray0 2020.1.7
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

//计算节点a指向节点b的角度
double A_star_path::Calc_Theta_a2b(XY_t a,XY_t b){
	double dl = A_star_path::distance_between(a, b);
	double tx = (double)a.x - (double)b.x;
	double ty = (double)a.y - (double)b.y;
	double theta = asinf(double(ty / dl));

	if      ((tx < 0) && (ty > 0))   theta = 3.14159 - theta;			
	else if ((tx < 0) && (ty < 0))   theta = -theta - 3.14159;

	theta=(double)(int)(theta*100)/100.f;
	return theta;		
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
bool A_star_path::Path_Calc_Raw(A_star_path::XY_t start, A_star_path::XY_t target, std::vector<signed char> griddata, int gridmap_width, int gridmap_height) {

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

	printf("[A*]    Path length: %.2f \n", path_length);
	printf("[A*]	Path calc done.\n");
	return true;
}

// 检查两个节点之间是否占用
bool A_star_path::isOccupied_a2b(A_star_path::XY_t a,A_star_path::XY_t b){

	std::vector<A_star_path::XY_t> maybe;
	int dx=b.x-a.x;
	int dy=b.y-a.y;
	A_star_path::XY_t add;

	if((dx==0)&&(dy!=0)){
		if(dy>0)
			for(int c=1;c<=dy;++c)	maybe.push_back(XY(a.x+0,a.y+c));
		else
			for(int c=dy;c<0;++c)	maybe.push_back(XY(a.x+0,a.y+c));
	}
	else if((dy==0)&&(dx!=0)){
		if(dx>0)
			for(int c=1;c<=dx;++c)	maybe.push_back(XY(a.x+c,a.y+0));
		else
			for(int c=dx;c<0;++c)	maybe.push_back(XY(a.x+c,a.y+0));
	}
	else if((dx!=0)&&(dy!=0)){
		if(dx>0)
			for(int c=1;c<=dx;++c)	maybe.push_back(XY(a.x+c,(dy>0)?a.y+c:a.y-c));
		else
			for(int c=dx;c<0;++c)	maybe.push_back(XY(a.x+c,(dy<0)?a.y+c:a.y-c));	
	}

	bool isOC=false;

	for(auto tp:maybe)	isOC=isOccupied(tp)?true:isOC;	//检查每个数据是否占用
	return isOC;
}

// 路径计算 + 优化
// 简洁节点描述、路径最优下减少转向次数
bool A_star_path::Path_Calc_Optimize(A_star_path::XY_t start, A_star_path::XY_t target, std::vector<signed char> griddata, int gridmap_width, int gridmap_height) {
	
	path_list.clear();
	// 计算路径
	if(!A_star_path::Path_Calc_Raw(start,target,griddata,gridmap_width,gridmap_height))
		return false;

	this->GridData = griddata;
	this->GridMap_width = gridmap_width;
	this->GridMap_height = gridmap_height;
	std::vector<A_star_path::XY_t> rp_list; // 转向节点表

	printf("[A*]    Optimizing Path...\n");
	// 简洁描述路径优化
	{
		rp_list.push_back(mypath_list[0]);
		double k1,k2;
		// 判断出转点 并添加
		for(int cnt=1;cnt<mypath_list.size()-1;++cnt)
		{
			k1=Calc_Theta_a2b(mypath_list[cnt-1],mypath_list[cnt]);
			k2=Calc_Theta_a2b(mypath_list[cnt],mypath_list[cnt+1]);
			//printf("%.4f,%.4f\r\n",k1,k2);
			if(k1!=k2)
				rp_list.push_back(mypath_list[cnt]);
		}
			
		rp_list.push_back(mypath_list[mypath_list.size()-1]);
	}

	//for(auto data:rp_list) printf("rp_list[%d,%d]\r\n",data.x,data.y);
	// 路径优化(必要优化) 减少转向
	{
		std::vector<bool> rp_flag(rp_list.size(),false); // 路径标识

		//遍历节点
		for(int cnt=0;cnt<rp_list.size()-2;++cnt){
			double temp1,temp2;
			//printf("now check [%d,%d]\r\n",rp_list[cnt].x,rp_list[cnt].y);
			if(cnt==rp_list.size()-3){
				temp1=Calc_Theta_a2b(rp_list[cnt-1],rp_list[cnt]);
				temp2=Calc_Theta_a2b(rp_list[cnt+1],rp_list[cnt+2]);
			}else{
				temp1=Calc_Theta_a2b(rp_list[cnt],rp_list[cnt+1]);
				temp2=Calc_Theta_a2b(rp_list[cnt+2],rp_list[cnt+3]);
			}

			if(temp1!=temp2)	rp_flag[cnt]=true;
			else if(!rp_flag[cnt]){	// 如果没有处理过
				
				// 判断 
				A_star_path::XY_t new_tp,new_tp2;
				new_tp.x=rp_list[cnt+2].x-rp_list[cnt+1].x+rp_list[cnt].x;
				new_tp.y=rp_list[cnt+2].y-rp_list[cnt+1].y+rp_list[cnt].y;
				new_tp2.x=rp_list[cnt+1].x-rp_list[cnt].x+new_tp.x;
				new_tp2.y=rp_list[cnt+1].y-rp_list[cnt].y+new_tp.y;
		
				if((isOccupied_a2b(rp_list[cnt],new_tp))||(isOccupied_a2b(new_tp,rp_list[cnt+2]))){
					rp_flag[cnt]=true;
				}else{
					rp_list[cnt]=new_tp;

					if((cnt!=rp_list.size()-3)&&(cnt!=0)){
						rp_list.erase(rp_list.begin()+cnt+1,rp_list.begin()+cnt+3);
						rp_flag.erase(rp_flag.begin()+cnt+1,rp_flag.begin()+cnt+3);
						--cnt;
					}
					else if(cnt==0){
						rp_list.erase(rp_list.begin()+cnt+1,rp_list.begin()+cnt+3);
						rp_flag.erase(rp_flag.begin()+cnt+1,rp_flag.begin()+cnt+3);
						rp_list.insert(rp_list.begin(),mypath_list[0]);
						rp_flag.insert(rp_flag.begin(),false);
						--cnt;
					}
					else{
						rp_list.erase(rp_list.begin()+cnt+1,rp_list.begin()+cnt+2);
						rp_flag.erase(rp_flag.begin()+cnt+1,rp_flag.begin()+cnt+2);
					}
				}
			}
		}
	}
	path_list=rp_list;
	printf("[A*]    Done.\n");
	return true;
}
