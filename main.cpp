

#include <opencv2/opencv.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdio.h>

#include "./cc_rs.h"
#include "./reeds_shepp.h"
#include "./glog/glog/logging.h"
#include "./gflags/gflags.h"
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <ctime>
#include <algorithm>
using namespace google;

//#pragma comment(lib, "glogd.lib")

//#pragma comment(lib, "./glogd.lib")
using namespace  cv;
using namespace  std;

#define GLOG_NO_ABBREVIATED_SEVERITIES 

#define VERTICAL     1
#define PARALLEL     2
#define INCLINE      3

#define ParkingMode  PARALLEL

#define RANDOM_ANGLE 10
#define CarLength   400
#define CarWidth    170

#define MapHight    1500
#define MapWidth    2000

#define MAP_SCALE    0.5//1


#define random(a,b) (rand()%(b-a+1)+a)
#define ParkWidthVer  250
#define ParkHightVer  500

#define ParkWidthPara  250
#define ParkHightPara  630
clock_t start_time, end_time;

const float  step_size = 30;
const float  rho = 400; // 430turning radius


vector <Point> RotateCar( Posture& RotateCenter, double WheelBase,double CarLen,double CarWid){
	
	Point CarCornerOld1, CarCornerOld2, CarCornerOld3, CarCornerOld4;
	vector <cv::Point> CarCorner;
	//41
	//->
	//32
	CarCornerOld1.x = RotateCenter.x + CarLen - WheelBase;
	CarCornerOld2.x = RotateCenter.x + CarLen - WheelBase;
	CarCornerOld4.x = RotateCenter.x - WheelBase;
	CarCornerOld3.x = RotateCenter.x - WheelBase;
	CarCornerOld1.y = RotateCenter.y - CarWid / 2;
	CarCornerOld2.y = RotateCenter.y + CarWid / 2;
	CarCornerOld4.y = RotateCenter.y - CarWid / 2;
	CarCornerOld3.y = RotateCenter.y + CarWid / 2;

	Point CarCorner1, CarCorner2, CarCorner3, CarCorner4;
	//ÈÆÒ»¸ö×ø±êµã(rx0,ry0)
	double a =  RotateCenter.yaw;
	CarCorner1.x = (CarCornerOld1.x - RotateCenter.x)*cos(a) - (CarCornerOld1.y - RotateCenter.y)*sin(a) + RotateCenter.x;
	CarCorner1.y = (CarCornerOld1.x - RotateCenter.x)*sin(a) + (CarCornerOld1.y - RotateCenter.y)*cos(a) + RotateCenter.y;	
	CarCorner.push_back(CarCorner1);

	CarCorner2.x = (CarCornerOld2.x - RotateCenter.x)*cos(a) - (CarCornerOld2.y - RotateCenter.y)*sin(a) + RotateCenter.x;
	CarCorner2.y = (CarCornerOld2.x - RotateCenter.x)*sin(a) + (CarCornerOld2.y - RotateCenter.y)*cos(a) + RotateCenter.y;
	CarCorner.push_back(CarCorner2);

	CarCorner3.x = (CarCornerOld3.x - RotateCenter.x)*cos(a) - (CarCornerOld3.y - RotateCenter.y)*sin(a) + RotateCenter.x;
	CarCorner3.y = (CarCornerOld3.x - RotateCenter.x)*sin(a) + (CarCornerOld3.y - RotateCenter.y)*cos(a) + RotateCenter.y;
	CarCorner.push_back(CarCorner3);

	CarCorner4.x = (CarCornerOld4.x - RotateCenter.x)*cos(a) - (CarCornerOld4.y - RotateCenter.y)*sin(a) + RotateCenter.x;
	CarCorner4.y = (CarCornerOld4.x - RotateCenter.x)*sin(a) + (CarCornerOld4.y - RotateCenter.y)*cos(a) + RotateCenter.y;
	CarCorner.push_back(CarCorner4);
	
	return CarCorner;

}

void rotate_point(Point& src, Posture& RotateCenter, Point& dst ){
	
	//point_list = [p1, p2, p4, p3, p1]
	/*dst.x = (src.x - RotateCenter.x)*cos(RotateCenter.yaw) - (src.y - RotateCenter.y)*sin(RotateCenter.yaw) + RotateCenter.y;
	dst.y = (src.x - RotateCenter.x)*sin(RotateCenter.yaw) + (src.y - RotateCenter.y)*cos(RotateCenter.yaw) + RotateCenter.y;*/
	return;

}

vector<cv::Point>& get_corner(vector<cv::Point>& CarCorner, Posture& CarCenter) {

	Point p1(CarCenter.x - CarLength / 2, CarCenter.y - CarWidth / 2);
	Point p2(CarCenter.x - CarLength / 2, CarCenter.y + CarWidth / 2);
	Point p3(CarCenter.x + CarLength / 2, CarCenter.y + CarWidth / 2);
	Point p4(CarCenter.x + CarLength / 2, CarCenter.y - CarWidth / 2);

	Point p1_rota, p2_rota, p3_rota, p4_rota;
	
	rotate_point(p1, CarCenter,p1_rota);
	rotate_point(p2, CarCenter, p2_rota);
	rotate_point(p3, CarCenter, p3_rota);
	rotate_point(p4, CarCenter, p4_rota);
	CarCorner.push_back(p1_rota);
	CarCorner.push_back(p2_rota);
	CarCorner.push_back(p3_rota);
	CarCorner.push_back(p4_rota);

	return CarCorner;
}




// ¼ÆËã |p1 p2| X |p1 p|
double GetCross(Point& p1, Point& p2, Point& p)
{
	double temp = (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
	return temp;
}

bool CarCornerCheck(vector<Posture>& objectlist, Posture& carCenter){
	//-----p1-------p4-------
	//-----|   p->   |-------
	//-----p2-------p3-------

	//carCenter.x = 200;
	//carCenter.y = 300;
	//carCenter.yaw = 0;

	//RotatedRect rRect(Point2f(carCenter.x, carCenter.y), Size2f(180 * MAP_SCALE, 420 * MAP_SCALE), (90) + 57.6*carCenter.yaw);
	//¶¨Òå¾ØÐÎµÄ4¸ö¶¥µã	
	//Point2f vertices[4];
	//¼ÆËã¾ØÐÎµÄ4¸ö¶¥µã	
	//rRect.points(vertices);
	vector<Point> car_corner;
	car_corner = RotateCar(carCenter, 110 * MAP_SCALE, CarLength * MAP_SCALE, CarWidth * MAP_SCALE);
	/*Point p4(car_corner[0].x, car_corner[0].y);
	Point p3(car_corner[1].x, car_corner[1].y);
	Point p2(car_corner[2].x, car_corner[2].y);
	Point p1(car_corner[3].x, car_corner[3].y);*/
	Point p4(car_corner[0].x, car_corner[0].y);
	Point p3(car_corner[1].x, car_corner[1].y);
	Point p2(car_corner[2].x, car_corner[2].y);
	Point p1(car_corner[3].x, car_corner[3].y);
	bool safe = false;
	

	/*Point p2(vertices[0].x, vertices[0].y);
	Point p3(vertices[1].x, vertices[1].y);
	Point p4(vertices[2].x, vertices[2].y);
	Point p1(vertices[3].x, vertices[3].y);*/

	/*Point p2(150, 250);
	Point p3(250, 250);
	Point p4(250, 350);
	Point p1(150, 350);*/
	Point objectxy;
	for (int i = 0; i < objectlist.size(); ++i){
		objectxy.x = objectlist[i].x;
		objectxy.y = objectlist[i].y;
		//Point objectxy(100, 100);//wupengzhuang
		//Point objectxy(200, 300);//pengzhuang
		safe = (GetCross(p1, p2, objectxy) * GetCross(p3, p4, objectxy) >= 0) && (GetCross(p2, p3, objectxy) * GetCross(p4, p1, objectxy) >= 0);
		//std::cout << "safe " << std::endl;
		if (safe == true){
			//std::cout << "true | center.xy " << carCenter.x << " ; " << carCenter.y << carCenter.yaw << std::endl;
			//std::cout << "true | objectlist[i].xy= " << objectlist[i].x << " ; " << objectlist[i].y << std::endl;
			return true;//ÎÞÅö×²
		}
	}
	return false;//Åö×²
}


bool PointCarCollisionCheck(vector<Posture>& objectlist,Posture& carCenter){	
	//-----p1-------p4-------
	//-----|   p->   |-------
	//-----p2-------p3-------
	bool safe = false;
	Point p1(carCenter.x - CarLength / 2, carCenter.y - CarWidth / 2);
	Point p2(carCenter.x - CarLength / 2, carCenter.y + CarWidth / 2);
	Point p3(carCenter.x + CarLength / 2, carCenter.y + CarWidth / 2);
	Point p4(carCenter.x + CarLength / 2, carCenter.y - CarWidth / 2);
	for (int i = 0; i < objectlist.size();++i){
		Point objectxy(objectlist[i].x, objectlist[i].y);
		safe = GetCross(p1, p2, objectxy) * GetCross(p3, p4, objectxy) >= 0 &&
				GetCross(p2, p3, objectxy) * GetCross(p4, p1, objectxy) >= 0;
		if (safe == false){
			return false;
		}
	}
	return safe;//true
}



class RRT{

public:
	//RRT();
	//~RRT(){};
	void Init(Posture q0, Posture q1, int minX, int minY, int maxX, int maxY, vector<Posture> obstacles){
		start = q0;
		goal = q1;
		minrandX = minX;
		minrandY = minY;

		maxrandX = maxX;
		maxrandY = maxY;
		obstacleList = obstacles;
	};
	vector<Posture> path;
	vector<Posture>  path_list_;
private:

	int  minrandX, minrandY;
	int  maxrandX, maxrandY;
	int  goalSampleRate = 95 ;
	int  maxIter        = 150;
	double rnd[3];
	
	vector<Posture> obstacleList;
	vector <Posture> nodeList;
	
	Posture newNode_;
	ReedsSheppPathGenerator  path_point;
	Posture start, goal;
	
public:
	//return node
	Posture GetRandomPointVertical(){
		Posture node;
		double rnd[4] = { 0 };
		srand((unsigned)time(NULL));
		if (random(1, 100) > goalSampleRate){
			rnd[0] = 1.0* random(minrandX, maxrandX);
			rnd[1] = 1.0* random(minrandY, maxrandY);
			rnd[2] = 0.01*(random(314, 628)-314);
			rnd[3] = 2;
		}
		else if (random(1, 100) < 1){
			rnd[0] = (goal.x + start.x) / 2;
			rnd[1] = (goal.y + start.y) / 2;
			rnd[2] = (goal.yaw + start.yaw) / 2;
			rnd[3] = (goal.speed + start.speed) / 2;

			
		}
		else{ // # goal point sampling
			rnd[0] = goal.x;
			rnd[1] = goal.y;
			rnd[2] = goal.yaw;
			rnd[3] = goal.speed;
		}
		//node = rnd;
		node.x = rnd[0];
		node.y = rnd[1];
		node.yaw = rnd[2];
		node.speed = rnd[3];
		node.cost = 0;
		return node;
	}
	Posture GetRandomPointParallel(){
		Posture node;
		double rnd[4] = { 0 };
		srand((unsigned)time(NULL));
		if (random(1, 100) > 95){
			rnd[0] = 1.0* random(minrandX, maxrandX);
			rnd[1] = 1.0* random(minrandY, maxrandY);
			rnd[2] = 0.01*(random(314, 628) - 314);
			rnd[3] = 2;
		}
		else if (random(1, 100) < 92){
			////Posture guidPoint;
			rnd[0] = goal.x + (ParkWidthPara*1.2   + random(1, 30))* MAP_SCALE;// (790 - random(1, 50))* MAP_SCALE;
			rnd[1] = goal.y - (ParkHightPara*0.3 + random(1, 20))* MAP_SCALE;//(300 - random(1,30)) * MAP_SCALE;//
			rnd[2] = - pi / 6;
			rnd[3] = -0.2;


		}
		else{ // # goal point sampling
			rnd[0] = goal.x;
			rnd[1] = goal.y;
			rnd[2] = goal.yaw;
			rnd[3] = goal.speed;
		}
		//node = rnd;
		node.x = rnd[0];
		node.y = rnd[1];
		node.yaw = rnd[2];
		node.speed = rnd[3];
		node.cost = 0;
		return node;
	}
	int GetNearestListIndex(vector <Posture> &nodeList, Posture rnd, double nearest_node[3]) {
		vector<double> dlist;
		int minindex = 0;
		double cost = 0;
		double min_cost = 1000000;
		for (int i = 0; i < nodeList.size(); ++i){
			cost = (nodeList[i].x - rnd.x) *(nodeList[i].x - rnd.x)
				+ (nodeList[i].y - rnd.y) *(nodeList[i].y - rnd.y)
				+ (nodeList[i].yaw - rnd.yaw) *(nodeList[i].yaw - rnd.yaw);
			if (cost >0){
				dlist.push_back(cost);
			}
		}
		for (int i = 0; i < dlist.size(); ++i){
			if ((dlist[i] < min_cost)  && (min_cost>0)){
				min_cost = dlist[i];
				minindex = i;
			}
			else{
				std::cout << "[ not the best node  ]" << std::endl;
			}
		}
		return minindex;
	}
	bool CollisionCheck(Posture &node, vector<Posture> &obstacleList){
		double dx, dy, dyaw, cost_d;
		dx = 0;
		dy = 0;
		dyaw = 0;
		cost_d = 0;
		double distance;
		for (int i = 0; i < obstacleList.size(); ++i){
			for (int j = 0; j < node.path_x.size(); ++j) {
				dx = obstacleList[i].x - node.path_x[j];
				dy = obstacleList[i].y - node.path_y[j];
				cost_d = (1.0*dx * dx + 1.0* dy * dy);
				distance = (obstacleList[i].yaw + 20 * MAP_SCALE) * (obstacleList[i].yaw + 20 * MAP_SCALE);
				
				if (cost_d <= distance) {//obstacleList[i].yaw * obstacleList[i].yaw
					return false; // # collision
				}
			}
		}
		return true;  //# safe
	}
	bool CollisionCheckNewNode(Posture& node, vector<Posture>& nodeList){
		double dx, dy, dyaw, cost_d;
		for (int i = 0; i < nodeList.size(); ++i){
			dx = nodeList[i].x - node.x;
			dy = nodeList[i].y - node.y;
			cost_d = 0.8*dx * dx + 1.2* dy * dy;
			if (cost_d <= nodeList[i].yaw * nodeList[i].yaw) {
				return false; // # collision
			}	
		}
		return true;  //# safe
	}
	
	//(random point/nearest point/)
	Posture steer(Posture randNode, int index) {

		Posture nearestNode;
		Posture newNode;
		double cost_sum = 0;
		nearestNode = nodeList[index];
	
		vector<Posture> path_listnode;
		
		path_listnode = path_point.Generate(nearestNode, randNode, step_size, rho);
		if (path_listnode.size()>1){
			newNode = path_listnode[path_listnode.size() - 1];
		}

		for (int i = 0; i < path_listnode.size(); ++i){
			//cost_sum += abs(path_list[i].cost);
			newNode.path_x.push_back(path_listnode[i].x);
			newNode.path_y.push_back(path_listnode[i].y);
			newNode.path_yaw.push_back(path_listnode[i].yaw);
			newNode.path_speed.push_back(path_listnode[i].speed);
		}
		if (path_listnode.size()>1){
			newNode.cost += (path_listnode[path_listnode.size() - 1]).cost;

 		}
		
		newNode.parent_index = index; //path_list.size() - 1;// index;
		return newNode;
	}
	vector<int>  find_near_nodes(Posture newNode) {
		double nodeNum = nodeList.size();
		double nearindex;
		double r = 5000.0 * sqrt((log(nodeNum) / nodeNum));
		vector<double> dlist;
		vector<int> nearindexlist;
		double temp = 0;
		for (int i = 0; i < nodeNum ; ++i){
			if (CarCornerCheck(obstacleList, nodeList[i])){
				temp = (nodeList[i].x - newNode.x) *(nodeList[i].x - newNode.x) +
					(nodeList[i].y - newNode.y) *(nodeList[i].y - newNode.y) +
					(nodeList[i].yaw - newNode.yaw) *(nodeList[i].yaw - newNode.yaw);
				dlist.push_back(temp);
				if ((temp <= r*r)){
					nearindex = i;
					nearindexlist.push_back(i);
					break;
				}
			}
		}

		return nearindexlist;
	}
	Posture choose_parent(Posture node, vector<int>& nearindexlist){
		Posture tempNode_;
		vector<double> dislist;
		if (nearindexlist.size() == 0){
			return node;
		}
		double mincost = 100000;
		int minindex =0;
		for (int i = 0; i < nearindexlist.size(); ++i) {
			tempNode_ = steer(node, nearindexlist[i]);
			if (tempNode_.path_x.size() ==0){
				continue;
			}
			    
			//if (CollisionCheck(tempNode_, obstacleList)){
			if (CarCornerCheck( obstacleList, tempNode_)){

				dislist.push_back(tempNode_.cost);
			
			}
			else{
				dislist.push_back(100000);
			}
		}

		for (int i = 0; i < dislist.size();++i){
			if (dislist[i] < mincost && (mincost>0)){
				mincost = dislist[i];
				minindex = nearindexlist[i];
			}	
		}
		if (mincost >= 100000){
			return node;
		}
		node = steer(node, minindex);
		return node;
	};
	void rewire(Posture newNode, vector<int> nearinds){
		Posture tNode;
		Posture nearNode;
		Posture temp_node;
		bool imporveCost;
		int temp;
		int numnode = nodeList.size();
		for (int i = 0; i < nearinds.size(); ++i){
			temp = nearinds[i];
			nearNode = nodeList[temp];

			tNode = steer(nearNode, numnode - 1);
			if (tNode.path_x.size() == 0){
				continue;
			}
			//bool obstacleOK = CollisionCheck(tNode, obstacleList);
			bool obstacleOK = CarCornerCheck(obstacleList, tNode);
			bool imporveCost = nearNode.cost > tNode.cost;

			if (obstacleOK && imporveCost){
				nodeList[temp].x = tNode.x;
				nodeList[temp].y = tNode.y;
				nodeList[temp].yaw = tNode.yaw;
				nodeList[temp].speed = tNode.speed;
				nodeList[temp].cost = tNode.cost;
				nodeList[temp].parent_index = tNode.parent_index;
			}
		}
	};

	double calc_dist_to_goal(double x, double y){
		double temp = sqrt((x - goal.x)*(x - goal.x) + (y - goal.y)*(y - goal.y));
		return temp;
	}
	float radians(float angle){
		return angle / 57.297f;
	}
	int get_best_last_index(){
		double YAWTH = radians(5.0);
		double XYTH = 100;

		vector <int> goalinds;

		for (int i = 0; i < nodeList.size(); ++i){
			if (calc_dist_to_goal(nodeList[i].x, nodeList[i].y) <= XYTH){
				goalinds.push_back(i);
			}
		}
		vector <int> fgoalinds;
		double min_cost = 20000;
		for (int i = 0; i < goalinds.size(); ++i){
			//std::cout << "delte yaw = " << abs(nodeList[i].yaw - goal.yaw) << std::endl;
			if (abs(nodeList[(goalinds[i])].yaw - goal.yaw) <= 10 * YAWTH){
				fgoalinds.push_back(goalinds[i]);
			}
		}
		if (fgoalinds.size() ==0 ){
			//std::cout << "fgoalinds = " << std::endl;
			return -1;
		}
		int index_min;
		for (int i = 0; i < fgoalinds.size(); ++i){
			if (nodeList[fgoalinds[i]].cost <= min_cost){
				min_cost = nodeList[fgoalinds[i]].cost;
				index_min = fgoalinds[i];
			}
		}
		for (int i = 0; i < fgoalinds.size(); ++i){
			std::cout << "fgoalinds " << nodeList[fgoalinds[i]].cost << std::endl;
			if (nodeList[fgoalinds[i]].cost == min_cost){

				return fgoalinds[i];
			}
		}	
		return 0;
	}

	vector<Posture> gen_final_course(int goalind) {	
		Posture node;
		Posture rs_node_seg;
		int index = goalind;
		if (index + 1 > nodeList.size()){
			return path;
		}
		while (nodeList[index].parent_index >-1){
			node = nodeList[index];

			for (int j = 0; j < nodeList[index].path_x.size() - 1; ++j){
				rs_node_seg.x = nodeList[index].path_x[j];
				rs_node_seg.y = nodeList[index].path_y[j];
				rs_node_seg.yaw = nodeList[index].path_yaw[j];
				rs_node_seg.cost = nodeList[index].cost;
				rs_node_seg.speed = nodeList[index].path_speed[j];
				if (j>0){
					rs_node_seg.parent_index = j - 1;
				}
				else {
					rs_node_seg.parent_index = 0;
				}
				if (CarCornerCheck(obstacleList, rs_node_seg) == true) {
					path.clear();
					break;//
				}
					path.push_back(rs_node_seg);
				//}
				//lastIndex = i;
			}//for
			index = node.parent_index;
		}
		//path.push_back(goal);
		return path;
}

	vector<Posture>&   Planning(bool animation){
			Posture newNode_;
			Posture rnd;
			//Posture rs_node_seg;
			
			int nearestindex;
			vector<int> nearindexlist;
			bool nocollision, collision2;
			nodeList.clear();
			path.clear();
			nodeList.push_back(start);
			
			double nind[3];
			int lastIndex = 0;
			for (int i = 0; i < maxIter; ++i) {
				if (ParkingMode == PARALLEL){
					rnd = GetRandomPointParallel();
				}
				else{
					rnd = GetRandomPointVertical();
				}
				nearestindex = GetNearestListIndex(nodeList, rnd, nind);
				newNode_ = steer(rnd, nearestindex);
				if (newNode_.path_x.size() == 0){
					continue;
				}
				nocollision  = CollisionCheck(newNode_, obstacleList);
				collision2   = CarCornerCheck(obstacleList, newNode_);//true;//
				if (nocollision &&(collision2!= true)&& CollisionCheckNewNode(newNode_, nodeList) == true){
					nearindexlist = find_near_nodes(newNode_);
					newNode_ = choose_parent(newNode_, nearindexlist);
					if (newNode_.path_x.size() == 1){
						break;
					}
					nodeList.push_back(newNode_);	
					rewire(newNode_, nearindexlist);
				}//if
				else{
					break;
				}		
			}//for
			//# generate coruse
			lastIndex = get_best_last_index();
			if (lastIndex == -1){
				//path.push_back(start);
				//path.push_back(goal);
				lastIndex = 1;
				path = gen_final_course(lastIndex);
				return path;//
			}
			path = gen_final_course(lastIndex);
			return path;
		};

	double mod2pi(float x) {
			double v = fmod(x, 2.0 * pi);
			if (v < -pi){
				v += 2.0 * pi;
			}
			else if (v > pi){
				v -= 2.0 * pi;
			}
			return v;
		}
	double pi_2_pi(double angle) {
			return fmod((angle + pi), 2.0* pi) - pi;
		}

	void DrawCar(cv::Mat &img, Posture &PathPoint, double color_b, double color_g ,double color_r ){
		//start show
		//std::cout << "drawcar1"<< endl;
#if 0
		RotatedRect rRect(Point2f(PathPoint.x, PathPoint.y), Size2f(180 * MAP_SCALE, 420 * MAP_SCALE), (90) + 57.6*PathPoint.yaw);
		//¶¨Òå¾ØÐÎµÄ4¸ö¶¥µã	
		Point2f vertices[4]; 
		//¼ÆËã¾ØÐÎµÄ4¸ö¶¥µã	
		rRect.points(vertices);  
		std::cout << "drawcar2" << endl;
		for (int i = 0; i < 4; i++)	{
			line(img, vertices[i], vertices[(i + 1) % 4], Scalar(color_b, color_g, color_r), 1.5);
			circle(img, vertices[i], 2 + 0.9*i, Scalar(color_b, color_g, color_r), 2);
		}
#else
		std::vector<Point> vertices;
		vertices = RotateCar(PathPoint, 130 * MAP_SCALE, 410 * MAP_SCALE, 190 * MAP_SCALE);
		for (int i = 0; i < 4; i++)	{
			cv::line(img, vertices[i], vertices[(i + 1) % 4], Scalar(color_b, color_g, color_r), 1.5);
			cv::circle(img, vertices[i], 2 + 0.9*i, Scalar(color_b, color_g, color_r), 2);
		}
#endif
	}
	void Line2Object(Point &point1, Point &point2 ,vector <Posture>& obstacleList){
		double k, b;
		
		Posture node;
		double count = 0;
		//count = max(point1.x - point2.x, point1.y - point2.y);
		//for (int i = 0; i < 10; ++i){
			if (abs(point1.x - point2.x) < 10 ){
				for (int j = 0; j < abs(point1.y - point2.y); j+=10){
					node.x = point1.x;
					double min_vaule = min(point1.y, point2.y);
					node.y = min_vaule + j;
					node.yaw = 15 * MAP_SCALE;
					//j = j + abs(point1.y - point2.y)/20;
					obstacleList.push_back(node);
				}
			}
			else if (abs(point1.y - point2.y) < 10){
				for (int j = 0; j < abs(point1.x - point2.x); j+=10){
					node.y = point1.y;
					double min_vaule = min(point1.x, point2.x);
					node.x = min_vaule + j;
					node.yaw = 15 * MAP_SCALE;
					//j = j + +abs(point1.x - point2.x) / 20;
					obstacleList.push_back(node);
				}
			}
			else{
				k =  (double)(point1.y - point2.y) / (double)(point1.x - point2.x);
				b = point2.y - k*point2.x;
				//float temp = abs(point1.x - point2.x) / 70;
				if (abs(point1.x - point2.x) > abs(point1.y - point2.y)){
					for (int j = 0; j <   abs(point1.x - point2.x); j+=10){

						node.x = min(point1.x, point2.x) + j;// *(point1.x - point2.x) / 20;

						node.y = k*node.x + b;
						 
						node.yaw = 15 * MAP_SCALE;
						obstacleList.push_back(node);
					}
				}
				else{
					for (int j = 0; j <  abs(point1.y - point2.y); j+=10){

						node.y = min(point1.y, point2.y) + j; //*(point1.y - point2.y) / 20
						node.x = (node.y - b)/k;	 
						node.yaw = 15 * MAP_SCALE;
						obstacleList.push_back(node);
					}
				
				}
				
			}		
		//}	
	}
	void InitParkingSpot(const int type, cv::Mat &img, Posture &CenterPoint, 
		double color_b, double color_g, double color_r, vector <Posture>& obstacleList){
	 
		double k = 0;
		double b = 0;
		std::vector<Point> vertices;
		Point point_r, point_l;
		switch (type){
		case VERTICAL:
		{
			vertices = RotateCar(CenterPoint, 250 * MAP_SCALE, 500 * MAP_SCALE, 250 * MAP_SCALE);
			//cv::line(img, vertices[1], vertices[2], Scalar(color_b, color_g, color_r), 4.5);
			//cv::line(img, vertices[2], vertices[3], Scalar(color_b, color_g, color_r), 4.5);
			//cv::line(img, vertices[3], vertices[0], Scalar(color_b, color_g, color_r), 4.5);
			// 0   1
			// | ^ |
			// 3---2
			Line2Object(vertices[1], vertices[2], obstacleList);
			Line2Object(vertices[2], vertices[3], obstacleList);
			Line2Object(vertices[3], vertices[0], obstacleList);

			if (abs(vertices[0].x - vertices[1].x) < 10){
				k = 0;
				point_l.x = vertices[0].x;
				point_l.y = 500;

				point_r.x = vertices[1].x;
				point_r.y = 0;
				std::cout << "[ 1 ]point_l.x = " << point_l.x << ",point_l.y= " << point_l.y << endl;
				std::cout << "[ 1 ]point_r.x = " << point_r.x << ",point_r.y= " << point_r.y << endl;
			}
			else if (abs(vertices[0].y - vertices[1].y) < 10){

				point_l.x = 0;
				point_l.y = vertices[0].y;
				point_r.x = 500;
				point_r.y = vertices[1].y;
				std::cout << "[ 2 ]point_l.x = " << point_l.x << ",point_l.y= " << point_l.y << endl;
				std::cout << "[ 2 ]point_r.x = " << point_r.x << ",point_r.y= " << point_r.y << endl;
			}
			else{
				k = -0.21 + (double)(vertices[0].y - vertices[1].y) / (double)(vertices[0].x - vertices[1].x);
				b = vertices[1].y - k*vertices[1].x;
				point_l.x = vertices[0].x - 500;
				point_l.x = max(0, point_l.x);
				point_l.x = min(0, 800);
				point_l.y = k*(point_l.x) + b;

				point_r.x = vertices[1].x + 500;
				point_r.x = max(0, point_r.x);
				point_r.x = min(0, 800);

				point_r.y = k*(point_r.x + 500) + b;
				std::cout << "[ 3 ]point_l.x = " << point_l.x << ",point_l.y= " << point_l.y << endl;
				std::cout << "[ 3 ]point_r.x = " << point_r.x << ",point_r.y= " << point_r.y << endl;
			}
			//left spot line
			//cv::line(img, point_l, vertices[0], Scalar(color_b, color_g, color_r), 4.5);
			Line2Object(point_l, vertices[0], obstacleList);
			//right spot line
			//cv::line(img, vertices[1], point_r, Scalar(color_b, color_g, color_r), 4.5);
			Line2Object(vertices[1], point_r, obstacleList);
			std::cout << "k = " << k << "b= " << b << endl;
		}
		break;
		case PARALLEL:
		{
			// 0---1
			// | ^ 
			// 3---2
			vertices.clear();
			vertices = RotateCar(CenterPoint, 300 * MAP_SCALE, 680 * MAP_SCALE, 340 * MAP_SCALE);
			//cv::line(img, vertices[0], vertices[1], Scalar(color_b, color_g, color_r), 4.5);
			//cv::line(img, vertices[2], vertices[3], Scalar(color_b, color_g, color_r), 4.5);
			//cv::line(img, vertices[0], vertices[3], Scalar(color_b, color_g, color_r), 4.5);
			Line2Object(vertices[0], vertices[1], obstacleList);
			Line2Object(vertices[0], vertices[3], obstacleList);
			Line2Object(vertices[3], vertices[2], obstacleList);
			if (abs(vertices[0].x - vertices[3].x) < 10){
				k = 0;
				point_l.x = vertices[0].x;
				point_l.y = 800;

				point_r.x = vertices[3].x;
				point_r.y = 0;
				std::cout << "[ 1 ]point_l.x = " << point_l.x << ", point_l.y= " << point_l.y << endl;
				std::cout << "[ 1 ]point_r.x = " << point_r.x << ", point_r.y= " << point_r.y << endl;
			}
			else if (abs(vertices[0].y - vertices[3].y) < 10){

				point_l.x = 0;
				point_l.y = vertices[0].y;

				point_r.x = 800;
				point_r.y = vertices[1].y;
				std::cout << "[ 2 ]point_l.x = " << point_l.x << ", point_l.y= " << point_l.y << endl;
				std::cout << "[ 2 ]point_r.x = " << point_r.x << ", point_r.y= " << point_r.y << endl;
			}
			else{
				k =  -0.18+((double)(vertices[1].y - vertices[2].y) / (double)(vertices[1].x - vertices[2].x));
				b = vertices[2].y - k*vertices[2].x;
				point_l.x = vertices[1].x + 200;
				 
				point_l.x = min(300, point_l.x);
				point_l.x = max(0, point_l.x);
				point_l.y = k*(point_l.x) + b;
				/*if (point_l.x > 400){
					point_l.x = 400;
					point_l.y = k*(point_l.x ) + b; 
				}
				else {
					point_l.y = k*(point_l.x) + b;
				}*/
				point_r.x = vertices[2].x - 200;
				point_r.x = min(300, point_r.x);
				point_r.x = max(0, point_r.x);
				point_r.y = k*(point_r.x) + b;
				/*if (point_r.x < 0){
					point_r.x = 0;
					point_r.y = k*(point_r.x) + b; 
				}
				else {

					point_r.y = k*(point_r.x ) + b;
				}*/
				/*point_l.x = min(point_l.x, 800);
				point_l.x = max(point_l.x, 0);
				point_r.x = min(point_r.x, 800);
				point_r.x = max(point_r.x, 0);

				point_l.y = min(point_l.y, 800);
				point_l.y = max(point_l.y, 0);
				point_r.y = min(point_r.y, 800);
				point_r.y = max(point_r.y, 0);*/
				//point_r.y = k*(point_r.x - 500) + b;
				std::cout << "[ 3 ]point_l.x = " << point_l.x << ", point_l.y= " << point_l.y << endl;
				std::cout << "[ 3 ]point_r.x = " << point_r.x << ", point_r.y= " << point_r.y << endl;
			}
			//left spot line
			//cv::line(img, point_l, vertices[1], Scalar(color_b, color_g, color_r), 4.5);
			Line2Object(point_l, vertices[1], obstacleList);
			//right spot line
			//cv::line(img, vertices[2], point_r, Scalar(color_b, color_g, color_r), 4.5);
			Line2Object(vertices[2], point_r, obstacleList);
			std::cout << "k = " << k << "b= " << b << endl;	
		}
		break;
		default:
			break;
		}
		Point  centerpoint;
		for (int list_index = 0; list_index < obstacleList.size(); list_index++){
			centerpoint.x = obstacleList[list_index].x;
			centerpoint.y = obstacleList[list_index].y;
			cv::circle(img, centerpoint, 5 * MAP_SCALE, Scalar(0, 255, 255), 5);
		}
	}
};

void InitObs(Posture &start, Posture &end, vector <Posture>& obstacleList){
	bool   animation = true;
	//1.build object
	Posture node;
	//vertical parking
	if (ParkingMode == VERTICAL){
		
		//ÉÏ±ßÏß
		for (int i = 2; i < MapWidth-50; ++i){
			node.x = i* MAP_SCALE;
			node.y = 50 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//×ó¿âÏß
		for (int i = 2; i < ParkHightVer; ++i){
			node.x = end.x - (ParkWidthVer/2) * MAP_SCALE;
			node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15 ;
			obstacleList.push_back(node);
		}
		//ÓÒ¿âÏß
		for (int i = 2; i < ParkHightVer; ++i){
			node.x = end.x + (ParkWidthVer / 2)* MAP_SCALE;//775 * MAP_SCALE;//775
			node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		////ÏÂ±ßÏß
		for (int i = 2; i < ParkWidthVer; ++i){
			node.x = end.x + (i - ParkWidthVer/2)* MAP_SCALE;
			node.y = end.y + ParkHightVer*MAP_SCALE;//1450 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15 ;
			obstacleList.push_back(node);
		}
		//ÏÂ×ó¿â±ßÏß
		for (int i = 2; i < end.x / MAP_SCALE - ParkWidthVer / 2; ++i){
			node.x = i* MAP_SCALE;
			node.y = end.y;//950 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15 ;
			obstacleList.push_back(node);
		}
		//ÏÂÓÒ¿â±ßÏß
		for (int i = end.x/ MAP_SCALE + ParkWidthVer / 2; i < MapWidth; ++i){
			node.x = i* MAP_SCALE;
			node.y = end.y ;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
	}

	if (ParkingMode == INCLINE){
		//double ParkWidthVer = 300;
		//double ParkHightVer = 500;
		//ÉÏ±ßÏß
		for (int i = 2; i < MapWidth - 50; ++i){
			node.x = i* MAP_SCALE;
			node.y = 50 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//×ó¿âÏß
		for (int i = 2; i < ParkHightVer; ++i){
			
			node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
			node.x = end.x - (ParkWidthVer *0.6 - i*tanh(end.yaw)) * MAP_SCALE;
			//x0 = (x - node.x)*cos(a) - (y - end.y)*sin(a) + node.x;

			//y0 = (x - node.x)*sin(a) + (y - end.y)*cos(a) + end.y;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//ÓÒ¿âÏß
		for (int i = 2; i < ParkHightVer; ++i){
			node.x = end.x - (-ParkWidthVer*0.6 - i*tanh(end.yaw)) * MAP_SCALE;
			node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		////ÏÂ±ßÏß
		for (int i = 2; i < 1.6*ParkWidthVer; ++i){
			node.x = end.x + (i*tanh(end.yaw) - ParkWidthVer*0.6)* MAP_SCALE;
			node.y = end.y + ParkHightVer*MAP_SCALE;//1450 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//ÏÂ×ó¿â±ßÏß
		for (int i = 2; i < end.x / MAP_SCALE - ParkWidthVer *0.6; ++i){
			node.x = i* MAP_SCALE;
			node.y = end.y;//950 * MAP_SCALE;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//ÏÂÓÒ¿â±ßÏß
		for (int i = end.x / MAP_SCALE + ParkWidthVer *0.6  ; i < MapWidth; ++i){
			node.x = i* MAP_SCALE;
			node.y = end.y;
			node.yaw = 15 * MAP_SCALE;
			i = i + 15;
			obstacleList.push_back(node);
		}
	}
	//2.5*6.8
	if (ParkingMode == PARALLEL){
		int parkslot_length = 650;
		int parkslot_width  = 280;
		//left line 
		for (int i = 0; i < parkslot_width; ++i){
			node.x = end.x -30- parkslot_length / 2 * MAP_SCALE;// 300 * MAP_SCALE;
			node.y =  i * MAP_SCALE;
			node.yaw = 4 * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
		//right line
		for (int i = 0; i < parkslot_width; ++i){//280
			node.x = end.x + parkslot_length / 2 * MAP_SCALE;// 1030 * MAP_SCALE;
			node.y = i* MAP_SCALE;
			node.yaw = 4 * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
		 
		//Ë®Æ½³µÎ»±ßÏß
		for (int i = 0; i < parkslot_length+30; ++i){//730
			node.x = end.x -30- (parkslot_length / 2 - i)* MAP_SCALE;;// (300 + i)* MAP_SCALE;
			node.y   = 4 * MAP_SCALE;
			node.yaw = 4 * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
		//×ó±ß¿â±ßÏß
		for (int i = 0; i < end.x / MAP_SCALE - parkslot_length / 2 - 60; ++i){
			node.x = i* MAP_SCALE;
			node.y = parkslot_width * MAP_SCALE;
			node.yaw = 4  * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
		//ÓÒ±ß¿â±ßÏß
		//for (int i = 1050 ; i < 1900; ++i){
		for (int i = end.x / MAP_SCALE + parkslot_length / 2; i < 1900; ++i){
			node.x = i * MAP_SCALE;
			node.y = parkslot_width * MAP_SCALE;
			node.yaw = 4 * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
		//ÏÂ±ß½ç
		for (int i = 0; i < 1900; ++i){
			node.x = i * MAP_SCALE;
			node.y = 1400 * MAP_SCALE;
			node.yaw = 4 * MAP_SCALE;
			i = i + 5;
			obstacleList.push_back(node);
		}
	}
}

int main(){

	std::vector<Posture>  obstacleList;
	ReedsSheppPathGenerator  path_point;
	start_time = clock();
	//0. start and goal init 
	Posture start, end,last,guide1;
	double minAreaX, minAreaY;
	double maxAreaX, maxAreaY;
	std::vector<Point> test;
	switch (ParkingMode)
	{
	case VERTICAL:
		start.x =  random(300, 1400)* MAP_SCALE;
		start.y = random(100, 800) * MAP_SCALE;//750 * MAP_SCALE;
		start.yaw = random(0, 30)*0.01744f;// 0;// pi / 8;
		start.speed = 0;

		end.x = 950 * MAP_SCALE;
		end.y = 1050 * MAP_SCALE;
		end.yaw = -pi / 2;
		end.speed = 0;

		//test = RotateCar(end, 100 * MAP_SCALE, 420 * MAP_SCALE, 190 * MAP_SCALE);

		minAreaX = 20 * MAP_SCALE;// 
		maxAreaX = 1950 * MAP_SCALE;// 
		minAreaY = 10 * MAP_SCALE;// 
		maxAreaY = 1050 * MAP_SCALE;// 
		break;

	case PARALLEL:
		start.x = 1400 * MAP_SCALE;
		start.y = 650 * MAP_SCALE;
		start.yaw = 0;// pi / 8;

		//end.x = 900 * MAP_SCALE;//600
		//end.y = 180 * MAP_SCALE;// 180 * MAP_SCALE;
		//end.yaw =0;
		end.x = 300 * MAP_SCALE;//600
		end.y = 880 * MAP_SCALE;// 180 * MAP_SCALE;0
		end.yaw = -pi/2+0.1*random(0,10)*pi/10;

		//start.x = 700 * MAP_SCALE;
		//start.y = 270 * MAP_SCALE;
		//start.yaw = pi/6;// pi / 8;

		//end.x = 690 * MAP_SCALE;//600
		//end.y = 180 * MAP_SCALE;// 180 * MAP_SCALE;
		//end.yaw = 0;

		minAreaX = 200 * MAP_SCALE; 
		minAreaY = 100 * MAP_SCALE; 

		maxAreaX = 1800 * MAP_SCALE; 
		maxAreaY = 1200 * MAP_SCALE; 

		break;
	case INCLINE:
		start.x = 250 * MAP_SCALE;
		start.y = 550 * MAP_SCALE;
		start.yaw = 0;//pi / 8;
		start.speed = 0;

		end.x = 1050 * MAP_SCALE;
		end.y = 800 * MAP_SCALE;
		end.yaw = -pi / 3;
		end.speed = 0;
		//test = RotateCar(end, 100 * MAP_SCALE, 420 * MAP_SCALE, 190 * MAP_SCALE);
		minAreaX = 200 * MAP_SCALE;// 
		maxAreaX = 1450 * MAP_SCALE;// 
		minAreaY = 100 * MAP_SCALE;// 
		maxAreaY = 1050 * MAP_SCALE;// 
		break;
	default:
		break;
	}

	bool show_animation = true;
	RRT rrt_Path;

	//1.int parking_mode = PARALLEL;//INCLINE;//VERTICAL;
	//InitObs(start, end, obstacleList);
	cv::Mat img(MapHight * MAP_SCALE, MapWidth * MAP_SCALE, CV_8UC3, Scalar(192, 192, 192));//Scalar(255, 255, 255)
	rrt_Path.InitParkingSpot(PARALLEL, img, end, 0, 255, 255, obstacleList);//  
	//2.rrt
	//RRT
	
	//ofstream LOG_TXT;
	time_t currtime = time(NULL);
	tm* p = localtime(&currtime);
	char filename[100] = { 0 };

	sprintf(filename, ".\\%d%02d%02d%02d%02d%02d.txt", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);

	ofstream LOG_TXT(filename);
	LOG_TXT << "filename" << filename << endl;
	

	int loop_time = 0;
	int success_time = 0;
	int LoopCount = 10;
	bool success = false;
	
	while ((loop_time++ < LoopCount)){
		std::vector<Posture>  parkingList0;
		parkingList0.clear();
		std::cout << "loop_time :" << loop_time << endl;
		if (ParkingMode == VERTICAL){
			//vertical
			//100%
			//start.x =   random(200, 800)* MAP_SCALE;
			//start.y =   random(620, 750) * MAP_SCALE;//750 * MAP_SCALE;
			//start.yaw = (1-random(0,2)) * random(0, 10)*0.01744f;// 0;// pi / 8;
			//start.speed = 0;
			//100%
			//start.x = random(800, 1500)* MAP_SCALE;
			//start.y = random(420, 650) * MAP_SCALE;//750 * MAP_SCALE;
			//start.yaw = -1 * random(0, 10)*0.01744f;// 0;// pi / 8;
			//start.speed = 0;
			//100
			start.x = random(400, 1600)* MAP_SCALE;
			start.y =  random(500, 700) * MAP_SCALE;//750 * MAP_SCALE;
			start.yaw = (random(0, 2) - 1)* random(0, RANDOM_ANGLE)*0.01744f;// 0;// pi / 8;
			start.speed = 0;
			//3-step guide  
			//CarCornerCheck(obstacleList, newNode_);
			
			if (start.x > end.x && start.y< (end.y+50)){
				std::cout << "[ guide1 ] : |  " << endl;
				guide1.x = end.x - ParkHightVer* MAP_SCALE;
				guide1.y = end.y - ParkHightVer / 2 * MAP_SCALE;
				guide1.yaw = 0;

				parkingList0 = path_point.Generate(start, guide1, step_size, rho);
			}
			else{
				guide1 = start;
			}
		}
		else{
			//paralle
			start.x = 370.5;//random(300, 1300)* MAP_SCALE; //170.5;//
			start.y = 582.5;// random(550, 900) * MAP_SCALE;//382.5;//		
			start.yaw = -pi/2*0.9;//(random(0,2)-1)* random(0, RANDOM_ANGLE)*0.01744f;// 0;// pi / 8;
			start.speed = 0;
			if ((start.x  > end.x + ParkWidthPara*0.5* MAP_SCALE) && ((start.yaw <0 && end.yaw<0))){
				std::cout << "[ guide1 para] : |  " << endl;
				guide1.x = end.x + 2.5*ParkWidthPara * MAP_SCALE;
				guide1.y = end.y - ParkHightPara*0.8 * MAP_SCALE;
				guide1.yaw = - pi / 6;
				//start2point1
				parkingList0 = path_point.Generate(start, guide1, step_size, rho);
			}
			else if ((start.x  < end.x - ParkWidthPara* MAP_SCALE) && ((start.yaw >0 && end.yaw>0))){
				guide1 = start;
			}
		}
		for (int i = 0; i < 60; ++i){
			rrt_Path.path_list_.clear();
			rrt_Path.Init(guide1, end, minAreaX, minAreaY, maxAreaX, maxAreaY, obstacleList);
			rrt_Path.path.clear();
			//point1->point2 
			rrt_Path.path_list_ = rrt_Path.Planning(show_animation);
			//guide Point
			if (parkingList0.size()>0){
				for (int iindex = 0; iindex < parkingList0.size(); ++iindex){
					rrt_Path.path_list_.push_back(parkingList0[iindex]);
				}
			}
			
		/*	std::vector<Posture>  parkingList1, parkingList2;
			Posture guidPoint;
			guidPoint.x = 790 * MAP_SCALE;
			guidPoint.y = 300 * MAP_SCALE;
			guidPoint.yaw = pi / 6;
			guidPoint.speed = 0.2;

			parkingList1 = path_point.Generate(guidPoint, start, step_size, rho);

			parkingList2 = path_point.Generate(end, guidPoint, step_size, rho);
			for (int iindex = 0; iindex < parkingList1.size(); ++iindex){
				rrt_Path.path_list_.push_back(parkingList1[iindex]);
			}
			for (int iindex = 0; iindex < parkingList2.size(); ++iindex){
				rrt_Path.path_list_.push_back(parkingList2[iindex]);
			}*/

			if (rrt_Path.path_list_.size()>3){
				success = true;
				break;
			}
			success = false;
		}//for
		if (rrt_Path.path_list_.size() < 1){
			std::cout << "size err1 :" << rrt_Path.path_list_.size() << endl;
			continue;
		}
		for (int i = 0; i < rrt_Path.path_list_.size();i++){
			if (CarCornerCheck(obstacleList, rrt_Path.path_list_[i]) == true) success = false;
		}
		if (success == true && rrt_Path.path_list_.size() > 3 ){
			success_time += 1;
			LOG_TXT << "[Success ]: i  =  :" << loop_time << " ;start_pos: x=" << start.x << " ; y="
				<< start.y << " ;yaw=" << start.yaw << ";end_pos:x=" << end.x << " ;y=" << end.y << " ;yaw=" << end.yaw << endl;
			std::cout << "loop_time :" << loop_time << endl;		//sÎªµ¥Î»	
			std::cout << "success_time :" << success_time << endl;
		}
		else{
			LOG_TXT << "[Failed ]: i  =  :" << loop_time << " ;start_pos: x=" << start.x << " ; y=" << start.y << " ;yaw=" << start.yaw << ";end_pos:x=" << end.x << " ;y=" << end.y << " ;yaw=" << end.yaw << endl;

		}
		//
		//postproc
		if (ParkingMode == VERTICAL){
			for (int i = 0; i < 5; ++i){
				last.x = end.x;
				last.y = end.y + 75 * i * MAP_SCALE;
				last.yaw = end.yaw;
				last.speed = end.speed;
				last.parent_index = end.parent_index + 1;
				rrt_Path.path_list_.push_back(last);
			}
		}
		else{
			Posture guidPoint,endoffset;
			guidPoint.x = end.x + (ParkWidthPara*1.2)* MAP_SCALE;//790
			guidPoint.y = end.y - (ParkHightPara*0.3)* MAP_SCALE;//320
			guidPoint.yaw = - pi / 6;
			guidPoint.speed = 0.2;
			endoffset = end;
			endoffset.x = end.x + 0.2*ParkWidthPara* MAP_SCALE;
			endoffset.y = end.y - 0.1*ParkWidthPara* MAP_SCALE;
			std::vector<Posture>  parkingList1, parkingList2;
			////point2->point3
			parkingList2 = path_point.Generate(guidPoint, endoffset, step_size, rho);
			for (int iindex = 0; iindex < parkingList2.size(); ++iindex){
				rrt_Path.path_list_.push_back(parkingList2[iindex]);
			}
		}
		//}
		LOG_TXT << "VERTICAL =1, PARALLEL=2, INCLINE=3" << endl;
		LOG_TXT << "mode =  :" << ParkingMode << endl;
		LOG_TXT << "start_pos =  :" << start.x << " ;" << start.y << " ;" << start.yaw << ";end_pos:" << end.x << " ;" << end.y << " ;" << end.yaw << endl;
		std::cout << "success_time =  :" << success_time << endl;
		LOG_TXT << "raito =success_time/LoopCount  :" << 100 * success_time / LoopCount << "%" << endl;
		std::cout << "raito =success_time/LoopCount  :" << 100 * success_time / LoopCount << "%" << endl;
		
		end_time = clock();
		double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		std::cout << "Total time:" << endtime << endl;		//sÎªµ¥Î»	
		std::cout << "Total time:" << endtime * 1000 << "ms" << endl;	//msÎªµ¥Î»	

		//3 show in image
		if (show_animation){
			
			string str_text;
		//	cv::Mat img(MapHight * MAP_SCALE, MapWidth * MAP_SCALE, CV_8UC3, Scalar(192, 192, 192));//Scalar(255, 255, 255)
			//show random point area
			cv::rectangle(img, cvPoint(minAreaX, minAreaY), cvPoint(maxAreaX, maxAreaY), cvScalar(255, 0, 0), 1, 4, 0);

			Point center_start = Point(start.x, start.y);
			Point center_end = Point(end.x, end.y);
			int radius = 10 * MAP_SCALE;
			int linewidth = 3 * MAP_SCALE;
			circle(img, center_start, radius, Scalar(0, 0, 255), linewidth);
			str_text = "Start_pos:(x,y,yaw)" + to_string(start.x) +","+ to_string(start.y) +","+ to_string(start.yaw);
			cv::putText(img, str_text, Point(50,300), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 0, 0), 0.1);
			str_text = "End_pos:(x,y,yaw)" + to_string(end.x) + ","  + to_string(end.y) + "," + to_string(end.yaw);
			cv::putText(img, str_text, Point(50,350), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 0, 0), 0.1);

			cv::line(img, center_start, Point(start.x + 100 * MAP_SCALE* cos(start.yaw), start.y + 100 * MAP_SCALE * sin(start.yaw)), cv::Scalar(0, 0, 255), 2);
			str_text = "Start";
			cv::putText(img, str_text, Point(start.x, start.y + 50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

			circle(img, center_end, radius, Scalar(0, 255, 0), 3);
			cv::line(img, center_end, Point(end.x + 100 * MAP_SCALE * cos(end.yaw), end.y + 100 * MAP_SCALE * sin(end.yaw)), cv::Scalar(0, 255, 0), 2);

			str_text = "End";
			cv::putText(img, str_text, Point(end.x, end.y + 50 * MAP_SCALE), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			//obstacale  show
			/*for (int i = 0; i < test.size(); ++i){
				Point center = Point(test[i].x, test[i].y);
				circle(img, center, 5 * MAP_SCALE, Scalar(255, 0, 0), 5);
			}*/

			if (rrt_Path.path_list_.size() < 1){
				std::cout << "size err2 :" << rrt_Path.path_list_.size() << endl;
				//return 0;
			}
			for (int i = 0; i < obstacleList.size(); ++i){
				Point center_obs = Point(obstacleList[i].x, obstacleList[i].y);
				cv::circle(img, center_obs, 5 * MAP_SCALE, Scalar(0, 255, 255), 5);
			}
			//path show
			cv::Point center_pro = Point(rrt_Path.path_list_[0].x, rrt_Path.path_list_[0].y);
#if 1
			//test
			/*Posture ParkingSpot1, ParkingSpot2 ;
			ParkingSpot1.x = 800;
			ParkingSpot1.y = 300;
			ParkingSpot1.yaw = 0.98*pi;
			ParkingSpot1.speed = 0;

			ParkingSpot2.x =100;
			ParkingSpot2.y = 350;
			
			ParkingSpot2.speed = 0;
			ParkingSpot2.yaw = -0.97*pi/2;*/
			//rrt_Path.DrawParkingSpot(VERTICAL, img, end, 0, 255, 255, obstacleList);// 

			//rrt_Path.DrawParkingSpot(PARALLEL, img, end, 0, 255, 255, obstacleList);//  
			//

#endif
			for (int i = 0; i < rrt_Path.path_list_.size(); ++i){
				//DrawCar
				if (CarCornerCheck(obstacleList, rrt_Path.path_list_[i]) != true){//CarCornerCheck
					if (rrt_Path.path_list_[i].speed == 0) rrt_Path.DrawCar(img, rrt_Path.path_list_[i], 255, 0, 0);// ±£³ÖÀ¶É«ÎÞÅö×²
					if (rrt_Path.path_list_[i].speed>0) rrt_Path.DrawCar(img, rrt_Path.path_list_[i], 0, 255, 50);// Ç°½øÂÌÉ«ÎÞÅö×²
					if (rrt_Path.path_list_[i].speed < 0) rrt_Path.DrawCar(img, rrt_Path.path_list_[i], 0, 0, 0);// µ¹ÍËºÚÉ«ÎÞÅö×²
					////std::cout << i << "  speed = |" << path_list_[i].speed << std::endl;
					////ParkingSpot = 
					////rrt_Path.DrawParkingSpot(VERTICAL,img, rrt_Path.path_list_[i], 0, 255, 255);// ºìÉ«Åö×²
				}
				else{
					rrt_Path.DrawCar(img, rrt_Path.path_list_[i], 0, 0, 255);// ºìÉ«Åö×²
					//std::cout << i << "  red = |" << path_list_[i].speed << std::endl;
				}
				//DrawCar path
				Point center = Point(rrt_Path.path_list_[i].x, rrt_Path.path_list_[i].y);

				str_text = to_string(i);
				cv::putText(img, str_text, center, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

				circle(img, center, 7 * MAP_SCALE, Scalar(255, 0, i * 255 / rrt_Path.path_list_.size()), 3);
				cv::line(img, center, Point(rrt_Path.path_list_[i].x + 40 * MAP_SCALE * cos(rrt_Path.path_list_[i].yaw), rrt_Path.path_list_[i].y +
					40 * MAP_SCALE * sin(rrt_Path.path_list_[i].yaw)), cv::Scalar(255, 0, 0), 4);

				if (i > 0 && i < rrt_Path.path_list_.size()){
					cv::Point start = center_pro;
					cv::Point end = center;
					cv::line(img, start, end, cv::Scalar(0, 0, 255), 1.5);
				}
				center_pro = center;
			}
			//file.write(str(ip[0])+' , '+str(ip[1])+'\n') 
			string str_window = "iFLYTEK_AutoParking_Planning: CostTime = " + to_string(endtime) + "/s";
			imshow(str_window, img);
			if (success == true){
				string successimg = "F:\\00-ADAS\\04-planing\\iflytek\\APS_MP\\APS_MP\\01SUCCESS\\success" + to_string(endtime) + ".jpg";
				imwrite(successimg, img);
			}
			else{
				string failedimg = "F:\\00-ADAS\\04-planing\\iflytek\\APS_MP\\APS_MP\\02FAILED\\failed" + to_string(endtime) + ".jpg";
				imwrite(failedimg, img);
			}
			waitKey(1);
			destroyAllWindows();
		}
	}//while
	return 0;
}
#if 0
int main_rs_ok(){
	ReedsSheppPathGenerator  path_point;
	std::vector<Posture> path_list_;
	//1.rs_point init
	Posture start, end;
	start.x = 100;
	start.y = 100;
	start.yaw = 0;

	end.x = 600;
	end.y = 910;
	end.yaw = pi / 2;
	//
	cv::Mat img(1500, 1500, CV_8UC3, Scalar(255, 255, 255));
	Point center_start = Point(start.x, start.y);
	Point center_end = Point(end.x, end.y);
	int r = 3;
	circle(img, center_start, 10, Scalar(0, 0, 255), 3);
	circle(img, center_end, 10, Scalar(0, 255, 0), 3);
	//2.generate rs path
	path_list = path_point.Generate(start, end, step_size, rho);
	
	for (int i = 0; i < path_list.size() - 1; ++i){
		//std::cout << "dist = " << path_list[i].x << std::endl;
		//std::cout << "dist = " << path_list[i].y << std::endl;
		//std::cout << "dist = " << path_list[i].yaw << std::endl;

		Point center = Point(path_list[i].x, path_list[i].y);
		//std::cout << "center: " << center << std::endl;
		circle(img, center, 3, Scalar(255, 255, 0), 3);
	}
	//
	

	imshow("base", img);
	waitKey();
	destroyAllWindows();

	return 0;
}
int img_test()//int argc,char* argv[]
{
	//1.read motionplanning result(x,y) from txt
	ifstream  fin("in_mp_result.txt");
	string s;
	//float x;
	string slid = ",";
	vector<float> point;
	int x, y;
	const char* vehicle_img = "car.jpg";
	cv::Mat img_car = imread(vehicle_img);

	cv::Mat img(1500, 1500, CV_8UC3, Scalar(255, 255, 255));
	if (img_car.empty()){
		fprintf(stderr, "cannot load img %s\n", img_car);
		return -1;
	}

	cv::Rect roi_rect = cv::Rect(128, 128, img_car.cols, img_car.rows);
	img_car.copyTo(img(roi_rect));

	//ï¿½ë¾¶
	int r = 5;

	while (fin >> s)
	{
		if (s == ",")
		{
			continue;
		}
		point.push_back(std::stof(s));
		//cout << "Read from file: " << std::stof(s) << endl;
	}
	for (int i = 0; i < point.size(); i++)
	{
		x = 80 * point[i];
		y = 80 * point[i + 1];
		i = i + 1;
		Point center = Point(x, y);
		//cout << "center: " << center << endl;
		circle(img, center, r, Scalar(0, 255, 0), 3);
	}
	point.clear();
	imshow("base", img);
	//waitKey();
	destroyAllWindows();
	return 0;
}

int main_py(int argc, char* argv[])//
{
	Path path;
	vector<Path> paths;
	//std::cout << "[ RRT_start_cc: ] path planner sample start!!" << std::endl;

	float start_x   = 800.0;  // [m]
	float start_y   = 500.0;  // [m]
	float start_yaw = radians(0.0);  // [rad]

	float end_x   = 500.0;  // [m]
	float end_y   = 600.0;  // [m]
	float end_yaw = radians(95.0);  // [rad]
	
	Point center_start = Point(start_x, start_y);
	Point center_end   = Point(end_x, end_y);

	float curvature = 2;//1
	float step_size = 10;//0.1
	
	reeds_shepp_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size, paths,path);
	//std::cout << "[ RRT_start_cc: ] show img" << std::endl;

	if (1){
		//plotting
		//plot_arrow(start_x, start_y, start_yaw);
		//plot_arrow(end_x, end_y, end_yaw);

		
		float x, y;
		const char* vehicle_img = "car.jpg";
		cv::Mat img_car = imread(vehicle_img);

		cv::Mat img(1500, 1500, CV_8UC3, Scalar(255, 255, 255));
		if (img_car.empty()){
			fprintf(stderr, "cannot load img %s\n", img_car);
			return -1;
		}

		cv::Rect roi_rect = cv::Rect(528, 728, img_car.cols, img_car.rows);
		img_car.copyTo(img(roi_rect));

		//ï¿½ë¾¶
		int r = 10;
		//int i = 1;
		
		for (int index = 0; index < paths.size(); ++index){
			for (int i = 0; i < paths[index].x.size(); ++i){
				x = (paths[index].x)[i];
				y = (paths[index].y)[i];
				Point center = Point(x, y);
				//std::cout << "[ path :] x = " << x << ";y = " << y << std::endl;
				//std::cout << "center: " << center << std::endl;
				circle(img, center, r, Scalar(255, 255, 0), 3);
			}
		}
		for (int i = 0; i < path.x.size(); ++i)
		{
			x = (path.x)[i];
			y = (path.y)[i];
			//std::cout << "x = " << x << ";y = " << y << std::endl;
			//i = i + 1;
			Point center = Point(x, y);
			//std::cout << "center: " << center << std::endl;
			circle(img, center, r, Scalar(0, 255, 0), 3);
		}
		circle(img, center_start, r, Scalar(255, 0, 0), 3);
		circle(img, center_end,   r, Scalar(255, 0, 0), 3);

		imshow("base", img);
		waitKey();
		destroyAllWindows();

		//std::vector<cv::Point> points;

		//points.push_back(cv::Point(200, 240));
		//points.push_back(cv::Point(300, 400));
		//points.push_back(cv::Point(400, 360));
		//points.push_back(cv::Point(500, 300));
		//points.push_back(cv::Point(500, 200));
		//points.push_back(cv::Point(300, 150));

		////ï¿½ï¿½ï¿½ï¿½Ïµï¿½ï¿½ï¿½Æµï¿½ï¿½Õ°ï¿½Í¼ï¿½ï¿½  
		//for (int i = 0; i < points.size(); i++)
		//{
		//	cv::circle(img, points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
		//}
		//cv::RotatedRect rotate_rect = cv::fitEllipse(points);
		////ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô?
		//cv::ellipse(img, rotate_rect, cv::Scalar(0, 255, 255), 2, 8);

		////ï¿½ï¿½È¡ï¿½ï¿½×ªï¿½ï¿½ï¿½Îµï¿½ï¿½Ä¸ï¿½ï¿½ï¿½ï¿½ï¿½
		//cv::Point2f* vertices = new cv::Point2f[4];
		//rotate_rect.points(vertices);

		//std::vector<cv::Point> contour;

		//for (int i = 0; i < 4; i++)
		//{
		//	contour.push_back(vertices[i]);
		//}

		//std::vector<std::vector<cv::Point>> contours;
		//contours.push_back(contour);
		//cv::drawContours(img, contours, 0, cv::Scalar(255, 255, 0), 1);

		
	}


	return 0;
}
#endif