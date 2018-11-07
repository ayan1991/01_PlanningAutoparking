#include "modules/planning/common/path_managerRRT.h"
//#include "modules/planning/common/ccrrt_rs.h"
#include "modules/common/type.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdio.h>

 
#include "./reeds_shepp_iapa.h"

#include<cstdlib>
#include<ctime>
#include <math.h>
#include<ctime>


namespace iapa {
    namespace planning {
		using namespace iapa::common;
		using namespace cv;
		clock_t start_time, end_time;
        PathManagerRRT::PathManagerRRT() {
            Clear();
        }

        void PathManagerRRT::Clear() {
            path_list_.clear();
        }

        /*void PathManagerRRT::Generate(const iapa::common::PostureRRT& start_pos, const iapa::common::PostureRRT& end_pos, const PathConf& path_conf) {
            path_conf_ = path_conf;
            path_list_.clear();
            path_list_ = path_generator_.Generate(start_pos, end_pos, path_conf_.step_size(), path_conf_.turning_radius());
            for (int i = 0; i < path_list_.size(); i++) {
                if (path_list_[i].v > path_conf_.velocity_thr())
                    path_list_[i].v = path_conf_.velocity_limit();
                else
                    path_list_[i].v = path_list_[i].v * path_conf_.velocity_limit();
            }
        }*/

		vector <Point> RotateCar(iapa::common::PostureRRT& RotateCenter, double WheelBase, double CarLen, double CarWid){

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
			//绕一个坐标点(rx0,ry0)
			double a = RotateCenter.yaw;
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

		// 叉乘 |p1 p2| X |p1 p|
		double GetCross(Point& p1, Point& p2, Point& p)
		{
			double temp = (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
			return temp;
		}

		bool CarCornerCheck(vector< iapa::common::PostureRRT>& objectlist, iapa::common::PostureRRT& carCenter){
			//-----p1-------p4-------
			//-----|   p->   |-------
			//-----p2-------p3-------

			//carCenter.x = 200;
			//carCenter.y = 300;
			//carCenter.yaw = 0;

			//RotatedRect rRect(Point2f(carCenter.x, carCenter.y), Size2f(180 * MAP_SCALE, 420 * MAP_SCALE), (90) + 57.6*carCenter.yaw);
			//定义矩形的4个顶点	
			//Point2f vertices[4];
			//计算矩形的4个顶点	
			//rRect.points(vertices);
			vector<Point> car_corner;
			car_corner = RotateCar(carCenter, 100 * MAP_SCALE, 420 * MAP_SCALE, 190 * MAP_SCALE);

			Point p4(car_corner[0].x, car_corner[0].y);
			Point p3(car_corner[1].x, car_corner[1].y);
			Point p2(car_corner[2].x, car_corner[2].y);
			Point p1(car_corner[3].x, car_corner[3].y);
			bool safe = false;



			Point objectxy;
			for (int i = 0; i < objectlist.size(); ++i){
				objectxy.x = objectlist[i].x;
				objectxy.y = objectlist[i].y;

				safe = (GetCross(p1, p2, objectxy) * GetCross(p3, p4, objectxy) >= 0) && (GetCross(p2, p3, objectxy) * GetCross(p4, p1, objectxy) >= 0);
				if (safe == true){
					return true;//无碰撞
				}
			}
			return false;//碰撞
		}


		bool PointCarCollisionCheck(vector< iapa::common::PostureRRT>& objectlist, iapa::common::PostureRRT& carCenter){
			//-----p1-------p4-------
			//-----|   p->   |-------
			//-----p2-------p3-------
			bool safe = false;
			Point p1(carCenter.x - CarLength / 2, carCenter.y - CarWidth / 2);
			Point p2(carCenter.x - CarLength / 2, carCenter.y + CarWidth / 2);
			Point p3(carCenter.x + CarLength / 2, carCenter.y + CarWidth / 2);
			Point p4(carCenter.x + CarLength / 2, carCenter.y - CarWidth / 2);
			for (int i = 0; i < objectlist.size(); ++i){
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

			RRT(iapa::common::PostureRRT q0, iapa::common::PostureRRT q1, int minX, int minY, int maxX, int maxY, vector< iapa::common::PostureRRT> obstacles){
				start = q0;
				goal = q1;
				minrandX = minX;
				minrandY = minY;

				maxrandX = maxX;
				maxrandY = maxY;
				obstacleList = obstacles;
			};

		private:

			int  minrandX, minrandY;
			int  maxrandX, maxrandY;
			int  goalSampleRate = 80;
			int  maxIter = 50;
			double rnd[3];
			vector< iapa::common::PostureRRT> path;
			vector< iapa::common::PostureRRT> obstacleList;
			vector < iapa::common::PostureRRT> nodeList;
			vector< iapa::common::PostureRRT>  path_list_;
			iapa::common::PostureRRT newNode_;
			ReedsSheppPathGeneratorRRT  path_point;
			iapa::common::PostureRRT start, goal;

		public:
			//return node
			iapa::common::PostureRRT get_random_point(){
				iapa::common::PostureRRT node;
				double rnd[4] = { 0 };
				srand((unsigned)time(NULL));
				if (random(1, 100) > goalSampleRate){
					rnd[0] = 1.0* random(minrandX, maxrandX);
					rnd[1] = 1.0* random(minrandY, maxrandY);
					rnd[2] = 0.01*(random(314, 628) - 314);
					rnd[3] = 2;
				}
				else if (random(0, 100) < 30){
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
				return node;
			}
			int GetNearestListIndex(vector < iapa::common::PostureRRT> &nodeList, iapa::common::PostureRRT rnd, double nearest_node[3]) {
				vector<double> dlist;
				int minindex = 0;
				double cost = 0;
				double min_cost = 1000000;
				for (int i = 0; i < nodeList.size(); ++i){
					cost = (nodeList[i].x - rnd.x) *(nodeList[i].x - rnd.x)
						+ (nodeList[i].y - rnd.y) *(nodeList[i].y - rnd.y)
						+ (nodeList[i].yaw - rnd.yaw) *(nodeList[i].yaw - rnd.yaw);
					if (cost > 0){
						dlist.push_back(cost);
					}
				}
				for (int i = 0; i < dlist.size(); ++i){
					if ((dlist[i] < min_cost) && (min_cost>0)){
						min_cost = dlist[i];
						minindex = i;
					}
					else{
						std::cout << "[ not the best node  ]" << std::endl;
					}
				}
				return minindex;
			}
			bool CollisionCheck(iapa::common::PostureRRT &node, vector< iapa::common::PostureRRT> &obstacleList){
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
			bool CollisionCheckNewNode(iapa::common::PostureRRT& node, vector< iapa::common::PostureRRT>& nodeList){
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
			iapa::common::PostureRRT steer(iapa::common::PostureRRT randNode, int index) {

				iapa::common::PostureRRT nearestNode;
				iapa::common::PostureRRT newNode;
				double cost_sum = 0;
				nearestNode = nodeList[index];

				vector< iapa::common::PostureRRT> path_listnode;

				path_listnode = path_point.GenerateRRT(nearestNode, randNode, step_size, rho);
				if (path_listnode.size() > 1){
					newNode = path_listnode[path_listnode.size() - 1];
				}

				for (int i = 0; i < path_listnode.size(); ++i){
					//cost_sum += abs(path_list[i].cost);
					newNode.path_x.push_back(path_listnode[i].x);
					newNode.path_y.push_back(path_listnode[i].y);
					newNode.path_yaw.push_back(path_listnode[i].yaw);
					newNode.path_speed.push_back(path_listnode[i].speed);
				}
				if (path_listnode.size() > 1){
					newNode.cost += (path_listnode[path_listnode.size() - 1]).cost;

				}

				newNode.parent_index = index; //path_list.size() - 1;// index;
				return newNode;
			}
			vector<int>  find_near_nodes(iapa::common::PostureRRT newNode) {
				double nodeNum = nodeList.size();
				double nearindex;
				double r = 5000.0 * sqrt((log(nodeNum) / nodeNum));
				vector<double> dlist;
				vector<int> nearindexlist;
				double temp = 0;
				for (int i = 0; i < nodeNum; ++i){
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
			iapa::common::PostureRRT choose_parent(iapa::common::PostureRRT node, vector<int>& nearindexlist){
				iapa::common::PostureRRT tempNode_;
				vector<double> dislist;
				if (nearindexlist.size() == 0){
					return node;
				}
				double mincost = 100000;
				int minindex = 0;
				for (int i = 0; i < nearindexlist.size(); ++i) {
					tempNode_ = steer(node, nearindexlist[i]);
					if (tempNode_.path_x.size() == 0){
						continue;
					}

					//if (CollisionCheck(tempNode_, obstacleList)){
					if (CarCornerCheck(obstacleList, tempNode_)){

						dislist.push_back(tempNode_.cost);

					}
					else{
						dislist.push_back(100000);
					}
				}

				for (int i = 0; i < dislist.size(); ++i){
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
			void rewire(iapa::common::PostureRRT newNode, vector<int> nearinds){
				iapa::common::PostureRRT tNode;
				iapa::common::PostureRRT nearNode;
				iapa::common::PostureRRT temp_node;
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
			double radians(double angle){
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
				if (fgoalinds.size() == 0){
					std::cout << "fgoalinds = " << std::endl;
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

			vector< iapa::common::PostureRRT> gen_final_course(int goalind) {
				iapa::common::PostureRRT node;
				iapa::common::PostureRRT rs_node_seg;
				int index = goalind;
				while (nodeList[index].parent_index > -1){
					node = nodeList[index];

					for (int j = 0; j < nodeList[index].path_x.size() - 1; ++j){
						rs_node_seg.x = nodeList[index].path_x[j];
						rs_node_seg.y = nodeList[index].path_y[j];
						rs_node_seg.yaw = nodeList[index].path_yaw[j];
						rs_node_seg.cost = nodeList[index].cost;
						rs_node_seg.speed = nodeList[index].path_speed[j];
						if (j > 0){
							rs_node_seg.parent_index = j - 1;
						}
						else {
							rs_node_seg.parent_index = 0;
						}
						path.push_back(rs_node_seg);
						//lastIndex = i;
					}//for
					index = node.parent_index;
				}
				path.push_back(goal);
				return path;
			}

			vector< iapa::common::PostureRRT>&   Planning(bool animation){
				iapa::common::PostureRRT newNode_;
				iapa::common::PostureRRT rnd;
				//PostureRRT rs_node_seg;

				int nearestindex;
				vector<int> nearindexlist;
				bool nocollision, nocollision2;
				nodeList.clear();
				nodeList.push_back(start);

				double nind[3];
				int lastIndex = 0;
				for (int i = 0; i < maxIter; ++i) {
					rnd = get_random_point();

					nearestindex = GetNearestListIndex(nodeList, rnd, nind);
					newNode_ = steer(rnd, nearestindex);
					if (newNode_.path_x.size() == 0){
						continue;
					}
					nocollision = CollisionCheck(newNode_, obstacleList);
					//nocollision2 = CarCornerCheck(obstacleList, newNode_);
					if (nocollision && CollisionCheckNewNode(newNode_, nodeList) == true){
						nearindexlist = find_near_nodes(newNode_);
						newNode_ = choose_parent(newNode_, nearindexlist);
						if (newNode_.path_x.size() == 1){
							break;
						}
						nodeList.push_back(newNode_);
						rewire(newNode_, nearindexlist);
					}//if
				}//for
				//# generate coruse
				lastIndex = get_best_last_index();

				if (lastIndex == -1){
					path.push_back(start);
					path.push_back(goal);
					return path;//
				}
				path = gen_final_course(lastIndex);

				return path;
			};

			double mod2pi(float x) {
				float v = fmod(x, 2.0 * pi);
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

			void DrawCar(cv::Mat &img, iapa::common::PostureRRT &PathPoint, double color_b, double color_g, double color_r){
				//start show
				std::cout << "drawcar1" << endl;
#if 0
				RotatedRect rRect(Point2f(PathPoint.x, PathPoint.y), Size2f(180 * MAP_SCALE, 420 * MAP_SCALE), (90) + 57.6*PathPoint.yaw);
				//定义矩形的4个顶点	
				Point2f vertices[4];
				//计算矩形的4个顶点	
				rRect.points(vertices);
				std::cout << "drawcar2" << endl;
				for (int i = 0; i < 4; i++)	{
					line(img, vertices[i], vertices[(i + 1) % 4], Scalar(color_b, color_g, color_r), 1.5);
					circle(img, vertices[i], 2 + 0.9*i, Scalar(color_b, color_g, color_r), 2);
				}
#else
				std::vector<Point> vertices;
				vertices = RotateCar(PathPoint, 100 * MAP_SCALE, 420 * MAP_SCALE, 190 * MAP_SCALE);
				for (int i = 0; i < 4; i++)	{
					line(img, vertices[i], vertices[(i + 1) % 4], Scalar(color_b, color_g, color_r), 1.5);
					circle(img, vertices[i], 2 + 0.9*i, Scalar(color_b, color_g, color_r), 2);
				}
#endif
			}
		};
		void PathManagerRRT::InitObs( iapa::common::PostureRRT &start,  iapa::common::PostureRRT &end, vector < iapa::common::PostureRRT>& obstacleList){
			bool   animation = true;
			//1.build object
			iapa::common::PostureRRT node;
			//vertical parking
			if (ParkingMode == VERTICAL){
				double ParkWidthVer = 250;
				double ParkHightVer = 500;
				//上边线
				for (int i = 2; i < MapWidth - 50; ++i){
					node.x = i* MAP_SCALE;
					node.y = 50 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//左库线
				for (int i = 2; i < ParkHightVer; ++i){
					node.x = end.x - (ParkWidthVer / 2) * MAP_SCALE;
					node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//右库线
				for (int i = 2; i < ParkHightVer; ++i){
					node.x = end.x + (ParkWidthVer / 2)* MAP_SCALE;//775 * MAP_SCALE;//775
					node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				////下边线
				for (int i = 2; i < ParkWidthVer; ++i){
					node.x = end.x + (i - ParkWidthVer / 2)* MAP_SCALE;
					node.y = end.y + ParkHightVer*MAP_SCALE;//1450 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//下左库边线
				for (int i = 2; i < end.x / MAP_SCALE - ParkWidthVer / 2; ++i){
					node.x = i* MAP_SCALE;
					node.y = end.y;//950 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//下右库边线
				for (int i = end.x / MAP_SCALE + ParkWidthVer / 2; i < MapWidth; ++i){
					node.x = i* MAP_SCALE;
					node.y = end.y;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
			}

			if (ParkingMode == INCLINE){
				double ParkWidthVer = 300;
				double ParkHightVer = 500;
				//上边线
				for (int i = 2; i < MapWidth - 50; ++i){
					node.x = i* MAP_SCALE;
					node.y = 50 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//左库线
				for (int i = 2; i < ParkHightVer; ++i){

					node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
					node.x = end.x - (ParkWidthVer *0.6 - i*tanh(end.yaw)) * MAP_SCALE;
					//x0 = (x - node.x)*cos(a) - (y - end.y)*sin(a) + node.x;

					//y0 = (x - node.x)*sin(a) + (y - end.y)*cos(a) + end.y;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//右库线
				for (int i = 2; i < ParkHightVer; ++i){
					node.x = end.x - (-ParkWidthVer*0.6 - i*tanh(end.yaw)) * MAP_SCALE;
					node.y = end.y + i* MAP_SCALE;//(1450 - i)* MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				////下边线
				for (int i = 2; i < 1.5*ParkWidthVer; ++i){
					node.x = end.x + (i*tanh(end.yaw) - ParkWidthVer*0.6)* MAP_SCALE;
					node.y = end.y + ParkHightVer*MAP_SCALE;//1450 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//下左库边线
				for (int i = 2; i < end.x / MAP_SCALE - ParkWidthVer *0.6; ++i){
					node.x = i* MAP_SCALE;
					node.y = end.y;//950 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//下右库边线
				for (int i = end.x / MAP_SCALE + ParkWidthVer *0.6; i < MapWidth; ++i){
					node.x = i* MAP_SCALE;
					node.y = end.y;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
			}
			//2.8*6.8
			if (ParkingMode == PARALLEL){
				//left line 
				for (int i = 0; i < 280; ++i){
					node.x = 350 * MAP_SCALE;
					node.y = i * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//right line
				for (int i = 0; i < 280; ++i){
					node.x = 1030 * MAP_SCALE;
					node.y = i* MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//车位长度：650
				//水平车位边线
				for (int i = 0; i < 700; ++i){
					node.x = (350 + i)* MAP_SCALE;
					node.y = 15 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//左边库边线
				for (int i = 0; i < 350; ++i){
					node.x = i* MAP_SCALE;
					node.y = 280 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//右边库边线
				for (int i = 1050; i < 1900; ++i){
					node.x = i * MAP_SCALE;
					node.y = 280 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
				//下边界
				for (int i = 0; i < 1500; ++i){
					node.x = i * MAP_SCALE;
					node.y = 1400 * MAP_SCALE;
					node.yaw = 15 * MAP_SCALE;
					i = i + 15;
					obstacleList.push_back(node);
				}
			}
		}

		void PathManagerRRT::GetPlanningRes(iapa::common::PostureRRT& start, iapa::common::PostureRRT& end, const PathConf& path_conf){
			path_conf_ = path_conf;
			std::vector<iapa::common::PostureRRT> path_list, obstacleList;
			ReedsSheppPathGeneratorRRT  path_point;
			start_time = clock();
			//0. start and goal init 
			//iapa::common::PostureRRT start, end;
			double minAreaX, minAreaY;
			double maxAreaX, maxAreaY;
			std::vector<Point> test;
			switch (ParkingMode)
			{
			case VERTICAL:
				start.x = 250 * MAP_SCALE;
				start.y = 650 * MAP_SCALE;
				start.yaw = 0;//pi / 8;
				start.speed = 0;

				end.x = 1050 * MAP_SCALE;
				end.y = 1050 * MAP_SCALE;
				end.yaw = -pi / 2;
				end.speed = 0;

				minAreaX = 200 * MAP_SCALE;// 
				maxAreaX = 1450 * MAP_SCALE;// 
				minAreaY = 100 * MAP_SCALE;// 
				maxAreaY = 1050 * MAP_SCALE;// 
				break;

			case PARALLEL:
				start.x = 1400 * MAP_SCALE;
				start.y = 650 * MAP_SCALE;
				start.yaw = 0;// pi / 8;

				end.x = 600 * MAP_SCALE;//600
				end.y = 180 * MAP_SCALE;
				end.yaw = 0;

				minAreaX = 200 * MAP_SCALE;
				minAreaY = 50 * MAP_SCALE;

				maxAreaX = 1600 * MAP_SCALE;
				maxAreaY = 900 * MAP_SCALE;

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
			//1.int parking_mode = PARALLEL;//INCLINE;//VERTICAL;
			InitObs(start, end, obstacleList);

			//2.rrt
			//RRT
			bool show_animation = true;
			RRT rrt_Path(start, end, minAreaX, minAreaY, maxAreaX, maxAreaY, obstacleList);
			int loop_time = 0;
			while (path_list.size() < 6 && (loop_time < 120)){
				loop_time++;
				path_list.clear();

				path_list = rrt_Path.Planning(show_animation);

			}
			//path_list.push_back(start);
			end_time = clock();
			double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
			std::cout << "Total time:" << endtime << endl;		//s为单位	
			std::cout << "Total time:" << endtime * 1000 << "ms" << endl;	//ms为单位	

			//3 show in image
			if (show_animation){
				string str_text;
				cv::Mat img(MapHight * MAP_SCALE, MapWidth * MAP_SCALE, CV_8UC3, Scalar(192, 192, 192));//Scalar(255, 255, 255)
				//show random point area
				cv::rectangle(img, cvPoint(minAreaX, minAreaY), cvPoint(maxAreaX, maxAreaY), cvScalar(255, 0, 0), 1, 4, 0);

				Point center_start = Point(start.x, start.y);
				Point center_end = Point(end.x, end.y);
				int radius = 10 * MAP_SCALE;
				int linewidth = 3 * MAP_SCALE;
				circle(img, center_start, radius, Scalar(0, 0, 255), linewidth);

				cv::line(img, center_start, Point(start.x + 100 * MAP_SCALE* cos(start.yaw), start.y + 100 * MAP_SCALE * sin(start.yaw)), cv::Scalar(0, 0, 255), 2);
				str_text = "Start";
				cv::putText(img, str_text, Point(start.x, start.y + 50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

				circle(img, center_end, radius, Scalar(0, 255, 0), 3);
				cv::line(img, center_end, Point(end.x + 100 * MAP_SCALE * cos(end.yaw), end.y + 100 * MAP_SCALE * sin(end.yaw)), cv::Scalar(0, 255, 0), 2);

				str_text = "End";
				cv::putText(img, str_text, Point(end.x, end.y + 50 * MAP_SCALE), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
				//obstacale  show
				for (int i = 0; i < test.size(); ++i){
					Point center = Point(test[i].x, test[i].y);
					circle(img, center, 5 * MAP_SCALE, Scalar(255, 0, 0), 5);
				}

				for (int i = 0; i < obstacleList.size(); ++i){
					Point center = Point(obstacleList[i].x, obstacleList[i].y);
					circle(img, center, 5 * MAP_SCALE, Scalar(0, 255, 255), 5);
				}
				//path show
				cv::Point center_pro = Point(path_list[0].x, path_list[0].y);

				/*for (int i = 0; i < obstacleList.size(); ++i){
				Point obstac_center(obstacleList[i].x, obstacleList[i].y);
				circle(img, obstac_center, 3, Scalar(255, 0, 0), 3);

				}*/
				for (int i = 0; i < path_list.size(); ++i){
					//DrawCar

					if (CarCornerCheck(obstacleList, path_list[i]) != true){//CarCornerCheck
						if (path_list[i].speed == 0) rrt_Path.DrawCar(img, path_list[i], 255, 0, 0);// 保持蓝色无碰撞
						if (path_list[i].speed > 0) rrt_Path.DrawCar(img, path_list[i], 0, 255, 50);// 前进绿色无碰撞
						if (path_list[i].speed < 0) rrt_Path.DrawCar(img, path_list[i], 0, 0, 0);// 倒退黑色无碰撞
						std::cout << i << "  speed = |" << path_list[i].speed << std::endl;
					}
					else{
						rrt_Path.DrawCar(img, path_list[i], 0, 0, 255);// 红色碰撞
						std::cout << i << "  red = |" << path_list[i].speed << std::endl;
					}
					//DrawCar path
					Point center = Point(path_list[i].x, path_list[i].y);

					str_text = to_string(i);
					cv::putText(img, str_text, center, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

					circle(img, center, 7 * MAP_SCALE, Scalar(255, 0, i * 255 / path_list.size()), 3);
					cv::line(img, center, Point(path_list[i].x + 40 * MAP_SCALE * cos(path_list[i].yaw), path_list[i].y +
						40 * MAP_SCALE * sin(path_list[i].yaw)), cv::Scalar(255, 0, 0), 4);

					if (i > 0 && i < path_list.size()){
						cv::Point start = center_pro;
						cv::Point end = center;
						cv::line(img, start, end, cv::Scalar(0, 0, 255), 1.5);
					}
					center_pro = center;
				}
				//file.write(str(ip[0])+' , '+str(ip[1])+'\n') 
				string str_window = "iFLYTEK_AutoParking_Planning: CostTime = " + to_string(endtime) + "/s";
				imshow(str_window, img);
				//waitKey();
				//destroyAllWindows();
			}
			return ;
		}

		//}
	};
}
