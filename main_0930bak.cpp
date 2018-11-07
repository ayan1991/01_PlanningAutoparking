

#include <opencv2/opencv.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdio.h>

#include "./cc_rs.h"
#include "./reeds_shepp.h"

#include<cstdlib>
#include<ctime>
#include <math.h>
#include<ctime>

using namespace  cv;
using namespace  std;
#define VERTICAL 1
#define PARALLEL 2
#define INCLINE  3
#define ParkingMode  2

#define random(a,b) (rand()%(b-a+1)+a)

clock_t start_time, end_time;

const float  step_size = 30;
const float  rho = 440; // turning radius

void get_point(double center[2], double radius, double orin, double x, double y) {
	x = center[0] + radius * cos(orin);
	y = center[1] + radius * sin(orin);
	return;
}
void  plot_car(double q[4]) {
	double a[2], b[2], c[2];
	
	//get_point([q[0], q[1]], step_size, q[2], a[0], a[1]);

	//double b = get_point(q[3], step_size / 2, q[2] + 150. / 180.*pi);
	//double c = get_point(q[3], step_size / 2, q[2] - 150. / 180.*pi);
	//tri = np.array([a, b, c, a]);
	//plt.plot(tri[:, 0], tri[:, 1], 'g-');
}


class RRT{

public:
	
	RRT(Posture q0, Posture q1, int minX, int minY, int maxX, int maxY, vector<Posture> obstacles){
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
	int goalSampleRate = 80 ;
	int maxIter        = 50;
	double rnd[3];
	vector<Posture> path ;
	vector<Posture> obstacleList;
	vector <Posture> nodeList;
	vector<Posture>  path_list_;
	Posture newNode_;
	ReedsSheppPathGenerator  path_point;
	Posture start, goal;
	
public:
	//return node
	Posture get_random_point(){
		Posture node;
		double rnd[3] = { 0 };
		srand((unsigned)time(NULL));
		if (random(1, 100) > goalSampleRate){
			rnd[0] = 1.0* random(minrandX, maxrandX);
			rnd[1] = 1.0* random(minrandY, maxrandY);
			rnd[2] = 0.01*(random(314, 628)-314);
		}
		else if (random(0, 100) < 30){
			rnd[0] = (goal.x + start.x) / 2;
			rnd[1] = (goal.y + start.y) / 2;
			rnd[2] = (goal.yaw + start.yaw) / 2;
		}
		else{ // # goal point sampling
			rnd[0] = goal.x;
			rnd[1] = goal.y;
			rnd[2] = goal.yaw;
		}

		//node = rnd;
		node.x = rnd[0];
		node.y = rnd[1];
		node.yaw = rnd[2];

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
				distance = (obstacleList[i].yaw + 20) * (obstacleList[i].yaw + 20);
				
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
		
		newNode = path_listnode[path_listnode.size() - 1];

		for (int i = 0; i < path_listnode.size(); ++i){
			//cost_sum += abs(path_list[i].cost);
			newNode.path_x.push_back(path_listnode[i].x);
			newNode.path_y.push_back(path_listnode[i].y);
			newNode.path_yaw.push_back(path_listnode[i].yaw);
		}
		newNode.cost += (path_listnode[path_listnode.size() - 1]).cost;
		
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
			temp = (nodeList[i].x - newNode.x) *(nodeList[i].x - newNode.x) +
				(nodeList[i].y - newNode.y) *(nodeList[i].y - newNode.y) +
				(nodeList[i].yaw - newNode.yaw) *(nodeList[i].yaw - newNode.yaw);
			dlist.push_back(temp);
			if ((temp <= r*r) ){
				nearindex = i;
				nearindexlist.push_back(i);
				break;
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
			    
			if (CollisionCheck(tempNode_, obstacleList)){
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
			bool obstacleOK = CollisionCheck(tNode, obstacleList);
			bool imporveCost = nearNode.cost > tNode.cost;

			if (obstacleOK && imporveCost){
				nodeList[temp].x = tNode.x;
				nodeList[temp].y = tNode.y;
				nodeList[temp].yaw = tNode.yaw;
				nodeList[temp].cost = tNode.cost;
				nodeList[temp].parent_index = tNode.parent_index;
			}
		}
	};

	double calc_dist_to_goal(double x, double y){
		double temp = sqrt((x - goal.x)*(x - goal.x) + (y - goal.y)*(y - goal.y));
		return temp;
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

	vector<Posture> gen_final_course(int goalind) {
		
		
		Posture node;
		Posture rs_node_seg;
		int index = goalind;
		while (nodeList[index].parent_index >-1){
			node = nodeList[index];

			for (int j = 0; j < nodeList[index].path_x.size() - 1; ++j){
				rs_node_seg.x = nodeList[index].path_x[j];
				rs_node_seg.y = nodeList[index].path_y[j];
				rs_node_seg.yaw = nodeList[index].path_yaw[j];
				rs_node_seg.cost = nodeList[index].cost;
				if (j>0){
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

	vector<Posture>&   Planning(bool animation){
			Posture newNode_;
			Posture rnd;
			Posture rs_node_seg;
			Posture temp_newnode;
			int nearestindex;
			vector<int> nearindexlist;
			bool no_collosion;
			
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
				bool nocollision = CollisionCheck(newNode_, obstacleList);
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
};


int main(){

	std::vector<Posture> path_list;
	ReedsSheppPathGenerator  path_point;
	start_time = clock();
	//0. start and goal init 
	Posture start, end;
	double minAreaX, minAreaY;
	double maxAreaX, maxAreaY;

	//int parking_mode = PARALLEL;//INCLINE;//VERTICAL;
	switch (ParkingMode)
	{
	case VERTICAL:
		start.x = 100;
		start.y = 400;
		start.yaw = -pi / 10;

		end.x = 625;
		end.y = 610;
		end.yaw = -pi / 2;

		minAreaX = 300;// 
		maxAreaX = 950;// 
		minAreaY = 100;// 
		maxAreaY = 650;// 
		break;

	case PARALLEL:
		start.x = 1400;
		start.y = 500;
		start.yaw = -pi / 10;

		end.x = 600;
		end.y = 150;
		end.yaw = 0;

		minAreaX = 400;//minArea - 50;
		minAreaY = 50;//minArea - 50;

		maxAreaX = 1200;//maxArea + 50;
		maxAreaY = 1200;//maxArea + 50;

		break;
	case INCLINE:
		start.x = 100;
		start.y = 400;
		start.yaw = -pi / 10;

		end.x = 625;
		end.y = 610;
		end.yaw = -pi / 2;
		minAreaX = 200;//minArea - 50;
		maxAreaX = 1400;//maxArea + 50;
		minAreaY = 200;//minArea - 50;
		maxAreaY = 1400;//maxArea + 50;
		break;
	default:
		break;
	}

	bool   animation = true;
	//1.object
	vector <Posture> obstacleList;
	Posture node;

	//vertical parking
	if (ParkingMode == VERTICAL){
		//VERTICAL
		for (int i = 0; i < 400; ++i){
			node.x   = 500;
			node.y   = 950 - i;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i <400; ++i){
			node.x   = 750;
			node.y   = 950 - i;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 260; ++i){
			node.x   = 500+i;
			node.y   = 950 ;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 1460; ++i){
			node.x =  i;
			node.y = 50;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 500; ++i){
			node.x = i;
			node.y = 550;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 750; i < 1500; ++i){
			node.x = i;
			node.y = 550;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 1500; ++i){
			node.x = i;
			node.y = 1250;
			node.yaw = 25;
			i = i + 15;
			obstacleList.push_back(node);
		}
		//PARALLEL
		//double yaw = pi/4;
	}
	if (ParkingMode == PARALLEL){
		for (int i = 0; i < 280; ++i){
			node.x = 500;
			node.y = i;
			node.yaw = 50;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 280; ++i){
			node.x = 1150;
			node.y = i;
			node.yaw = 50;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 660; ++i){
			node.x = 500 + i;
			node.y = 10;
			node.yaw = 50;
			i = i + 15;
			obstacleList.push_back(node);
		}
		for (int i = 0; i < 1500; ++i){
			node.x =  i;
			node.y = 850;
			node.yaw = 50;
			i = i + 15;
			obstacleList.push_back(node);
		}
	}

	//2.rrt
	//RRT
	bool show_animation = true;
	
	RRT rrt_Path(start, end, minAreaX, minAreaY, maxAreaX, maxAreaY, obstacleList);
	
	//path_list = rrt_Path.Planning(show_animation);
	int loop_time = 0;
	while (path_list.size() < 6 && (loop_time< 120)){
		loop_time++;
		path_list.clear();
		//nodeList.clear();
		path_list = rrt_Path.Planning(show_animation);

	}
	
	//path_list.push_back(start);
	end_time = clock();
	double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	std::cout << "Total time:" << endtime << endl;		//sΪ��λ	
	std::cout << "Total time:" << endtime * 1000 << "ms" << endl;	//msΪ��λ	

	//3 show in image
	if (show_animation){
		string str_text;

		cv::Mat img(1000, 1500, CV_8UC3, Scalar(255, 255, 255));
		//show random point area
		cv::rectangle(img, cvPoint(minAreaX, minAreaY), cvPoint(maxAreaX, maxAreaY), cvScalar(0, 255, 100), 3, 4, 0);
		Point center_start = Point(start.x, start.y);
		Point center_end   = Point(end.x, end.y);
		int radius = 10;
		int linewidth = 3;
		circle(img, center_start, radius, Scalar(0, 0, 255), linewidth);
		 
		cv::line(img, center_start, Point(start.x + 100 * cos(start.yaw), start.y + 100 * sin(start.yaw)), cv::Scalar(0, 0, 255),2);
		str_text = "Start";
		cv::putText(img, str_text, Point(start.x, start.y+50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
		
		circle(img, center_end, radius, Scalar(0, 255, 0), 3);
		cv::line(img, center_end, Point(end.x + 100 * cos(end.yaw), end.y + 100 * sin(end.yaw)), cv::Scalar(0, 255, 0),2);

		str_text = "End";
		cv::putText(img, str_text, Point(end.x, end.y+50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

		//obstacale  show
		for (int i = 0; i < obstacleList.size() ; ++i){
			Point center = Point(obstacleList[i].x, obstacleList[i].y);	 
			circle(img, center,5, Scalar(0, 255, 255), 5);
		}
		//path show

		cv::Point center_pro = Point(path_list[0].x, path_list[0].y);
		for (int i = 0; i < path_list.size(); ++i){
			Point center = Point(path_list[i].x, path_list[i].y);
			str_text = to_string(i);
			cv::putText(img, str_text, center, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 0.5);

			circle(img, center, 7, Scalar(255, 0, i*255/path_list.size()), 3);
			cv::line(img, center, Point(path_list[i].x + 40 * cos(path_list[i].yaw), path_list[i].y +
				40 * sin(path_list[i].yaw)), cv::Scalar(255, 0, 0), 4);

			if (i > 0 && i < path_list.size()){
				cv::Point start = center_pro; 
				cv::Point end = center;   
				cv::line(img, start, end, cv::Scalar(0,0, 255),1.5); 
			}
			center_pro = center;
		}
        //file.write(str(ip[0])+' , '+str(ip[1])+'\n') 
		string str_window = "iFLYTEK_AutoParking_Planning: CostTime = " + to_string(endtime)+"/s";
		imshow(str_window, img);
		waitKey();
		destroyAllWindows();

	}
	return 0;
}
#if 0
int main_rs_ok(){
	ReedsSheppPathGenerator  path_point;
	std::vector<Posture> path_list;
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

	//�뾶
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

		//�뾶
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

		////����ϵ���Ƶ��հ�ͼ��  
		//for (int i = 0; i < points.size(); i++)
		//{
		//	cv::circle(img, points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
		//}
		//cv::RotatedRect rotate_rect = cv::fitEllipse(points);
		////����������?
		//cv::ellipse(img, rotate_rect, cv::Scalar(0, 255, 255), 2, 8);

		////��ȡ��ת���ε��ĸ�����
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