

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
#include <math>
using namespace  cv;
using namespace  std;

#define random(a,b) (rand()%(b-a+1)+a)

const float STEP_SIZE = 0.1;
const float curvature = 0.3;//#1.0

const float  step_size = 50;
const float  rho = 430; // turning radius

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
struct  NodeRRT{

	double x, y, yaw;
	vector<double> path_x, path_y, path_yaw;
	double cost;
	Posture	parent;

};

class RRT{
public:
	RRT();
	~RRT();

	/*RRT(Posture & start, Posture & goal, double minArea, double maxArea,
		double goalSampleRate,double maxIter, vector <Point3d> &obstacleList){
		}*/
private:
	Posture  start;
	Posture  goal;
	//start = NodeRRT(start.x, start.y, start.yaw);
	//self.end = Node(goal[0], goal[1], goal[2])
	int  minrand;
	int  maxrand;
	int goalSampleRate;
	int maxIter;
	double rnd[3];
	vector<Posture> obstacleList;
	vector <Posture> nodeList;
	vector<Posture> parent;
public:
	void get_random_point(double node[3]){
		double rnd[3] = { 0 };
		srand((unsigned)time(NULL));
		if (random(1, 100) > goalSampleRate){

			rnd[0] = 1.0* random(minrand, maxrand);
			rnd[1] = 1.0*random(minrand, maxrand);
			rnd[2] = 0.01*random(-314, 314);
		}
		else if (random(0, 100) < 30){
			//rnd = [(self.end.x + start[0]) / 2, (self., (self.end.yaw + start[2]) / 2]
			rnd[0] = (goal.x + start.x) / 2;
			rnd[1] = (goal.y + start.y) / 2;
			rnd[2] = (goal.yaw + start.yaw) / 2;
		}
		else{ // # goal point sampling
			rnd[0] = goal.x;
			rnd[1] = goal.y;
			rnd[2] = goal.yaw;
		}
		node = rnd;

		return;
	}
	void GetNearestListIndex(vector <Posture> nodeList, double rnd[3], double nearest_node[3], double minindex) {
		vector<double> dlist;
		//double nearest_node[3];
		double cost = 0;
		double min_cost = 10000;
		for (int i = 0; i < nodeList.size(); ++i){
			cost = (nodeList[i].x - rnd[0]) *(nodeList[i].x - rnd[0])
				+ (nodeList[i].y - rnd[1]) *(nodeList[i].y - rnd[1])
				+ (nodeList[i].yaw - rnd[2]) *(nodeList[i].yaw - rnd[2])*1.2;
			dlist.push_back(cost);
			if (cost < min_cost){
				min_cost = cost;
				nearest_node[0] = nodeList[i].x;
				nearest_node[1] = nodeList[i].y;
				nearest_node[2] = nodeList[i].yaw;
				minindex = i;
			}
			else{
				std::cout << "[ not the best node  ]" << std::endl;
			}
		}
		return;
	}
	bool CollisionCheck(vector<Posture> node, vector<Posture> obstacleList){
		double dx, dy, dyaw, cost_d;
		for (int i = 0; i < obstacleList.size(); ++i){
			for (int j = 0; j < node.size(); ++j) {
				dx = obstacleList[i].x - node[j].x;
				dx = obstacleList[i].x - node[j].x;
				cost_d = 0.8*dx * dx + 1.2* dy * dy;
				if (cost_d <= obstacleList[i].yaw * obstacleList[i].yaw) {
					return false; // # collision
				}
			}
		}
		return true;  //# safe
	}
	//todo
	//(random point/nearest point/)
	void steer(double rnd[3], double nind[3], vector<Posture> newNode) {


		Posture nearestNode;
		Posture randNode;
		double cost_sum;
		nearestNode.x = nind[0];
		nearestNode.y = nind[1];
		nearestNode.yaw = nind[2];

		randNode.x = rnd[0];
		randNode.y = rnd[1];
		randNode.yaw = rnd[2];
		path_list = path_point.Generate(nearestNode, randNode, step_size, rho);

		if (path_list.size() == 0){
			return;
		}


		for (int i = 0; i < path_list.size(); ++i){
			cost_sum += abs(path_list[i].cost);
		}
		path_list[path_list.size() - 1].cost = cost_sum;
		//newNode.cost
		//	newNode.parent = nind;
		parent.push_back(nearestNode);
		newNode.push_back(path_list.back());
		return;
	}


	void find_near_nodes( Posture newNode) {
		double nodeNum = nodeList.size();
		double r = 50.0 * sqrt((log(nodeNum) / nodeNum));
		// r = self.expandDis * 5.0
		double dlist;
		for (int i = 0; i < nodeNum-1; ++i){
			
			dlist = (nodeList[i].x - newNode.x) *(nodeList[i].x - newNode.x) +
				(nodeList[i].y - newNode.y) *(nodeList[i].y - newNode.y) +
				(nodeList[i].yaw - newNode.yaw) *(nodeList[i].yaw - newNode.yaw);

	
		}
		
		return ;
	}

	void planning(bool animation,vector<Posture> & path){
		
		 vector <Posture> newNode;
		 double minindex;
		 nodeList.push_back(start);
		 nodeList.push_back(goal);
		 double nind[3];
		 //self.nodeList = [self.start];
		for (int i = 0; i < maxIter; ++i) {
			get_random_point(rnd);
			GetNearestListIndex(nodeList, rnd, nind,minindex);

			steer(rnd, nind, newNode);
			if (nodeList.size() == 0){
				continue;
			}
			if (CollisionCheck(newNode, obstacleList) ){
				nearinds = find_near_nodes(newNode);
				newNode = choose_parent(newNode, nearinds);
				if (newNode.size() == 0){
					continue;
				}
				nodeList.append(newNode);
				rewire(newNode, nearinds);
			}
			//if( animation and i % 5 == 0) {
				//DrawGraph(rnd = rnd);
			//}
		}
				 //# generate coruse
		lastIndex = get_best_last_index();
		if (lastIndex is None){
			return None;
		}
		path = sgen_final_course(lastIndex);
	 
	 };
	 choose_parent(){
	 
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
		 return fmod((angle + pi),2.0* pi) - pi;
	 }

	 Posture steer(Posture rnd, Posture nind){
		 std::vector<Posture>  nodeList;
		 Posture nearestNode = nodeList[0];

			 /*px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
			 nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature, STEP_SIZE)*/
		 //Posture start,end;

		 //start.x = nearestNode.x;

		 path_point.Generate(nearestNode, rnd, curvature, STEP_SIZE);

		 if ( nodeList.capacity){
				 return None;
			 }
			 newNode = copy.deepcopy(nearestNode)
			 newNode.x = px[-1]
			 newNode.y = py[-1]
			 newNode.yaw = pyaw[-1]

			 newNode.path_x = px
			 newNode.path_y = py
			 newNode.path_yaw = pyaw
			 newNode.cost += sum([abs(c) for c in clen])
			 newNode.parent = nind

			 return newNode;
	 };
private:
	ReedsSheppPathGenerator  path_point;
	std::vector<Posture> path_list;
	NodeRRT  newnode;
};

int main(){
	//0. start and goal init
	std::vector<Posture> path_list;
	ReedsSheppPathGenerator  path_point;
	Posture start, end;
	start.x = 100;
	start.y = 100;
	start.yaw = 0;

	end.x = 600;
	end.y = 910;
	end.yaw = pi / 2;

	double minArea = 0;
	double maxArea = 1000;
	double goalSampleRate = 80;
	double maxIter = 200;
	bool   animation = true;
	//1.object
	vector <Point3d> obstacleList;
	Point3d node;
	for (int i = 0; i < 500;++i){
		node.x = 0;
		node.y = i;
		node.z = 0;
		obstacleList.push_back(node);
	}
	
	//2.rrt
	//RRT
	RRT rrt_Path;
	rrt_Path.planning(animation, path_list);
	
	//rrt = RRT(start, goal, randArea = [-2.0, 20.0], obstacleList = obstacleList);
	//path = rrt.Planning(animation = show_animation);

	//3 show in image

	return 0;
}

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
		std::cout << "dist = " << path_list[i].x << std::endl;
		std::cout << "dist = " << path_list[i].y << std::endl;
		std::cout << "dist = " << path_list[i].yaw << std::endl;

		Point center = Point(path_list[i].x, path_list[i].y);
		std::cout << "center: " << center << std::endl;
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

	//半径
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
		cout << "center: " << center << endl;
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
	std::cout << "[ RRT_start_cc: ] path planner sample start!!" << std::endl;

	float start_x   = 800.0;  // [m]
	float start_y   = 500.0;  // [m]
	float start_yaw = radians(0.0);  // [rad]

	float end_x   = 500.0;  // [m]
	float end_y   = 600.0;  // [m]
	float end_yaw = radians(95.0);  // [rad]
	
	Point center_start = Point(start_x, start_y);
	Point center_end   = Point(end_x, end_y);

	//curvature = 2;//1
	float step_size = 10;//0.1
	
	reeds_shepp_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size, paths,path);
	std::cout << "[ RRT_start_cc: ] show img" << std::endl;

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

		//半径
		int r = 10;
		//int i = 1;
		
		for (int index = 0; index < paths.size(); ++index){
			for (int i = 0; i < paths[index].x.size(); ++i){
				x = (paths[index].x)[i];
				y = (paths[index].y)[i];
				Point center = Point(x, y);
				std::cout << "[ path :] x = " << x << ";y = " << y << std::endl;
				std::cout << "center: " << center << std::endl;
				circle(img, center, r, Scalar(255, 255, 0), 3);
			}
		}
		for (int i = 0; i < path.x.size(); ++i)
		{
			x = (path.x)[i];
			y = (path.y)[i];
			std::cout << "x = " << x << ";y = " << y << std::endl;
			//i = i + 1;
			Point center = Point(x, y);
			std::cout << "center: " << center << std::endl;
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

		////将拟合点绘制到空白图上  
		//for (int i = 0; i < points.size(); i++)
		//{
		//	cv::circle(img, points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
		//}
		//cv::RotatedRect rotate_rect = cv::fitEllipse(points);
		////绘制拟合椭圆
		//cv::ellipse(img, rotate_rect, cv::Scalar(0, 255, 255), 2, 8);

		////获取旋转矩形的四个顶点
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
