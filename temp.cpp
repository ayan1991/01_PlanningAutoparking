//#include <opencv2\opencv.hpp>
//#include <iostream>
//#include <iomanip>
//#include <fstream>
//#include<string>
//#include<stdio.h>
//#include <boost/math/constants/constants.hpp>
//
//using namespace  cv;
//using namespace  std;
//
//
//template<class Type>
//
//bool show_animation = True
//const float STEP_SIZE = 0.1
//const float curvature = 0.3#1.0
//
//Type stringToNum(const string& str)
//{
//	istringstream iss(str);
//	Type num;
//	iss >> num;
//	return num;
//}
///**
//class RRT1{
//
//private:
//
//typedef struct tagCoord3Df {
//float x;
//float y;
//float yaw;
//} Coord3Df;
//typedef struct tagCoord2Df {
//float x;
//float y;
////float yaw;
//} Coord2Df;
//Coord3Df start, goal;
//vector<Coord3Df> obstacleList;
//vector<Coord2Df> randArea;
//int goalSampleRate = 90;
//int maxIter = 100;
//
//public:
////RRT(char *name, int age, float score);
////RRT(start, goal, randArea = [-2.0, 15.0], obstacleList = obstacleList);
//
//void show();
//};
//*/
//class RRT {
//private:
//
//	typedef struct tagCoord3Df {
//		float x;
//		float y;
//		float yaw;
//	} Coord3Df;
//	typedef struct tagCoord2Df {
//		float width;
//		float hight;
//		//float yaw;
//	} Coord2Df;
//	vector<Coord3Df> obstacleList;
//	//Class for RRT Planning;
//	//def __init__(  start, goal, obstacleList, randArea,
//	//goalSampleRate = 90, maxIter = 100) :
//
//	//Setting Parameter;
//
//	//start: Start Position[x, y];
//	//goal: Goal Position[x, y];
//	//obstacleList: obstacle Positions[[x, y, size], ...];
//	//randArea: Ramdom Samping Area[min, max];
//public:
//	RRT(Coord3Df start, Coord3Df goal, Coord2Df randArea, vector<Coord3Df>   obstacleList){
//
//		PointNode start = PointNode(start.x, start.y, start.yaw);
//		end = Node(goal.x, goal.y, goal.yaw);
//		int minrand = randArea.width;
//		int maxrand = randArea.hight;
//		goalSampleRate = goalSampleRate;
//		maxIter = maxIter;
//		obstacleList = obstacleList;
//	}
//	int Planning(animation = True) {
//
//		//	Pathplanning
//
//		//animation : flag for animation on or off
//		//	
//
//		nodeList = [start]
//			for i in range(maxIter) :
//				rnd = get_random_point()
//				nind = GetNearestListIndex(nodeList, rnd)
//
//				newNode = steer(rnd, nind)
//				if newNode is None :
//		continue
//			if  CollisionCheck(newNode, obstacleList) :
//				nearinds = find_near_nodes(newNode);
//		newNode = choose_parent(newNode, nearinds);
//		if newNode is None :
//		continue
//			nodeList.append(newNode);
//		rewire(newNode, nearinds);
//
//		if animation and i % 5 == 0 :
//		DrawGraph(rnd = rnd);
//
//		// generate coruse
//		lastIndex = get_best_last_index();
//		if lastIndex is None :
//		return None;
//		path = gen_final_course(lastIndex);
//		return path;
//	}
//	int choose_parent(newNode, nearinds) {
//		if len(nearinds) == 0 :
//		return newNode;
//
//
//		dlist = []
//			for i in nearinds :
//		tNode = steer(newNode, i)
//			if tNode is None :
//		continue
//
//			if  CollisionCheck(tNode, obstacleList) :
//				dlist.append(tNode.cost)
//			else :
//			dlist.append(float("inf"))
//
//			mincost = min(dlist)
//			minind = nearinds[dlist.index(mincost)]
//
//			if mincost == float("inf") :
//				print("mincost is inf")
//				return newNode
//
//				newNode = steer(newNode, minind)
//
//				return newNode
//	}
//	float pi_2_pi(angle) {
//		return (angle + math.pi) % (2 * math.pi) - math.pi
//	}
//	int steer(rnd, nind) {
//
//		nearestNode = nodeList[nind];
//
//		px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
//			nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature, STEP_SIZE);
//
//		if px is None :
//		return None;
//
//		newNode = copy.deepcopy(nearestNode);
//		newNode.x = px[-1];
//		newNode.y = py[-1];
//		newNode.yaw = pyaw[-1];
//
//		newNode.path_x = px;
//		newNode.path_y = py;
//		newNode.path_yaw = pyaw;
//		newNode.cost += sum([abs(c) for c in clen]);
//		newNode.parent = nind;
//
//		return newNode;
//	}
//	int get_random_point(self) {
//
//		if random.randint(0, 100) > goalSampleRate:
//		rnd = [random.uniform(minrand, maxrand),
//			random.uniform(minrand, maxrand),
//			random.uniform(-math.pi, math.pi)
//		]
//		elif random.randint(0, 100) < 30 :
//		rnd = [(end.x + 15.0) / 2, (end.y + 15.0) / 2, (end.yaw + 20.0) / 2]
//		else: // goal point sampling
//		rnd = [end.x, end.y, end.yaw]
//
//			node = Node(rnd[0], rnd[1], rnd[2])
//
//			return node
//	}
//	int  get_best_last_index(self) {
//		// print("get_best_last_index")
//
//		YAWTH = math.radians(3.0);
//		XYTH = 0.5;
//
//		goalinds = []
//			for (i, node) in enumerate(nodeList) :
//				if  calc_dist_to_goal(node.x, node.y) <= XYTH :
//					goalinds.append(i)
//					//  print("OK XY TH num is")
//					//  print(len(goalinds))
//
//					// angle check
//					fgoalinds = [];
//		for i in goalinds :
//		if abs(nodeList[i].yaw - end.yaw) <= YAWTH :
//			fgoalinds.append(i);
//		//  print("OK YAW TH num is")
//		//  print(len(fgoalinds))
//
//		if (len(fgoalinds)) == 0{
//			return None;
//		}
//		mincost = min([nodeList[i].cost for i in fgoalinds])
//			for i in fgoalinds :
//		if  nodeList[i].cost == mincost :
//			return i
//
//			return None
//	}
//	int gen_final_course(goalind) {
//		path = [[end.x, end.y]]
//			while  nodeList[goalind].parent is not None :
//		node = nodeList[goalind]
//			for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)) :
//				path.append([ix, iy])
//				//  path.append([node.x, node.y])
//				goalind = node.parent
//				path.append([start.x, start.y])
//				return path
//	}
//	int calc_dist_to_goal(x, y) {
//		return np.linalg.norm([x - end.x, y - end.y])
//	}
//	int find_near_nodes(newNode) {
//		nnode = len(nodeList)
//			r = 50.0 * math.sqrt((math.log(nnode) / nnode))
//			// r = expandDis * 5.0
//			dlist = [(node.x - newNode.x) ** 2 +
//			(node.y - newNode.y) ** 2 +
//			(node.yaw - newNode.yaw) ** 2
//			for node in  nodeList]
//				nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
//				return nearinds
//
//				def rewire(newNode, nearinds) :
//
//				nnode = len(nodeList)
//
//				for i in nearinds :
//		nearNode = nodeList[i]
//			tNode = steer(nearNode, nnode - 1)
//			if tNode is None :
//		continue
//
//			obstacleOK = CollisionCheck(tNode, obstacleList)
//			imporveCost = nearNode.cost > tNode.cost
//
//			if obstacleOK and imporveCost :
//		//  print("rewire")
//		nodeList[i] = tNode
//	}
//	void DrawGraph(vector < vector<float>  > rnd) {
//		//Draw Graph
//		//plt.clf();
//		if (rnd.size() = 0){
//			//plt.plot(rnd.x, rnd.y, "^k");
//			std::cout << "rnd list is empty" << std::endl;
//		}
//		for (int i = 0; i < rnd.size(); i++){
//			if (i = 0){
//				rnd_ = rnd;
//			}
//			if (rnd_ != NULL){
//				//plt.plot(node.path_x, node.path_y, "-g");
//
//
//			}
//		}
//		//  plt.plot([node.x,  nodeList[node.parent].x], [
//		//  node.y,  nodeList[node.parent].y], "-g")
//		/*
//		for (ox, oy, size) in  obstacleList{
//		plt.plot(ox, oy, "ok", ms = 30 * size);
//		}
//		reeds_shepp_path_planning.plot_arrow(
//		start.x, start.y, start.yaw);
//		reeds_shepp_path_planning.plot_arrow(
//		end.x, end.y, end.yaw);
//		*/
//		//plt.axis([-2, 20, -2, 20]);
//		//plt.grid(True);
//		//plt.pause(0.01);
//
//		//  plt.show()
//		//  input()
//	}
//	int GetNearestListIndex(nodeList, rnd) {
//		dlist = [(node.x - rnd.x) ** 2 +
//			(node.y - rnd.y) ** 2 +
//			(node.yaw - rnd.yaw) ** 2 for node in nodeList];
//		minind = dlist.index(min(dlist));
//
//		return minind;
//	}
//	bool CollisionCheck(node, obstacleList) {
//
//		for (ox, oy, size) in obstacleList{
//			for (ix, iy) in zip(node.path_x, node.path_y) {
//				dx = ox - ix;
//				dy = oy - iy;
//				d = dx * dx + dy * dy;
//				if (d <= size ** 2){
//					return False;//  # collision
//				}
//			}
//		}
//		return True; // safe
//	}
//}
//typedef struct pointNode{
//	float x;
//	float y;
//	float yaw;
//
//	vector<float> path_x, path_y, path_yaw;
//	float cost = 0.0;
//	char* parent = NULL;
//}PointNode;
//float Angle2Rad(float angle){
//	float Rad;
//	Rad = angle / 57.68f;
//	return Rad;
//}
//int main(int argc, char* argv[])
//{
//	std::cout << "Start rrt star with cc planning" << std::endl;
//
//	// ====Search Path with RRT====
//	float node[3] = { 0 };
//	//float obstacleList[][3] = { 0 };
//	vector < vector<float>  >  obstacleList1;
//	obstacleList1 = { { 2.5, 0, 0.1 }, { 2.5, 0.4, 0.1 } };
//	//obstacleList1.pushback({ 2.5, 0, 0.1 });
//
//	//obstacleList1.pushback({ 2.5, 0.4, 0.1 });
//	//}; // [x, y, size(radius)]
//	//Set Initial parameters
//	float start[3] = { 15.0, 15.0, Angle2Rad(20.0) };
//	float goal[3] = { 4.20, 3.0, Angle2Rad(90.0) };
//	//obstacleList1.push_back(start[3]);
//	RRT rrt;
//	//rrt = RRT(start, goal, randArea = [-2.0, 15.0], obstacleList = obstacleList);
//	//path = rrt.Planning(animation = show_animation);
//
//	// Draw final path
//	if (1){
//		rrt.DrawGraph(obstacleList1);
//		//plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r');
//		//plt.grid(True);
//		//plt.pause(0.001);
//
//		//plt.show();
//	}
//	return 0;
//}
//
//int DrawPath()//int argc,char* argv[]
//{
//	//1.read motionplanning result(x,y) from txt
//	ifstream  fin("in_mp_result.txt");
//	string s;
//	//float x;
//	string slid = ",";
//	vector<float> point;
//	int x, y;
//	const char* vehicle_img = "car.jpg";
//	cv::Mat img_car = imread(vehicle_img);
//
//	cv::Mat img(1500, 1500, CV_8UC3, Scalar(255, 255, 255));
//	if (img_car.empty()){
//		fprintf(stderr, "cannot load img %s\n", img_car);
//		return -1;
//	}
//
//	cv::Rect roi_rect = cv::Rect(128, 128, img_car.cols, img_car.rows);
//	img_car.copyTo(img(roi_rect));
//
//	//°ë¾¶
//	int r = 5;
//
//	while (fin >> s)
//	{
//		if (s == ",")
//		{
//			continue;
//		}
//		point.push_back(std::stof(s));
//		//cout << "Read from file: " << std::stof(s) << endl;
//	}
//	for (int i = 0; i < point.size(); i++)
//	{
//		x = 80 * point[i];
//		y = 80 * point[i + 1];
//		i = i + 1;
//		Point center = Point(x, y);
//		cout << "center: " << center << endl;
//		circle(img, center, r, Scalar(0, 255, 0), 3);
//	}
//
//	imshow("base", img);
//	waitKey();
//	destroyAllWindows();
//	return 0;
//}
