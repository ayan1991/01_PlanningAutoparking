////for rrt-rs-cc
//#include <vector>
//
//using namespace std;
//
//bool show_animation = true;
//const float  STEP_SIZE = 0.1;
//const float  curvature = 0.3;//1.0
//
//typedef struct tagPosture {
//	double x;
//	double y;
//	double yaw;
//	double acc;
//	double speed;
//} Posture;
//
//class Node{
//public:
//	Node() {};
//private:
//	Posture node;
//
//	vector<double> path_x, path_y, path_yaw;
//	double cost;//= 0.0
//	double parent[3];//{0}
//
//}
//class RRT_Generate
//{
//private:
//	
//	 
//	double 	start[3],goal[3];
//	int goalSampleRate ;
//	int maxIter ;
//	vector<Posture> obstacleList;
//
//public:
//	//RRT() {};
//	//Class for RRT Planning
//	//Posture start, end;
//	RRT_Generate(double start[3], double goal[3], vector<Posture> &obstacleList,
//		double randArea[2], int  goalSampleRate = 80, int  maxIter = 200);
//			
//	
//
//	void Planning(){
//		nodeList = start;
//			for i in range(self.maxIter) :
//				rnd = self.get_random_point()
//				nind = self.GetNearestListIndex(self.nodeList, rnd)
//
//				newNode = self.steer(rnd, nind)
//				if newNode is None :
//		continue
//
//			if self.CollisionCheck(newNode, self.obstacleList) :
//				nearinds = self.find_near_nodes(newNode)
//				newNode = self.choose_parent(newNode, nearinds)
//				if newNode is None :
//		continue
//			self.nodeList.append(newNode)
//			self.rewire(newNode, nearinds)
//
//			if animation and i % 5 == 0 :
//				self.DrawGraph(rnd = rnd)
//
//				// generate coruse
//				lastIndex = self.get_best_last_index()
//				if lastIndex is None :
//		return None
//			path = self.gen_final_course(lastIndex)
//			return path
//	}
//	void choose_parent(newNode, nearinds) {
//		if len(nearinds) == 0 :
//			return newNode
//
//			dlist = []
//			for i in nearinds :
//	tNode = self.steer(newNode, i)
//		if tNode is None :
//	continue
//
//		if self.CollisionCheck(tNode, self.obstacleList) :
//			dlist.append(tNode.cost)
//		else :
//		dlist.append(float("inf"))
//
//		mincost = min(dlist)
//		minind = nearinds[dlist.index(mincost)]
//
//		if mincost == float("inf") :
//			print("mincost is inf")
//			return newNode
//
//			newNode = self.steer(newNode, minind)
//
//			return newNode
//
//			void pi_2_pi(self, angle) :
//			return (angle + math.pi) % (2 * math.pi) - math.pi
//
//			void steer(self, rnd, nind) :
//
//			nearestNode = self.nodeList[nind]
//
//			px, py, pyaw, mode, clen = reeds_shepp_path_planning.reeds_shepp_path_planning(
//			nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, curvature, STEP_SIZE)
//
//			if px is None :
//	return None
//
//		newNode = copy.deepcopy(nearestNode)
//		newNode.x = px[-1]
//		newNode.y = py[-1]
//		newNode.yaw = pyaw[-1]
//
//		newNode.path_x = px
//		newNode.path_y = py
//		newNode.path_yaw = pyaw
//		newNode.cost += sum([abs(c) for c in clen])
//		newNode.parent = nind
//
//		return newNode
//
//		void get_random_point(self) :
//
//		if random.randint(0, 100) > self.goalSampleRate:
//	rnd = [random.uniform(self.minrand, self.maxrand),
//		random.uniform(self.minrand, self.maxrand),
//		random.uniform(-math.pi, math.pi)
//	]
//	elif random.randint(0, 100) <30 :
//	rnd = [(self.end.x + start[0]) / 2, (self.end.y + start[1]) / 2, (self.end.yaw + start[2]) / 2]
//		else:  // goal point sampling
//	rnd = [self.end.x, self.end.y, self.end.yaw]
//
//		node = Node(rnd[0], rnd[1], rnd[2])
//
//		return node
//
//		void get_best_last_index(self) :
//		//  print("get_best_last_index")
//
//		YAWTH = math.radians(3.0)
//		XYTH = 0.5
//
//		goalinds = []
//		for (i, node) in enumerate(self.nodeList) :
//			if self.calc_dist_to_goal(node.x, node.y) <= XYTH :
//				goalinds.append(i)
//				//  print("OK XY TH num is")
//				//  print(len(goalinds))
//
//				// angle check
//				fgoalinds = []
//				for i in goalinds :
//	if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH :
//		fgoalinds.append(i)
//		//  print("OK YAW TH num is")
//		//  print(len(fgoalinds))
//
//		if len(fgoalinds) == 0 :
//			return None
//
//			mincost = min([self.nodeList[i].cost for i in fgoalinds])
//			for i in fgoalinds :
//	if self.nodeList[i].cost == mincost :
//		return i
//
//		return None
//
//		void gen_final_course(self, goalind) :
//		path = [[self.end.x, self.end.y]]
//		while self.nodeList[goalind].parent is not None :
//	node = self.nodeList[goalind]
//		for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)) :
//			path.append([ix, iy])
//			//  path.append([node.x, node.y])
//			goalind = node.parent
//			path.append([self.start.x, self.start.y])
//			return path
//
//			void calc_dist_to_goal(self, x, y) :
//			return np.linalg.norm([x - self.end.x, y - self.end.y])
//
//			void find_near_nodes(self, newNode) :
//			nnode = len(self.nodeList)
//			r = 50.0 * math.sqrt((math.log(nnode) / nnode))
//			//  r = self.expandDis * 5.0
//			dlist = [(node.x - newNode.x) ** 2 +
//			(node.y - newNode.y) ** 2 +
//			(node.yaw - newNode.yaw) ** 2
//			for node in self.nodeList]
//				nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
//				return nearinds
//
//				void rewire(self, newNode, nearinds) :
//
//				nnode = len(self.nodeList)
//
//				for i in nearinds :
//	nearNode = self.nodeList[i]
//		tNode = self.steer(nearNode, nnode - 1)
//		if tNode is None :
//	continue
//
//		obstacleOK = self.CollisionCheck(tNode, self.obstacleList)
//		imporveCost = nearNode.cost > tNode.cost
//
//		if obstacleOK and imporveCost :
//	//  print("rewire")
//	self.nodeList[i] = tNode
//
//		void DrawGraph(self, rnd = None) :
//		"""
//		Draw Graph
//		"""
//		plt.clf()
//		if rnd is not None :
//	plt.plot(rnd.x, rnd.y, "^k")
//		for node in self.nodeList :
//			if node.parent is not None :
//	plt.plot(node.path_x, node.path_y, "-b")
//		plt.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y], "-g")
//
//		for (ox, oy, size) in self.obstacleList :
//			plt.plot(ox, oy, "oy", ms = 30 * size)
//
//			reeds_shepp_path_planning.plot_arrow(
//			self.start.x, self.start.y, self.start.yaw)
//			reeds_shepp_path_planning.plot_arrow(
//			self.end.x, self.end.y, self.end.yaw)
//
//			plt.axis([-2, 20, -2, 20])
//			plt.grid(True)
//			plt.pause(0.01)
//
//			//  plt.show()
//			//  input()
//
//			void GetNearestListIndex(self, nodeList, rnd) :
//			dlist = [(node.x - rnd.x) ** 2 +
//			(node.y - rnd.y) ** 2 +
//			((node.yaw - rnd.yaw) ** 2)*1.2 for node in nodeList]
//			minind = dlist.index(min(dlist))
//
//			return minind
//
//			void CollisionCheck(self, node, obstacleList) :
//
//			for (ox, oy, size) in obstacleList :
//	for (ix, iy) in zip(node.path_x, node.path_y) :
//		dx = ox - ix
//		dy = oy - iy
//		d = 0.8*dx * dx + 1.2* dy * dy
//		if d <= size ** 2 :
//			return False  // collision
//
//			return True  // safe
//
//
//	class Node() :
//		"""
//		RRT Node
//		"""
//
//		void __init__(self, x, y, yaw) :
//		self.x = x
//		self.y = y
//		self.yaw = yaw
//		self.path_x = []
//		self.path_y = []
//		self.path_yaw = []
//		self.cost = 0.0
//		self.parent = None
//
//
//void main() {
//	//print("Start rrt start planning")
//	//file = open('pathList.txt', 'w')
//
//	// ====Search Path with RRT====
//	// [x,y,size(radius)]
//	std::vector<Posture> obstacleList;
//
//
//	// Set Initial parameters
//	double start[3] = { -1.0, 7.0, math.radians(0.0) };
//	double goal[3] = { 3.85, 4.0, math.radians(90.0) };
//	RRT rrt_path;
//	rrt = rrt_path.RRT(start, goal, randArea = [-2.0, 20.0], obstacleList = obstacleList)
//	path = rrt.Planning(animation = show_animation)
//
//			// Draw final path
//	//if show_animation:
//	//	rrt.DrawGraph()
//	//		plt.plot([x for (x, y) in path], [y for (x, y) in path], 'or', 3)
//	//		//plt.arrow([x for (x, y) in path], [y for (x, y) in path], [math.cos(yaw) for (x, yaw) in path], math.sin([yaw for (x, yaw) in path]), "r", "b", 1, 1)
//	//		plt.plot(x, y)
//	//		for ip in path :
//		//print "ip = ", ip
//			//file.write(str(ip[0]) + ' , ' + str(ip[1]) + '\n')
//			//file.write(ip[1])
//			//file.write('\n')
//			//file.close()
//			//plt.grid(True)
//			//plt.pause(0.001)
//
//			//plt.show()
//}