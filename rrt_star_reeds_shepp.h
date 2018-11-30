//#ifndef __RRT_STAR_REEDS_SHEPP_HPP
//#define __RRT_STAR_REEDS_SHEPP_HPP
//
//using namespace __shedskin__;
//namespace __rrt_star_reeds_shepp__ {
//
//class RRT;
//class Node;
//
//
//extern str *__name__;
//extern __ss_bool show_animation;
//extern double STEP_SIZE, curvature;
//extern void *reeds_shepp_path_planning;
//
//
//extern class_ *cl_RRT;
//class RRT : public pyobj {
///**
//Class for RRT Planning
//*/
//public:
//    __ss_int maxIter;
//    double maxrand;
//    Node *end;
//    Node *start;
//    list<tuple2<double, double> *> *obstacleList;
//    list<Node *> *nodeList;
//    __ss_int goalSampleRate;
//    double minrand;
//
//    RRT() {}
//    RRT(list<double> *start, list<double> *goal, list<tuple2<double, double> *> *obstacleList, list<double> *randArea, __ss_int goalSampleRate, __ss_int maxIter) {
//        this->__class__ = cl_RRT;
//        __init__(start, goal, obstacleList, randArea, goalSampleRate, maxIter);
//    }
//    Node *get_random_point();
//    void *get_best_last_index();
//    list<double> *calc_dist_to_goal(double x, double y);
//    void *DrawGraph(Node *rnd);
//    __ss_int GetNearestListIndex(list<Node *> *nodeList, Node *rnd);
//    __ss_bool CollisionCheck(Node *node, list<tuple2<double, double> *> *obstacleList);
//    Node *choose_parent(Node *newNode, list<__ss_int> *nearinds);
//    list<list<double> *> *gen_final_course(void *goalind);
//    void *rewire(Node *newNode, list<__ss_int> *nearinds);
//    list<list<double> *> *Planning(__ss_bool animation);
//    void *__init__(list<double> *start, list<double> *goal, list<tuple2<double, double> *> *obstacleList, list<double> *randArea, __ss_int goalSampleRate, __ss_int maxIter);
//    Node *steer(Node *rnd, __ss_int nind);
//    list<__ss_int> *find_near_nodes(Node *newNode);
//};
//
//extern class_ *cl_Node;
//class Node : public pyobj {
//	/**
//	RRT Node
//	*/
//public:
//	double cost;
//	double x;
//	double yaw;
//}
//
//#endif
