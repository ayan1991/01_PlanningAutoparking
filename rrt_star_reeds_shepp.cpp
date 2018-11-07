//#include "builtin.hpp"
//
///**
//
//
//*/
//
//namespace __rrt_star_reeds_shepp__ {
//
//
//str *__name__;
//void *reeds_shepp_path_planning;
//__ss_bool show_animation;
//double STEP_SIZE, curvature;
//
//
//__ss_bool  default_0;
//Node * default_1;
//
//
///**
//class RRT
//*/
//
//class_ *cl_RRT;
//
//Node *RRT::get_random_point() {
//    Node *node;
//    list<double> *rnd;
//
//  
//    return node;
//}
//
//void *RRT::get_best_last_index() {
//    
//
//    YAWTH = __math__::radians(3.0);
//    XYTH = 0.5;
//    
//
//    return NULL;
//}
//
//list<double> *RRT::calc_dist_to_goal(double x, double y) {
//    
//    return (new list<double>(2,(x-(this->end)->x),(y-(this->end)->y)));
//}
//
//void *RRT::DrawGraph(Node *rnd) {
//    /**
//    Draw Graph
//    */
//    
//    return NULL;
//}
//
//__ss_int RRT::GetNearestListIndex(list<Node *> *nodeList, Node *rnd) {
//    __ss_int minind;
//    list<double> *dlist;
//
//    dlist = list_comp_4(nodeList, rnd);
//    minind = dlist->index(___min(1, 0, dlist));
//    return minind;
//}
//
//__ss_bool RRT::CollisionCheck(Node *node, list<tuple2<double, double> *> *obstacleList) {
//    list<tuple2<double, double> *> *__59;
//    list<tuple2<void *, void *> *>::for_in_loop __70;
//    list<tuple2<double, double> *>::for_in_loop __62;
//    __iter<tuple2<void *, void *> *> *__65;
//    __iter<tuple2<double, double> *> *__60;
//    tuple2<void *, void *> *__63;
//    tuple2<double, double> *__58;
//    void *d, *dx, *dy, *ix, *iy;
//    double ox, oy, size;
//    list<tuple2<void *, void *> *> *__64;
//    __ss_int __61, __66, __69;
//    list<void *> *__67, *__68;
//
//
//    
//
//    return True;
//}
//
//Node *RRT::choose_parent(Node *newNode, list<__ss_int> *nearinds) {
//    __iter<__ss_int> *__5;
//    Node *tNode;
//    double mincost;
//    list<__ss_int> *__4;
//    list<double> *dlist;
//    list<__ss_int>::for_in_loop __7;
//    __ss_int __6, i, minind;
//
//    if ((len(nearinds)==0)) {
//        return newNode;
//    }
//    dlist = (new list<double>());
//
//    
//
//    mincost = ___min(1, 0, dlist);
//    minind = nearinds->__getfast__(dlist->index(mincost));
//    if ((mincost==__float(const_0))) {
//        print2(NULL,0,1, const_1);
//        return newNode;
//    }
//    newNode = this->steer(newNode, minind);
//    return newNode;
//}
//
//list<list<double> *> *RRT::gen_final_course(void *goalind) {
//    list<tuple2<void *, void *> *>::for_in_loop __38;
//    __iter<void *> *__35, *__36;
//    Node *node;
//    __iter<tuple2<void *, void *> *> *__33;
//    tuple2<void *, void *> *__31;
//    list<list<double> *> *path;
//    void *ix, *iy;
//    list<tuple2<void *, void *> *> *__32;
//    __ss_int __34, __37;
//
//    
//
//  
//    path->append((new list<double>(2,(this->start)->x,(this->start)->y)));
//    return path;
//}
//
//void *RRT::rewire(Node *newNode, list<__ss_int> *nearinds) {
//    __iter<__ss_int> *__48;
//    Node *nearNode, *tNode;
//    __ss_bool __51, __52, imporveCost, obstacleOK;
//    list<Node *> *__53;
//    list<__ss_int> *__47;
//    list<__ss_int>::for_in_loop __50;
//    __ss_int __49, i, nnode;
//
//    nnode = len(this->nodeList);
//
//   
//
//    return NULL;
//}
//
//list<list<double> *> *RRT::Planning(__ss_bool animation) {
//    /**
//    Pathplanning
//    
//    animation: flag for animation on or off
//    */
//    Node *newNode, *rnd;
//    __ss_bool __2, __3;
//    void *lastIndex;
//    list<list<double> *> *path;
//    list<__ss_int> *nearinds;
//    __ss_int __0, __1, i, nind;
//
//    this->nodeList = (new list<Node *>(1,this->start));
//
//    
//
//    lastIndex = this->get_best_last_index();
//    if ((lastIndex==NULL)) {
//        return NULL;
//    }
//    path = this->gen_final_course(lastIndex);
//    return path;
//}
//
//void *RRT::__init__(list<double> *start, list<double> *goal, list<tuple2<double, double> *> *obstacleList, list<double> *randArea, __ss_int goalSampleRate, __ss_int maxIter) {
//    /**
//    Setting Parameter
//    
//    start:Start Position [x,y]
//    goal:Goal Position [x,y]
//    obstacleList:obstacle Positions [[x,y,size],...]
//    randArea:Ramdom Samping Area [min,max]
//    
//    */
//    
//    this->start = (new Node(start->__getfast__(0), start->__getfast__(1), start->__getfast__(2)));
//    this->end = (new Node(goal->__getfast__(0), goal->__getfast__(1), goal->__getfast__(2)));
//    this->minrand = randArea->__getfast__(0);
//    this->maxrand = randArea->__getfast__(1);
//    this->goalSampleRate = goalSampleRate;
//    this->maxIter = maxIter;
//    this->obstacleList = obstacleList;
//    return NULL;
//}
//
//Node *RRT::steer(Node *rnd, __ss_int nind) {
//    Node *nearestNode, *newNode;
//    void *__8, *clen, *mode, *px, *py, *pyaw;
//
//    nearestNode = (this->nodeList)->__getfast__(nind);
//    __8 = reeds_shepp_path_planning->reeds_shepp_path_planning();
//    px = __8->__getfirst__();
//    py = __8->__getsecond__();
//    pyaw = __8->__getfast__(2);
//    mode = __8->__getfast__(3);
//    clen = __8->__getfast__(4);
//    if ((px==NULL)) {
//        return NULL;
//    }
//    newNode = __copy__::deepcopy(nearestNode);
//    newNode->x = px->__getfast__();
//    newNode->y = py->__getfast__();
//    newNode->yaw = pyaw->__getfast__();
//    newNode->path_x = px;
//    newNode->path_y = py;
//    newNode->path_yaw = pyaw;
//    newNode->cost = __add2(newNode->cost, __sum(list_comp_0(clen)));
//    newNode->parent = nind;
//    return newNode;
//}
//
//list<__ss_int> *RRT::find_near_nodes(Node *newNode) {
//    list<__ss_int> *nearinds;
//    __ss_int nnode;
//    double r;
//    list<double> *dlist;
//
//    nnode = len(this->nodeList);
//    r = (50.0*__math__::sqrt((__math__::log(nnode)/nnode)));
//    dlist = list_comp_2(newNode, this);
//    nearinds = list_comp_3(dlist, r);
//    return nearinds;
//}
//
///**
//class Node
//*/
//
//class_ *cl_Node;
//
//void *Node::__init__(double x, double y, double yaw) {
//    
//    this->x = x;
//    this->y = y;
//    this->yaw = yaw;
//    this->path_x = (new list<void *>());
//    this->path_y = (new list<void *>());
//    this->path_yaw = (new list<void *>());
//    this->cost = 0.0;
//    this->parent = NULL;
//    return NULL;
//}
//
//Node *Node::__deepcopy__(dict<void *, pyobj *> *memo) {
//    Node *c = new Node();
//    memo->__setitem__(this, c);
//    c->cost = __deepcopy(cost);
//    c->x = __deepcopy(x);
//    c->yaw = __deepcopy(yaw);
//    c->parent = __deepcopy(parent);
//    c->path_x = __deepcopy(path_x);
//    c->path_y = __deepcopy(path_y);
//    c->y = __deepcopy(y);
//    c->path_yaw = __deepcopy(path_yaw);
//    return c;
//}
//
//void *__ss_main() {
//    list<tuple2<double, double> *> *obstacleList;
//    list<list<double> *>::for_in_loop __74;
//    file *_file;
//    __iter<list<double> *> *__72;
//    list<list<double> *> *__71, *path;
//    list<double> *goal, *ip, *start;
//    RRT *rrt;
//    __ss_int __73;
//
//    print2(NULL,0,1, const_2);
//    _file = open(const_3, const_4);
//    obstacleList = (new list<tuple2<double, double> *>(59,(new tuple2<double, double>(3,2.5,((double)(0)),0.1)),(new tuple2<double, double>(3,2.5,0.4,0.1)),(new tuple2<double, double>(3,2.5,0.6,0.1)),(new tuple2<double, double>(3,2.5,0.8,0.1)),(new tuple2<double, double>(3,2.5,1.0,0.1)),(new tuple2<double, double>(3,2.5,1.4,0.1)),(new tuple2<double, double>(3,2.5,1.6,0.1)),(new tuple2<double, double>(3,2.5,1.8,0.1)),(new tuple2<double, double>(3,2.5,2.0,0.1)),(new tuple2<double, double>(3,2.5,2.2,0.1)),(new tuple2<double, double>(3,2.5,2.4,0.1)),(new tuple2<double, double>(3,2.5,2.8,0.1)),(new tuple2<double, double>(3,2.5,3.0,0.1)),(new tuple2<double, double>(3,2.5,3.4,0.1)),(new tuple2<double, double>(3,2.5,3.6,0.1)),(new tuple2<double, double>(3,2.5,3.8,0.1)),(new tuple2<double, double>(3,2.5,4.0,0.1)),(new tuple2<double, double>(3,2.5,4.2,0.1)),(new tuple2<double, double>(3,2.5,4.48,0.1)),(new tuple2<double, double>(3,2.5,4.6,0.1)),(new tuple2<double, double>(3,2.5,4.8,0.1)),(new tuple2<double, double>(3,2.5,4.9,0.1)),(new tuple2<double, double>(3,2.5,5.0,0.1)),(new tuple2<double, double>(3,((double)(5)),((double)(0)),0.1)),(new tuple2<double, double>(3,((double)(5)),0.4,0.1)),(new tuple2<double, double>(3,((double)(5)),0.6,0.1)),(new tuple2<double, double>(3,((double)(5)),0.8,0.1)),(new tuple2<double, double>(3,((double)(5)),1.0,0.1)),(new tuple2<double, double>(3,((double)(5)),1.4,0.1)),(new tuple2<double, double>(3,((double)(5)),1.6,0.1)),(new tuple2<double, double>(3,((double)(5)),1.8,0.1)),(new tuple2<double, double>(3,((double)(5)),2.0,0.1)),(new tuple2<double, double>(3,((double)(5)),2.2,0.1)),(new tuple2<double, double>(3,((double)(5)),2.4,0.1)),(new tuple2<double, double>(3,((double)(5)),2.8,0.1)),(new tuple2<double, double>(3,((double)(5)),3.0,0.1)),(new tuple2<double, double>(3,((double)(5)),3.4,0.1)),(new tuple2<double, double>(3,((double)(5)),3.6,0.1)),(new tuple2<double, double>(3,((double)(5)),3.8,0.1)),(new tuple2<double, double>(3,((double)(5)),4.0,0.1)),(new tuple2<double, double>(3,((double)(5)),4.2,0.1)),(new tuple2<double, double>(3,((double)(5)),4.48,0.1)),(new tuple2<double, double>(3,((double)(5)),4.6,0.1)),(new tuple2<double, double>(3,((double)(5)),4.8,0.1)),(new tuple2<double, double>(3,((double)(5)),4.9,0.1)),(new tuple2<double, double>(3,((double)(5)),5.0,0.1)),(new tuple2<double, double>(3,2.6,((double)(0)),0.1)),(new tuple2<double, double>(3,2.8,((double)(0)),0.1)),(new tuple2<double, double>(3,3.0,((double)(0)),0.1)),(new tuple2<double, double>(3,3.2,((double)(0)),0.1)),(new tuple2<double, double>(3,3.4,((double)(0)),0.1)),(new tuple2<double, double>(3,3.6,((double)(0)),0.1)),(new tuple2<double, double>(3,3.8,((double)(0)),0.1)),(new tuple2<double, double>(3,4.0,((double)(0)),0.1)),(new tuple2<double, double>(3,4.2,((double)(0)),0.1)),(new tuple2<double, double>(3,4.4,((double)(0)),0.1)),(new tuple2<double, double>(3,4.6,((double)(0)),0.1)),(new tuple2<double, double>(3,4.8,((double)(0)),0.1)),(new tuple2<double, double>(3,5.0,((double)(0)),0.1))));
//    start = (new list<double>(3,(-1.0),7.0,__math__::radians(0.0)));
//    goal = (new list<double>(3,3.85,4.0,__math__::radians(90.0)));
//    rrt = (new RRT(start, goal, obstacleList, (new list<double>(2,(-2.0),20.0)), 90, 100));
//    path = rrt->Planning(show_animation);
//    if (show_animation) {
//        rrt->DrawGraph(NULL);
//
//        FOR_IN(ip,path,71,73,74)
//            print2(NULL,0,2, const_5, ip);
//            _file->write(__add_strs(4, __str(ip->__getfast__(0)), const_6, __str(ip->__getfast__(1)), const_7));
//        END_FOR
//
//        _file->close();
//    }
//    return NULL;
//}
//
//void __init() {
//    __name__ = new str("__main__");
//
//    show_animation = True;
//    STEP_SIZE = 0.1;
//    curvature = 0.3;
//    default_0 = True;
//    default_1 = NULL;
//    cl_RRT = new class_("RRT");
//    cl_Node = new class_("Node");
//    if (__eq(__name__, const_8)) {
//        __ss_main();
//    }
//}
//
//} // module namespace
//
//int main(int, char **) {
//    __shedskin__::__init();
//    __math__::__init();
//    __time__::__init();
//    __random__::__init();
//    __copy__::__init();
//    __shedskin__::__start(__rrt_star_reeds_shepp__::__init);
//}
