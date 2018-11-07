

///err file
#include <vector>
#include <iostream>
#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <valarray>
#include "math.h"
#include <random>

#define pi 3.1415

using namespace std;

uniform_real_distribution<float> f_random(0.0, 1.0);
bool show_animation = true;

namespace{

	struct  Path{
		
			//Path();
			vector<float> lengths;
			vector<string> ctypes;
			float L = 0.0;
			vector<float> x, y, yaw, directions;
	};
	
	//vector<Path> paths;
	void plot_arrow(float x, float y, float yaw, float length = 1.0, float width = 0.5, char*  fc = "r", char* ec = "k") {

		if (x != 0) {
			std::cout << "x" << endl;
		}
	}
	float mod(float n, float m)
	{

		/*while (n < 0)
			n += m;*/
		float result = int(n) % int(m);

		return result;
	}
	float mod2pi(float x) {
		float v = fmod(x, 2.0 * pi);
		if (v < -pi){
			v += 2.0 * pi;
		}
		else if (v > pi){
			v -= 2.0 * pi;
		}
		return v;
	}
	//inline double mod2pi(double x)
	//{
	//	double v = fmod(x, 2*pi);
	//	if (v < -pi)
	//		v += 2*pi;
	//	else
	//		if (v > pi)
	//			v -= 2*pi;
	//	return v;
	//}
	//inline void polar(double x, double y, double &r, double &theta)
	//{
	//	r = sqrt(x*x + y*y);
	//	theta = atan2(y, x);
	//}
	float temp_f[2] = { 0 };
	float* polar_cc(float x, float y) {

		float r = std::sqrt(x *x + y *y);
		float theta = std::atan2(y, x);
		temp_f[0] = r;
		temp_f[1] = theta;
		return temp_f;
	}
	void SLS(float x, float y, float phi,float point_state[4]){
		phi = mod2pi(phi);

		if (y > 0.0 && phi > 0.0 && phi < pi * 0.99){
			float xd = -y / tan(phi) + x;
			float t = xd - tan(phi / 2.0);
			float u = phi;
			float v = sqrt((x - xd)*(x - xd) + y*y) - tan(phi / 2.0);
			point_state[0] = true;
			point_state[1] = t;
			point_state[2] = u;
			point_state[3] = v;

			return ;
		}
		else if (y < 0.0 && phi > 0.0 && phi < pi * 0.99){
			float xd = -y / tan(phi) + x;
			float t = xd - tan(phi / 2.0);
			float u = phi;
			float v = -sqrt((x - xd) *(x - xd) + y *y) - tan(phi / 2.0);
			point_state[0] = true;
			point_state[1] = t;
			point_state[2] = u;
			point_state[3] = v;

			return ;
		}
		point_state[0] = false;
		point_state[1] = 0.0f;
		point_state[2] = 0.0f;
		point_state[3] = 0.0f;

		return ;
	}

	
	/*std::valarray<float> myvalarray;*/

	void set_path( vector<float> lengths, vector<string> ctypes, vector<Path> &paths) {
		float sum_result = 0;
		Path path;
		path.ctypes = ctypes;
		path.lengths = lengths;
		
		float paths_len = 0.0;
		float path_len  = 0.0;
		// check same path exist
		for (int i = 0; i < paths.size(); i++) {
			bool typeissame = (paths[i].ctypes == path.ctypes);
			if (typeissame){
				for (int j = 0; j < paths[i].lengths.size(); j++){
					path_len += lengths[j];
					paths_len += paths[i].lengths[j];
				}
				if (paths_len - path_len <= 0.1){
					//paths[i].x.push_back(lengths[0]);
					//paths[i].y.push_back(lengths[1]);
					//paths[i].yaw.push_back(lengths[2]);
					paths.push_back(path);
					return ;// // not insert path;
				}
			}
		}
		for (int i = 0; i < lengths.size(); i++){
			sum_result = sum_result + std::abs(lengths[i]);
		}
		path.L = sum_result;

		// Base.Test.@test path.L >= 0.01
		if (path.L >= 0.1){
			paths.push_back(path);
		}
		return ;
	}

	void SCS(float x, float y, float phi, vector<Path> &paths) {
		float action[4] = { 0 };

		SLS(x, y, phi, action);
		
		std::cout << "action[0] " << action[0] << std::endl;
		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "S", "L", "S" }, paths);
		}
		SLS(x, -y, -phi, action);
		
		std::cout << "action[0] " << action[0] << std::endl;
		if (action[0]){
			set_path( { action[1], action[2], action[3] }, { "S", "R", "S" }, paths);
		}

		return ;
	}
	
	void LSL(float x, float y, float phi, float  point_state[4]) {
		
		//float point_state[4] = { 0 };
		
		(polar_cc(x - std::sin(phi), y - 1.0 + std::cos(phi)));
		float u = temp_f[0];
		float t = temp_f[1];
		float v = 0.0;

		if (t >= 0.0){
			v = mod2pi(phi - t);
			if (v >= 0.0){
				point_state[0] = true;
				point_state[1] = t;
				point_state[2] = u;
				point_state[3] = v;
				return ;
			}
		}
		point_state[0] = false;
		point_state[1] = 0;
		point_state[2] = 0;
		point_state[3] = 0;
		return ;
	}

	void LRL(float x, float y, float phi, float point_state[4]) {
		//float point_state[4] = { 0 };
		temp_f[2] = *(polar_cc(x - std::sin(phi), (y - 1.0 + std::cos(phi))));


		//float u, t, v;
		float u1 = temp_f[0];
		float t1 = temp_f[1];
		float u, v, t;
		if (u1 <= 4.0){
			u = -2.0 * std::asin(0.25 * u1);
			t = mod2pi(t1 + 0.5 * u + pi);
			v = mod2pi(phi - t + u);

			if (t >= 0.0 && u <= 0.0){
				point_state[0] = true;
				point_state[1] = t;
				point_state[2] = u;
				point_state[3] = v;
				return ;
			}
		}
		point_state[0] = false;
		point_state[1] = 0;
		point_state[2] = 0;
		point_state[3] = 0;
		return ;
	}

	void CCC(float x, float  y, float  phi, vector<Path>& paths) {
		float action[4] = { 0 };
		LRL(x, y, phi, action);

		if (action[0]) {
			set_path({ action[1], action[2], action[3] }, { "L", "R", "L" }, paths);
		}

		LRL(-x, y, -phi, action);
		
		if (action[0]) {
			set_path( { -action[1], -action[2], -action[3] }, { "L", "R", "L" }, paths);
		}

		LRL(x, -y, -phi, action);
		
		if (action[0]) {
			set_path( { action[1], action[2], action[3] }, { "R", "L", "R" }, paths);
		}
		LRL(-x, -y, phi, action);
		
		if (action[0]) {
			set_path( { -action[1], -action[2], -action[3] }, { "R", "L", "R" }, paths);
		}

		// backwards
		float xb = x * std::cos(phi) + y * std::sin(phi);
		float yb = x * std::sin(phi) - y * std::cos(phi);
		// println(xb, ",", yb, ",", x, ",", y)

		LRL(xb, yb, phi, action);
	

		if (action[0]) {
			set_path({ action[1], action[2], action[3] }, { "L", "R", "L" }, paths);
		}

		LRL(-xb, yb, -phi, action);
	

		if (action[0]){
			set_path( { -action[1], -action[2], -action[3] }, { "L", "R", "L" }, paths);
		}
		LRL(xb, -yb, -phi, action);
	
		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "R", "L", "R" }, paths);
		}
		LRL(-xb, -yb, phi, action);
		
		if (action[0]){
			set_path({ -action[1], -action[2], -action[3] }, { "R", "L", "R" }, paths);
		}
		return ;

	}
	void LSR(float x, float y, float phi, float point_state[4]) {
		//float point_state[4] = { 0 };
		float utl[2] = { 0 };
	
		float  theta;
		polar_cc(x + std::sin(phi), y - 1.0 - std::cos(phi));
	
		utl[0] = temp_f[0];
		utl[1] = temp_f[1];

		utl[0] = utl[0] * utl[0];
		float t, u, v;
		if (utl[0] >= 4.0){
			u = std::sqrt(utl[0] - 4.0);
			theta = std::atan2(2.0, u);
			t = mod2pi(utl[1] + theta);
			v = mod2pi(t - phi);

			if (t >= 0.0 && v >= 0.0){
				point_state[0] = true;
				point_state[1] = t;
				point_state[2] = u;
				point_state[3] = v;
				return ;
			}
		}
		point_state[0] = false;
		point_state[1] = 0;
		point_state[2] = 0;
		point_state[3] = 0;

		return ;
	}

	void CSC(float x, float  y, float phi, vector<Path> &paths) {
		float action[4] = { 0 };
	
		LSL(x, y, phi, action);
		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "L", "S", "L" }, paths);
		}
		(LSL(-x, y, -phi, action));

		if (action[0]){
			set_path({ -action[1], -action[2], -action[3] }, { "L", "S", "L" }, paths);
		}
		(LSL(x, -y, -phi, action));
		
		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "R", "S", "R" }, paths);
		}
		(LSL(-x, -y, phi, action));

		if (action[0]){
			set_path({ -action[1], -action[2], -action[3] }, { "R", "S", "R" }, paths);
		}
		(LSR(x, y, phi, action));

		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "L", "S", "R" }, paths);
		}
	
		(LSR(-x, y, -phi, action));

		if (action[0]){
			set_path({ -action[1], -action[2], -action[3] }, { "L", "S", "R" }, paths);
		}
		
		(LSR(x, -y, -phi, action));

		if (action[0]){
			set_path({ action[1], action[2], action[3] }, { "R", "S", "L" }, paths);
		}
		
		(LSR(-x, -y, phi, action));

		if (action[0]){
			set_path( { -action[1], -action[2], -action[3] }, { "R", "S", "L" },paths);
		}
		return ;
	}

	//vector<Path> paths;
	void generate_path(float(&q0)[3], float(&q1)[3], float maxc, vector<Path> &paths) {
		float dx = q1[0] - q0[0];
		float dy = q1[1] - q0[1];
		float dth = q1[2] - q0[2];

		float c = std::cos(q0[2]);
		float s = std::sin(q0[2]);
		float x = (c * dx + s * dy) * maxc;
		float y = (-s * dx + c * dy) * maxc;

		SCS(x, y, dth, paths);
		CSC(x, y, dth, paths);
		CCC(x, y, dth, paths);

		return ;
	}
	//todo
	void interpolate(int ind, float l, string m, float maxc, float ox, float oy, float oyaw, vector<float> &px, vector< float> &py, vector< float> &pyaw, vector< float> &directions) {
		float ldx, ldy, gdx, gdy;

		if (m == "S") {
			px[ind] = ox + l / maxc * std::cos(oyaw);
			py[ind] = oy + l / maxc * std::sin(oyaw);
			pyaw[ind] = oyaw;
		}
		else{  // curve
			ldx = std::sin(l) / maxc;
			if (m == "L"){  // left turn
				float ldy = (1.0 - std::cos(l)) / maxc;
			}
			else if (m == "R"){  // right turn
				ldy = (1.0 - std::cos(l)) / -maxc;
				gdx = std::cos(-oyaw) * ldx + std::sin(-oyaw) * ldy;
				gdy = -std::sin(-oyaw) * ldx + std::cos(-oyaw) * ldy;
				px[ind] = ox + gdx;
				py[ind] = oy + gdy;
			}
		}
		if (m == "L"){ // left turn
			pyaw[ind] = oyaw + l;
		}
		else if (m == "R"){  // right turn
			pyaw[ind] = oyaw - l;
		}
		if (l > 0.0){
			directions[ind] = 1;
		}
		else {
			directions[ind] = -1;
		}
		return ;
	}
	//vector<float> px, py, pyaw, directions;
	void generate_local_course(float L, vector<float> lengths, vector<string> mode, float maxc, int step_size, vector<float> &px, vector<float> &py, vector<float> & pyaw, vector<float> &directions) {
		float npoint = std::trunc(L / step_size) + lengths.size() + 4;

		for (float f_i = 0.0; f_i < npoint; f_i++){
			px.push_back(f_i);
			py.push_back(f_i);
			pyaw.push_back(f_i);
			directions.push_back(f_i);
		}
		int ind = 1;
		int d, pd, ll;
		if (lengths[0] > 0.0){
			directions[0] = 1;
		}
		else{
			directions[0] = -1;
		}
		if (lengths[0] > 0.0){
			d = step_size;
		}
		else {
			d = -step_size;
		}
		pd = d;
		ll = 0.0;
		float origin[3] = { 0 };
		//for (m, l, i) in zip(mode, lengths, range(len(mode))) {
		for (int i = 0; i < lengths.size(); i++) {
			if (lengths[i] > 0.0){
				d = step_size;
			}
			else {
				d = -step_size;
			}
			// set origin state
			origin[0] = px[ind];
			origin[1] = py[ind];
			origin[2] = pyaw[ind];

			//};

			ind -= 1;
			if (i >= 1 && (lengths[i - 1] * lengths[i]) > 0){
				pd = -d - ll;
			}
			else{
				pd = d - ll;
			}
			while (abs(pd) <= abs(lengths[i])) {
				ind += 1;
				interpolate(
					ind, pd, mode[i], maxc, origin[0], origin[1], origin[2], px, py, pyaw, directions);
				pd += d;
			}
			ll = lengths[i] - pd - d; // calc remain length

			ind += 1;
			interpolate(
				ind, lengths[i], mode[i], maxc, origin[0], origin[1], origin[2], px, py, pyaw, directions);
			 
		}

		////remove unused data
		int last_index = px.size() - 1;
		//vector<int>::iterator   iter = px.end - 1;
		while (px[last_index] == 0.0){
			px.pop_back();
			py.pop_back();
			pyaw.pop_back();
			directions.pop_back();
		}
		return ;
	}


	float pi_2_pi(float angle) {
		return (angle + pi) / (2 * pi) - pi;
	}
	
	vector<float> px, py, pyaw, directions;
	//todo
	void calc_paths(float sx, float sy, float syaw, float gx, float gy, float gyaw, float maxc, int step_size, vector<Path> &paths) {
		float q0[3] = { 0 };
		float q1[3] = { 0 };
		q0[0] = sx;
		q0[1] = sy;
		q0[2] = syaw;
		q1[0] = gx;
		q1[1] = gy;
		q1[2] = gyaw;

		//vector<float>  state = { 0, 0, 0, 0 };// x, y, yaw, directions
		generate_path(q0, q1, maxc,paths);
		std::cout << "paths" << std::endl;
		//vector<float> x, y;
		for (int i = 0; i < paths.size(); i++){
			 generate_local_course(
				 paths[i].L, paths[i].lengths, paths[i].ctypes, maxc, step_size * maxc, px, py, pyaw, directions);

			// convert global coordinate
			for (int j = 0; j < px.size(); j++){
				//for (int k = j; k < y.size();k++){
				float p_x = std::cos(-q0[2]) * px[j] + std::sin(-q0[2])* py[j] + q0[0];
				cout << "p_x ="<< p_x << endl;
				(paths[i].x).push_back(std::cos(-q0[2]) * px[j] + std::sin(-q0[2])* py[j] + q0[0]);
				(paths[i].y).push_back(-std::sin(-q0[2]) * px[j] + std::cos(-q0[2])* py[j] + q0[1]);
				(paths[i].yaw).push_back(pi_2_pi(pyaw[j] + q0[2]));//yaw
				(paths[i].directions).push_back(directions[j]);
				(paths[i].lengths).push_back(j / maxc);
				paths[i].L = paths[i].L / maxc;
				//}
			}
			//std::cout << paths.x << std::endl;
		}
		return ;
	}
	
	void reeds_shepp_path_planning(float sx, float sy, float syaw,
		float gx, float gy, float gyaw, float maxc, int step_size, vector<Path> &paths,Path & bpath) {
		
		calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size, paths);

		if (paths.size() == 0){
			std::cout << "No path" << std::endl;
			std::cout << "sx, sy, syaw, gx, gy, gyaw" << sx << sy <<
				syaw << gx << gy << gyaw << std::endl;
			bpath.x.push_back(0);
			bpath.y.push_back(0);
			bpath.yaw.push_back(0);
			bpath.x.push_back(0);
			return ;
		}
		float minL = 10000.0;
		int best_path_index = 0;//-1
		for (int i = 0; i < paths.size(); i++) {
			if (paths[i].L <= minL){
				minL = paths[i].L;
				best_path_index = i;
			}
			bpath = paths[best_path_index];
		}
		return ;
	}

	/*float radians(float angle){
		return angle / 57.297f;
	}*/

	//void test() {
	//
	//	int NTEST = 5;
	//	float start_x, start_y,start_yaw,end_x,end_y,end_yaw;
	//	float curvature, step_size;
	//	float f_random;
	//	for (int i = 0; i < NTEST;i++){
	//		start_x = (f_random - 0.5) * 10.0;  // [m]
	//		start_y = (f_random - 0.5) * 10.0;  // [m]
	//		start_yaw = radians((f_random - 0.5) * 180.0);  // [rad]
	//
	//		end_x = (f_random - 0.5) * 10.0;  // [m]
	//		end_y = (f_random - 0.5) * 10.0; // [m]
	//		end_yaw = radians((f_random - 0.5) * 180.0);  // [rad]
	//
	//		curvature = 1.0 / (f_random * 20.0);
	//		step_size = 0.1;
	//
	//		//px, py, pyaw, mode, clen = reeds_shepp_path_planning(
	//			//start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size);
	//		path = reeds_shepp_path_planning(
	//			start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size);
	//	}
	//	if (show_animation){
	//		//plt.cla()
	//		//plt.plot(px, py, label = "final course " + str(mode))
	//
	//		//  plotting
	//		plot_arrow(start_x, start_y, start_yaw);
	//		plot_arrow(end_x, end_y, end_yaw);
	//
	//		//plt.legend();
	//		//plt.grid(True);
	//		//plt.axis("equal");
	//		//plt.xlim(-10, 10);
	//		//plt.ylim(-10, 10);
	//		//plt.pause(1.0);
	//
	//		//  plt.show()
	//	}
	//	std::cout<<("Test done");
	//}



	//
	//void main_cc_rs_rrt() {
	//	std::cout << "RRT_start_cc path planner sample start!!" << std::endl;
	//
	//	float start_x = 100.0;  // [m]
	//	float start_y = 400.0;  // [m]
	//	float start_yaw = radians(-20.0);  // [rad]
	//
	//	float end_x = 500.0;  // [m]
	//	float end_y = 900.0;  // [m]
	//	float end_yaw = radians(25.0);  // [rad]
	//
	//	float curvature = 10.0;
	//	float step_size = 20;
	//
	//	//px, py, pyaw, mode, clen = reeds_shepp_path_planning(
	//	//	start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size);
	//
	//	path = reeds_shepp_path_planning(
	//		start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size);
	//	std::cout << " [ RRT_start_cc : ]!!" << path.x[1] << std::endl;
	//
	//	if( show_animation){
	//			//plt.cla()
	//			//plt.plot(px, py, label = "final course " + str(mode))
	//
	//			// plotting
	//			/*plot_arrow(start_x, start_y, start_yaw)
	//			plot_arrow(end_x, end_y, end_yaw)
	//
	//			plt.legend()
	//			plt.grid(True)
	//			plt.axis("equal")
	//			plt.show()*/
	//	}
	//		/*if not px :
	//			assert False, "No path"*/
	//}


}//namespace