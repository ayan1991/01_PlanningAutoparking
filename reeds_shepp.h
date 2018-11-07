/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Guan-Horng Liu.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

//#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <vector>
#include <limits>

using namespace std;

typedef int (*ReedsSheppPathSamplingCallback)(double q[3], void* user_data);
typedef int (*ReedsSheppPathTypeCallback)(int t, void* user_data);
typedef int(*ReedsSheppPathSamplingVCallback)(double q[3], void* user_data);
class ReedsSheppStateSpace
{
public:

    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };

    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    
    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);
        
        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(double q0[3], double q1[3]);

    void type(double q0[3], double q1[3], ReedsSheppPathTypeCallback cb, void* user_data);
	//
    double sample(double q0[3], double q1[3], double step_size, ReedsSheppPathSamplingCallback cb, void* user_data);
	double sample_with_vec(double q0[3], double q1[3], double step_size, ReedsSheppPathSamplingVCallback cb, void* user_data);
    /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

protected:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);
	void interpolate_with_vec(double q0[3], ReedsSheppPath &path, double seg, double seg_step, double s[4], double& seg_left);
    /** \brief Turning radius */
    double rho_;
};

typedef struct tagPosture {
	double x;
	double y;
	double yaw;
	double acc;
	double speed;
	double cost;
	int parent_index;
	vector<double> path_x;
	vector<double> path_y;
	vector<double> path_yaw;
	vector<double> path_speed;
	double path_length;
} Posture;


typedef struct tagpointNode {
	double x;
	double y;
	double yaw;
	
	
	double cost;
	double parent_index;


} pointNode;
class ReedsSheppPathGenerator
{
public:
	ReedsSheppPathGenerator() {}

	std::vector<Posture>& Generate(const Posture& start_pos, const Posture& end_pos, float step_size, float turning_radius);

	void AddToPath(double d[3]) {
		Posture state;
		state.x = d[0];
		state.y = d[1];
		state.yaw = d[2];
		state.acc = 0.0;
		state.speed = 0.0;
		path_.push_back(state);
	}

	void AddToPathWithVec(double d[4]) {
		Posture tmp_coord;
		tmp_coord.x = d[0];
		tmp_coord.y = d[1];
		tmp_coord.yaw = d[2];
		tmp_coord.speed = d[3];
		tmp_coord.acc = 0.0;
		//tmp_coord.speed = 0.0;
		path_.push_back(tmp_coord);
	}
private:
	std::vector<Posture> path_;
};


#endif