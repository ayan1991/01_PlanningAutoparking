#ifndef __IFLY_APA_CONTROL_PATH_MANGERRRT_H__
#define __IFLY_APA_CONTROL_PATH_MANGERRRT_H__

#include "modules/common/type.h"
#include "modules/common/macro.h"
#include "modules/common/log.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/common/reeds_shepp_iapa.h"
#include <vector>

using namespace iapa::common;
using namespace iapa::planning;
#define VERTICAL     1
#define PARALLEL     2
#define INCLINE      3

#define ParkingMode  PARALLEL

#define CarLength   420
#define CarWidth    190

#define MapHight    1500
#define MapWidth    2000

#define MAP_SCALE    0.5//1

#define pi 3.1415926
#define random(a,b) (rand()%(b-a+1)+a)



const float  step_size = 30;
const float  rho = 450; // turning radius

namespace iapa {
    namespace planning {

        /**
        * @class PathManager
        *
        * @brief
        */
        class PathManagerRRT
        {
        public:
            PathManagerRRT();
            ~PathManagerRRT() {}

            void Clear();

            int Size() { return path_list_.size(); }
			void InitObs(iapa::common::PostureRRT &start, iapa::common::PostureRRT &end, vector<iapa::common::PostureRRT>& obstacleList);
            //void Generate(const iapa::common::Posture& start_pos, const iapa::common::Posture& end_pos, const PathConf& path_conf);
			void GetPlanningRes(iapa::common::PostureRRT& start_pos, iapa::common::PostureRRT& end_pos, const PathConf& path_conf);
        public:
            std::vector<iapa::common::PostureRRT>  path_list_;
			 
        private:
            ReedsSheppPathGeneratorRRT             path_generator_;
            iapa::planning::PathConf            path_conf_;
        };
    }
}

#endif /* __IFLY_APA_CONTROL_PATH_MANGERRRT_H__ */
