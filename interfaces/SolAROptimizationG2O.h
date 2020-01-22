/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef SOLAROPTIMIZATIONG2O_H
#define SOLAROPTIMIZATIONG2O_H

#include "SolARG2OAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/solver/map/IBundler.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {

/**
 * @class SolAROptimizationG2O
 * @brief <B>Bundle adjustment optimization.</B>
 * <TT>UUID: 870d89ba-bb5f-460a-a817-1fcb6473df70</TT>
 *
 */

class SOLARG2O_EXPORT_API SolAROptimizationG2O : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::map::IBundler
{
public:
    SolAROptimizationG2O();
    ~SolAROptimizationG2O();

    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
    void unloadComponent () override final;

    /// @brief solve a non-linear problem related to bundle adjustement statement expressed as:
    /// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
    /// @param[in,out] framesToAdjust: contains a set of {2D points, camera extrinsics}.
    /// @param[in,out] mapToAjust: contains a set of of 3D points .
    /// @param[in] K: camera calibration parameters responsible of 3D points generation.
    /// @param[in] D: camera distorsion parameters responsible of 3D points generation
    /// K, D represent the camera intrinsic parameters
    /// @param[in] selectKeyframes : selected views to bundle following a given strategies (ex: poseGraph).
    /// @return the mean re-projection error after {pts3d, intrinsic, extrinsic} correction.
    double solve(const std::vector<SRef<Keyframe>> & originalKeyframes,
				const std::vector<CloudPoint> & originalCloud,
				const  CamCalibration & originalCalib,
				const CamDistortion & originalDist,
				const std::vector<int> & selectKeyframes,
				std::vector<Transform3Df> & correctedPoses,
				std::vector<CloudPoint>&correctedCloud,
				CamCalibration&correctedCalib,
				CamDistortion &correctedDist) override;

private:
	int			m_iterations;
	int			m_setVerbose;
	int			m_nbMaxFixedKeyframes;
};

}
}
}

#endif // SOLAROPTIMIZATIONG2O_H
