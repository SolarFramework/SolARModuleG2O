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
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
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
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @param[in] selectKeyframes : selected views to bundle following a given strategies. If it is empty then take all keyframes into account to perform global bundle adjustment.
	/// @param[in] useSpanningTree: in the case of the global bundle adjustment, if it true, the optimization is based on a maximal spanning tree.
	/// @return the mean re-projection error after optimization.
	double bundleAdjustment(CamCalibration & K, CamDistortion & D, const std::vector<uint32_t> & selectKeyframes, const bool & useSpanningTree = false) override;

private:
	/// @brief solve a non-linear problem related to local bundle adjustement statement expressed as:
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @param[in] selectKeyframes : selected views to bundle following a given strategies (ex: poseGraph).
	/// @return the mean re-projection error after {pts3d, intrinsic, extrinsic} correction.
	double localBundleAdjustment(CamCalibration & K, CamDistortion & D, const std::vector<uint32_t> & selectKeyframes);

	/// @brief solve a non-linear problem related to global bundle adjustement statement expressed as:
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @return the mean re-projection error after {pts3d, intrinsic, extrinsic} correction.
	double globalBundleAdjustment(CamCalibration & K, CamDistortion & D);

	/// @brief Optimize global bundle adjustment based on a maximal spanning tree
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @return the mean re-projection error.
	double optimizeSpanningTree(CamCalibration & K, CamDistortion & D);

private:
	int							m_iterations;
	int							m_setVerbose;
	int							m_nbMaxFixedKeyframes;
	SRef<IPointCloudManager>	m_pointCloudManager;
	SRef<IKeyframesManager>		m_keyframesManager;
	SRef<ICovisibilityGraph>	m_covisibilityGraph;
};

}
}
}

#endif // SOLAROPTIMIZATIONG2O_H
