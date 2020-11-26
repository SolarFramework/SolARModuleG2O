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
    ~SolAROptimizationG2O() override;

	/// @brief set mapper reference to optimize
	/// @param[in] map: the input map.
	/// @return FrameworkReturnCode::_SUCCESS_ if the map is set, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode setMapper(const SRef<api::solver::map::IMapper> &map) override;    

	/// @brief solve a non-linear problem related to bundle adjustement statement expressed as:
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @param[in] selectKeyframes : selected views to bundle following a given strategies. If it is empty then take all keyframes into account to perform global bundle adjustment.
	/// @return the mean re-projection error after optimization.
	double bundleAdjustment(CamCalibration & K, CamDistortion & D, const std::vector<uint32_t> & selectKeyframes = {}) override;

	/// @brief solve a non-linear problem related to sim3D optimization between two overlaped keyframes of two different maps:
	/// @param[in] K1: camera calibration parameters responsible of 3D points generation from map 1.
	/// @param[in] K2: camera calibration parameters responsible of 3D points generation from map 2.
	/// @param[in] keyframe1: first overlapping keyframe from map 1.
	/// @param[in] keyframe2: second overlapping keyframe from map 2.
	/// @param[in] matches: matches between two keyframes.
	/// @param[in] pts3D1: first set of 3D points.
	/// @param[in] pts3D2: second set of 3D points.
	/// @param[in, out] pose: Sim3 matrix pose between map1 and map2
	/// @return the mean re-projection error.
	double optimizeSim3(CamCalibration& K1,
						CamCalibration& K2,
						const SRef<Keyframe>& keyframe1,
						const SRef<Keyframe>& keyframe2,
						const std::vector<DescriptorMatch>& matches,
						const std::vector<Point3Df> & pts3D1,
						const std::vector<Point3Df> & pts3D2,
						Transform3Df & pose) override;

	void unloadComponent() override final;

private:
	int							m_iterationsLocal = 10;
	int							m_iterationsGlobal = 10;
	int							m_setVerbose;
	int							m_nbMaxFixedKeyframes;
	float						m_errorOutlier = 3.f;
	int							m_useSpanningTree = 0;
	int							m_isRobust = 1;
	int							m_fixedMap = 0;
	int							m_fixedKeyframes = 0;
	int							m_fixedScale = 0;
	SRef<IPointCloudManager>	m_pointCloudManager;
	SRef<IKeyframesManager>		m_keyframesManager;
	SRef<ICovisibilityGraph>	m_covisibilityGraph;
};

}
}
}

#endif // SOLAROPTIMIZATIONG2O_H
