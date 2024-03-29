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
#include "api/storage/ICovisibilityGraphManager.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/ICameraParametersManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/geom/I3DTransform.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {

/**
 * @class SolAROptimizationG2O
 * @brief <B>Bundle adjustment optimization.</B>
 * <TT>UUID: 870d89ba-bb5f-460a-a817-1fcb6473df70</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
 * @SolARComponentInjectable{SolAR::api::storage::ICameraParametersManager}
 * @SolARComponentInjectable{SolAR::api::storage::IKeyframesManager}
 * @SolARComponentInjectable{SolAR::api::storage::ICovisibilityGraphManager}
 * @SolARComponentInjectablesEnd
 *
 * @SolARComponentPropertiesBegin
 * @SolARComponentProperty{ nbIterationsLocal,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 10 }}
 * @SolARComponentProperty{ nbIterationsGlobal,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 10 }}
 * @SolARComponentProperty{ setVerbose,
 *                          (0 = false\, 1 = true),
 *                          @SolARComponentPropertyDescNum{ int, [0\, 1], 0 }}
 * @SolARComponentProperty{ nbMaxFixedKeyframes,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ int, [0..MAX INT], 0 }}
 * @SolARComponentProperty{ errorOutlier,
 *                          ,
 *                          @SolARComponentPropertyDescNum{ float, [0..MAX FLOAT], 3.f }}
 * @SolARComponentProperty{ useSpanningTree,
 *                          (0 = false\, 1 = true),
 *                          @SolARComponentPropertyDescNum{ int, [0\, 1], 0 }}
 * @SolARComponentProperty{ isRobust,
 *                          (0 = false\, 1 = true),
 *                          @SolARComponentPropertyDescNum{ int, [0\, 1], 1 }}
 * @SolARComponentProperty{ fixedMap,
 *                          (0 = false\, 1 = true),
 *                          @SolARComponentPropertyDescNum{ int, [0\, 1], 0 }}
 * @SolARComponentProperty{ fixedKeyframes,
 *                          (0 = false\, 1 = true),
 *                          @SolARComponentPropertyDescNum{ int, [0\, 1], 0 }}
 * @SolARComponentPropertiesEnd
 *
 * 
 */

class SOLARG2O_EXPORT_API SolAROptimizationG2O : public org::bcom::xpcf::ConfigurableBase,
    public api::solver::map::IBundler
{
public:
    SolAROptimizationG2O();
    ~SolAROptimizationG2O() override;

	/// @brief set map reference to optimize
	/// @param[in] map the input map.
	/// @return FrameworkReturnCode::_SUCCESS_ if the map is set, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode setMap(const SRef<datastructure::Map> map) override;

	/// @brief solve a non-linear problem related to bundle adjustement statement expressed as:
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
	/// @param[in] selectKeyframes : selected views to bundle following a given strategies. If it is empty then take all keyframes into account to perform global bundle adjustment.
	/// @return the mean re-projection error after optimization.
	double bundleAdjustment(const std::vector<uint32_t> & selectKeyframes = {}) override;

	/// @brief solve a non-linear problem related to sim3D optimization between two overlaped keyframes of two different maps:
	/// @param[in] keyframe1: first overlapping keyframe from map 1.
	/// @param[in] keyframe2: second overlapping keyframe from map 2.
	/// @param[in] matches: matches between two keyframes.
	/// @param[in] pts3D1: first set of 3D points.
	/// @param[in] pts3D2: second set of 3D points.
	/// @param[in, out] pose: Sim3 matrix pose between map1 and map2
	/// @return the mean re-projection error.
    double optimizeSim3(const SRef<Keyframe>& keyframe1,
						const SRef<Keyframe>& keyframe2,
						const std::vector<DescriptorMatch>& matches,
						const std::vector<Point3Df> & pts3D1,
						const std::vector<Point3Df> & pts3D2,
						datastructure::Transform3Df & pose) override;

	void unloadComponent() override final;

private:
	int							m_iterationsLocal = 10;
	int							m_iterationsGlobal = 10;
	int							m_iterationsSim3   = 20;
	int							m_setVerbose;
	int							m_nbMaxFixedKeyframes;
	float						m_errorOutlier = 3.f;
	float						m_errorSim3 = 50.f;
	int							m_useSpanningTree = 0;
	int							m_isRobust = 1;
	int							m_fixedMap = 0;
	int							m_fixedKeyframes = 0;
	int							m_fixedScale = 0;
    SRef<datastructure::Map>                        m_map;
    SRef<api::geom::I3DTransform>					m_transform3D;
	SRef<api::storage::IPointCloudManager>			m_pointCloudManager;
	SRef<api::storage::IKeyframesManager>			m_keyframesManager;
    SRef<api::storage::ICameraParametersManager>	m_cameraParametersManager;
	SRef<api::storage::ICovisibilityGraphManager>	m_covisibilityGraphManager;
};

}
}
}

#endif // SOLAROPTIMIZATIONG2O_H
