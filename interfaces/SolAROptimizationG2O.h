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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/core/robust_kernel_impl.h>

namespace SolAR {
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
    FrameworkReturnCode setMapper(const SRef<api::solver::map::IMapper> map) override;

	/// @brief solve a non-linear problem related to bundle adjustement statement expressed as:
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
	/// @param[in, out] K: camera calibration parameters responsible of 3D points generation.
	/// @param[in, out] D: camera distorsion parameters responsible of 3D points generation
	/// @param[in] selectKeyframes : selected views to bundle following a given strategies. If it is empty then take all keyframes into account to perform global bundle adjustment.
	/// @return the mean re-projection error after optimization.
	double bundleAdjustment(datastructure::CamCalibration & K, datastructure::CamDistortion & D, const std::vector<uint32_t> & selectKeyframes = {}) override;

	/// @brief Apply bundle adjustment on a set of 3D lines between multiple frames
	/// @param[in] originalFrames: set of frames with extracted 2D lines and estimated camera poses.
	/// @param[in] frameTriangulatedLines: set of triangulated <CloudLine> for each frame pair.
	/// @param[out] correctedLineCloud: corrected 3D line cloud.
	/// @param[out] correctedPoses: corrected camera poses.
	/// @return the mean re-projection error after 3D lines and poses correction.
	double solve(	const std::vector<SRef<datastructure::Frame>> & originalFrames,
					const std::vector<std::vector<SRef<datastructure::CloudLine>>> & frameTriangulatedLines,
					std::vector<SRef<datastructure::CloudLine>> & correctedLineCloud,
					std::vector<datastructure::Transform3Df> & correctedPoses) override;

	void setCameraParameters(const datastructure::CamCalibration & intrinsicParams, const datastructure::CamDistortion & distortionParams);

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
	SRef<api::storage::IPointCloudManager>	m_pointCloudManager;
	SRef<api::storage::IKeyframesManager>	m_keyframesManager;
	SRef<api::storage::ICovisibilityGraph>	m_covisibilityGraph;
	
	Eigen::Matrix<double, 5, 1> m_camDistortion;
	Eigen::Matrix<double, 3, 3> m_camMatrix;
	Eigen::Matrix<double, 3, 3> m_Kinv;
};

// TODO(mpapin): move to separate class ?
class EdgeLineProjectXYZ : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
	EdgeLineProjectXYZ() {}

	virtual void computeError() {
		// get pose
		const g2o::VertexSE3Expmap* v1 = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
		// get point
		const g2o::VertexSBAPointXYZ* v2 = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
		// get line function
		Eigen::Vector3d obs = _measurement;
		// project point in image
		Eigen::Vector3d pt3D_proj = v1->estimate().map(v2->estimate());
		pt3D_proj[0] = pt3D_proj[0] / pt3D_proj[2];
		pt3D_proj[1] = pt3D_proj[1] / pt3D_proj[2];
		Eigen::Vector2d pt2D;
		pt2D[0] = pt3D_proj[0] * fx + cx;
		pt2D[1] = pt3D_proj[1] * fy + cy;
		// calculate error
		_error(0) = std::abs(obs(0) * pt2D(0) + obs(1)*pt2D(1) + obs(2)) / std::sqrt(obs(0) * obs(0) + obs(1) * obs(1));
		_error(1) = 0.0;
		_error(2) = 0.0;
		//std::cout << "Error line: " << _error << std::endl;
	}

	virtual bool read(std::istream& is) {
		for (int i = 0; i < 3; i++) {
			is >> _measurement[i];
		}

		for (int i = 0; i < 3; ++i) {
			for (int j = i; j < 3; ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
		}
		return true;
	}

	virtual bool write(std::ostream& os) const {
		for (int i = 0; i < 3; i++) {
			os << measurement()[i] << " ";
		}

		for (int i = 0; i < 3; ++i) {
			for (int j = i; j < 3; ++j) {
				os << " " << information()(i, j);
			}
		}
		return os.good();
	}

	double fx, fy, cx, cy;
};

}
}
}

#endif // SOLAROPTIMIZATIONG2O_H
