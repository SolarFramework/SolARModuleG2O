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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/core/robust_kernel_impl.h>

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

	/// @brief this method is used to set intrinsic parameters and distortion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distortion parameters.
	void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) override;

    /// @brief solve a non-linear problem related to bundle adjustement statement expressed as:
    /// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
    /// @param[in,out] framesToAdjust: contains a set of {2D points, camera extrinsics}.
    /// @param[in,out] mapToAjust: contains a set of of 3D points .
    /// @param[in] K: camera calibration parameters responsible of 3D points generation.
    /// @param[in] D: camera distorsion parameters responsible of 3D points generation
    /// K, D represent the camera intrinsic parameters
    /// @param[in] selectKeyframes : selected views to bundle following a given strategies (ex: poseGraph).
    /// @return the mean re-projection error after {pts3d, intrinsic, extrinsic} correction.
    double solve(	const std::vector<SRef<Keyframe>> & originalKeyframes,
					const std::vector<CloudPoint> & originalCloud,
					const  CamCalibration & originalCalib,
					const CamDistortion & originalDist,
					const std::vector<int> & selectKeyframes,
					std::vector<Transform3Df> & correctedPoses,
					std::vector<CloudPoint>&correctedCloud,
					CamCalibration&correctedCalib,
					CamDistortion &correctedDist) override;

	double solve(	const std::vector<std::vector<Keyline>> & originalKeylines,
					const std::vector<Edge3Df> & lineCloud,
					const std::vector<DescriptorMatch> & matches,
					const std::vector<int> & indices,
					const std::vector<Transform3Df> originalPoses,
					std::vector<Edge3Df> & correctedLineCloud,
					std::vector<Transform3Df> & correctedPoses) override;

private:
	Eigen::Matrix<double, 5, 1> m_camDistortion;
	Eigen::Matrix<double, 3, 3> m_camMatrix;
	Eigen::Matrix<double, 3, 3> m_Kinv;

	int	m_iterations;
	int	m_setVerbose;
	int	m_nbMaxFixedKeyframes;
};

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
