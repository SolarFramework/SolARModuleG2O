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
#ifndef SOLARPOSEESTIMATIONPNPL_H
#define SOLARPOSEESTIMATIONPNPL_H

#include "SolARG2OAPI.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3DPointLine.h"
#include "datastructure/Keyline.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {
/**
 * @class SolARPoseEstimationPnPL
 * @brief <B>Pose Estimation using Perspective-n-PointLine.</B>
 * <TT>UUID: b76fe9a6-af2b-4778-a121-b54a41565a70</TT>
 */

class SOLARG2O_EXPORT_API SolARPoseEstimationPnPL : public org::bcom::xpcf::ConfigurableBase,
	public api::solver::pose::I3DTransformFinderFrom2D3DPointLine
{
public:
	SolARPoseEstimationPnPL();
	~SolARPoseEstimationPnPL();

	org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
	void unloadComponent() override final;

	/// @brief this method is used to set intrinsic parameters and distortion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distortion parameters.
	void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) override;

	/// @brief Estimates camera pose from a set of 2D image points and 2D lines and their corresponding 3D world points and lines.
	/// @param[in] imagePoints: set of 2D points.
	/// @param[in] worldPoints: set of 3D points.
	/// @param[in] imageLines: set of 2D lines.
	/// @param[in] worldLines: set of 3D lines.
	/// @param[out] pose: camera pose (pose of the camera defined in world coordinate system) expressed as a <Transform3Df>.
	/// @param[in] initialPose: (optional) a <Transform3Df> to initialize the pose (reducing convergence time and improving success rate).
	FrameworkReturnCode estimate(	const std::vector<Point2Df> & imagePoints,
									const std::vector<Point3Df> & worldPoints,
									const std::vector<Edge2Df> & imageLines,
									const std::vector<Edge3Df> & worldLines,
									Transform3Df & pose,
									const Transform3Df initialPose = Transform3Df::Identity()) override;

	/// @brief Estimates camera pose from a set of 2D image points and 2D lines and their corresponding 3D world points and lines,
	/// and performing RANSAC estimation iteratively to deduce inliers.
	/// @param[in] imagePoints: set of 2D points.
	/// @param[in] worldPoints: set of 3D points.
	/// @param[in] imageLines: set of 2D lines.
	/// @param[in] worldLines: set of 3D lines.
	/// @param[out] imagePoints_inliers: set of 2D points suspected as inliers by RANSAC.
	/// @param[out] worldPoints_inliers: set of 3D points suspected as inliers by RANSAC.
	/// @param[out] imageLines_inliers: set of 2D lines suspected as inliers by RANSAC.
	/// @param[out] worldLines_inliers: set of 3D lines suspected as inliers by RANSAC.
	/// @param[out] pointInliers: boolean vector to store whether a point is considered as an inlier or as an outlier.
	/// @param[out] lineInliers: boolean vector to store whether a line is considered as an inlier or as an outlier.
	/// @param[out] pose: camera pose (pose of the camera defined in world coordinate system) expressed as a <Transform3Df>.
	/// @param[in] initialPose: (optional) a <Transform3Df> to initialize the pose (reducing convergence time and improving success rate).
	FrameworkReturnCode SolARPoseEstimationPnPL::estimateRansac(const std::vector<Point2Df> & imagePoints,
																const std::vector<Point3Df >& worldPoints,
																const std::vector<Edge2Df> & imageLines,
																const std::vector<Edge3Df> & worldLines,
																std::vector<Point2Df>& imagePoints_inliers,
																std::vector<Point3Df>& worldPoints_inliers,
																std::vector<Edge2Df> & imageLines_inliers,
																std::vector<Edge3Df> & worldLines_inliers,
																std::vector<bool> & pointInliers,
																std::vector<bool> & lineInliers,
																Transform3Df & pose,
																const Transform3Df initialPose = Transform3Df::Identity()) override;

private:
	g2o::SE3Quat toSE3Quat(const Transform3Df & pose);
	Transform3Df toSolarPose(const g2o::SE3Quat & SE3);

	float getPointReprojError(const Point2Df pt2D, const Point3Df pt3D, const Eigen::Matrix3f R, const Eigen::Vector3f t);
	float getLineReprojError(const Edge2Df ln2D, const Edge3Df ln3D, const Eigen::Matrix3f R, const Eigen::Vector3f t);

	Eigen::Matrix<double, 5, 1> m_camDistortion;
	Eigen::Matrix3d m_camMatrix;

	int m_iterations = 10;
	int m_setVerbose = 0;
};
}
}
}

#endif // SOLARPOSEESTIMATIONPNPL_H