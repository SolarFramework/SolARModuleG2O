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

#include "SolARPoseEstimationPnPL.h"
#include <core/Log.h>
#include <random>
#include <numeric>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::G2O::SolARPoseEstimationPnPL)

// From https://github.com/symao/PnPL/blob/master/src/pnpl.cpp
namespace g2o {
	typedef Eigen::Matrix<double, 2, 1, Eigen::ColMajor>   Vector2D;
	typedef Eigen::Matrix<double, 3, 1, Eigen::ColMajor>   Vector3D;
	typedef Eigen::Matrix<double, 4, 1, Eigen::ColMajor>   Vector4D;
	typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor>   Vector6D;
	class VertexSBALine : public BaseVertex<6, Vector6D>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			VertexSBALine();
		virtual bool read(std::istream& is);
		virtual bool write(std::ostream& os) const;

		virtual void setToOriginImpl() {
			_estimate.fill(0.);
		}

		virtual void oplusImpl(const double* update)
		{
			Eigen::Map<const Vector6D> v(update);
			_estimate += v;
		}
	};

	VertexSBALine::VertexSBALine() : BaseVertex<6, Vector6D>() { }

	bool VertexSBALine::read(std::istream& is)
	{
		Vector6D lv;
		for (int i = 0; i < 6; i++)
			is >> _estimate[i];
		return true;
	}

	bool VertexSBALine::write(std::ostream& os) const
	{
		Vector6D lv = estimate();
		for (int i = 0; i < 6; i++) {
			os << lv[i] << " ";
		}
		return os.good();
	}

	class EdgeProjectLine : public  BaseBinaryEdge<4, Vector4D, VertexSBALine, VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		EdgeProjectLine();

		bool read(std::istream& is);

		bool write(std::ostream& os) const;

		void computeError() {
			const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
			const VertexSBALine* v2 = static_cast<const VertexSBALine*>(_vertices[0]);
			const CameraParameters * cam
				= static_cast<const CameraParameters *>(parameter(0));
			Vector4D obs(_measurement);
			Vector6D est = v2->estimate();
			Vector2D u1 = cam->cam_map(v1->estimate().map(Vector3D(est[0], est[1], est[2])));
			Vector2D u2 = cam->cam_map(v1->estimate().map(Vector3D(est[3], est[4], est[5])));
			double dx = obs[2] - obs[0];
			double dy = obs[3] - obs[1];
			double n = hypot(dx, dy);
			dx /= n;
			dy /= n;
			double d = -dy * obs[0] + dx * obs[1];
			double dist1 = -dy * u1[0] + dx * u1[1] - d;
			double dist2 = -dy * u2[0] + dx * u2[1] - d;
			_error = Vector4D(dist1, dist2, 0, 0);
		}

		CameraParameters * _cam;
	};

	EdgeProjectLine::EdgeProjectLine() : BaseBinaryEdge<4, Vector4D, VertexSBALine, VertexSE3Expmap>()
	{
		_cam = 0;
		resizeParameters(1);
		installParameter(_cam, 0);
	}

	bool EdgeProjectLine::read(std::istream& is)
	{
		int paramId;
		is >> paramId;
		setParameterId(0, paramId);

		for (int i = 0; i < 4; i++) {
			is >> _measurement[i];
		}
		for (int i = 0; i < 4; i++)
			for (int j = i; j < 4; j++) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
		return true;
	}

	bool EdgeProjectLine::write(std::ostream& os) const
	{
		os << _cam->id() << " ";
		for (int i = 0; i < 4; i++) {
			os << measurement()[i] << " ";
		}

		for (int i = 0; i < 4; i++)
			for (int j = i; j < 4; j++) {
				os << " " << information()(i, j);
			}
		return os.good();
		return true;
	}
}

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {

g2o::SE3Quat SolARPoseEstimationPnPL::toSE3Quat(const Transform3Df &pose)
{
	Eigen::Matrix<double, 3, 3> R;
	R << pose(0, 0), pose(0, 1), pose(0, 2),
		pose(1, 0), pose(1, 1), pose(1, 2),
		pose(2, 0), pose(2, 1), pose(2, 2);

	Eigen::Matrix<double, 3, 1> t(pose(0, 3), pose(1, 3), pose(2, 3));

	return g2o::SE3Quat(R, t);
}

Transform3Df SolARPoseEstimationPnPL::toSolarPose(const g2o::SE3Quat &SE3)
{
	Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
	Transform3Df pose;
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			pose(row, col) = (float)eigMat(row, col);
	return pose;
}

float SolARPoseEstimationPnPL::getPointReprojError(const Point2Df pt2D, const Point3Df pt3D, const Eigen::Matrix3f R, const Eigen::Vector3f t)
{
	Eigen::Vector3f U(pt3D.getX(), pt3D.getY(), pt3D.getZ());
	Eigen::Vector3f u = m_camMatrix.cast<float>() * (R * U + t);
	Point2Df pt2D_reproj( u(0) / u(2), u(1) / u(2));
	return (pt2D - pt2D_reproj).magnitude();
}

float SolARPoseEstimationPnPL::getLineReprojError(const Edge2Df ln2D, const Edge3Df ln3D, const Eigen::Matrix3f R, const Eigen::Vector3f t)
{
	Eigen::Vector3f Xs(ln3D.p1.getX(), ln3D.p1.getY(), ln3D.p1.getY());
	Eigen::Vector3f Xe(ln3D.p2.getX(), ln3D.p2.getY(), ln3D.p2.getY());
	Eigen::Vector3f xs = m_camMatrix.cast<float>() * (R * Xs + t);
	Eigen::Vector3f xe = m_camMatrix.cast<float>() * (R * Xe + t);
	Eigen::Vector3f u1(ln2D.p1.getX(), ln2D.p1.getY(), 1.f);
	Eigen::Vector3f u2(ln2D.p2.getX(), ln2D.p2.getY(), 1.f);
	return ((u1.cross(u2)).normalized() - (xs.cross(xe)).normalized()).norm();
}

SolARPoseEstimationPnPL::SolARPoseEstimationPnPL() : ConfigurableBase(xpcf::toUUID<SolARPoseEstimationPnPL>())
{
	addInterface<api::solver::pose::I3DTransformFinderFrom2D3DPointLine>(this);
	declareProperty("nbIterations", m_iterations);
	declareProperty("setVerbose", m_setVerbose);
	LOG_DEBUG("SolARPoseEstimationPnPL constructor");
}

SolARPoseEstimationPnPL::~SolARPoseEstimationPnPL()
{
	LOG_DEBUG("SolARPoseEstimationPnPL destructor");
}

org::bcom::xpcf::XPCFErrorCode SolARPoseEstimationPnPL::onConfigured()
{
	LOG_DEBUG("SolARPoseEstimationPnPL onConfigured");
	return xpcf::_SUCCESS;
}

FrameworkReturnCode SolARPoseEstimationPnPL::estimate(	const std::vector<Point2Df> & imagePoints,
														const std::vector<Point3Df >& worldPoints,
														const std::vector<Edge2Df> & imageLines,
														const std::vector<Edge3Df> & worldLines,
														Transform3Df & pose,
														const Transform3Df initialPose)
{
	int nbPoints = worldPoints.size();
	int nbLines = worldLines.size();

	// Setup optimizer
	auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(m_setVerbose);
	
	// Register Pose
	int vertex_id = 0;
	g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
	if (initialPose.isApprox(Transform3Df::Identity()))
	{
		Eigen::Vector3d trans(0.1, 0.1, 0.1);
		Eigen::Quaterniond q;
		q.setIdentity();
		g2o::SE3Quat poseQuat(q, trans);
		vSE3->setEstimate(poseQuat);
	}
	else
		vSE3->setEstimate(toSE3Quat(initialPose.inverse()));

	vSE3->setId(vertex_id++);
	optimizer.addVertex(vSE3);

	// Set Camera Intrinsics
	g2o::CameraParameters * cam_params = new g2o::CameraParameters(m_camMatrix(0, 0),
		Eigen::Vector2d(m_camMatrix(0, 2), m_camMatrix(1, 2)), 0.);
	cam_params->setId(0);
	optimizer.addParameter(cam_params);

	// Register vertex and edges for points
	for (unsigned i = 0; i < nbPoints; i++)
	{
		Point2Df point2D = imagePoints[i];
		Point3Df point3D = worldPoints[i];
		// Vertex
		g2o::VertexSBAPointXYZ * vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Eigen::Vector3d(point3D.getX(), point3D.getY(), point3D.getZ()));
		vPoint->setId(vertex_id++);
		vPoint->setMarginalized(true);
		vPoint->setFixed(true);
		optimizer.addVertex(vPoint);
		// Edge projection
		g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
		e->setVertex(0, vPoint);
		e->setVertex(1, vSE3);
		e->setMeasurement(Eigen::Vector2d(point2D.getX(), point2D.getY()));
		e->information() = Eigen::Matrix2d::Identity();
		e->setRobustKernel(new g2o::RobustKernelHuber);
		e->setParameterId(0, 0);
		optimizer.addEdge(e);
	}
	// Register vertex and edges for lines
	for (unsigned i = 0; i < nbLines; i++)
	{
		Edge2Df line2D = imageLines[i];
		Edge3Df line3D = worldLines[i];
		// Vertex
		g2o::VertexSBALine * vLine = new g2o::VertexSBALine();
		g2o::Vector6D line;
		line << line3D.p1.getX(), line3D.p1.getY(), line3D.p1.getZ(),
				line3D.p2.getX(), line3D.p2.getY(), line3D.p2.getZ();
		vLine->setEstimate(line);
		vLine->setId(vertex_id++);
		vLine->setMarginalized(true);
		vLine->setFixed(true);
		optimizer.addVertex(vLine);
		// Edge projection
		g2o::EdgeProjectLine * e = new g2o::EdgeProjectLine();
		e->setVertex(0, vLine);
		e->setVertex(1, vSE3);
		e->setMeasurement(Eigen::Vector4d(line2D.p1.getX(), line2D.p1.getY(), line2D.p2.getX(), line2D.p2.getY()));
		e->information() = Eigen::Matrix4d::Identity();
		e->setRobustKernel(new g2o::RobustKernelHuber);
		e->setParameterId(0, 0);
		optimizer.addEdge(e);
	}
	// Optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(m_iterations);
	// Retrieve Pose
	g2o::SE3Quat SE3quat = vSE3->estimate();
	pose = toSolarPose(SE3quat).inverse();

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPoseEstimationPnPL::estimateRansac(const std::vector<Point2Df>& imagePoints,
															const std::vector<Point3Df>& worldPoints,
															const std::vector<Edge2Df>& imageLines,
															const std::vector<Edge3Df>& worldLines,
															std::vector<Point2Df>& imagePoints_inliers,
															std::vector<Point3Df>& worldPoints_inliers,
															std::vector<Edge2Df>& imageLines_inliers,
															std::vector<Edge3Df>& worldLines_inliers,
															std::vector<bool>& pointInliers,
															std::vector<bool>& lineInliers,
															Transform3Df & pose,
															const Transform3Df initialPose)
{
	// RANSAC params
	int iterationsCount = 10;
	float maxReprojError = 1.f;
	float confidence = 0.99f;
	int minNbInliers = 50;
	int minRequireData = 20;

	auto rng = std::default_random_engine{};

	std::vector<Point2Df> pt2dInliers;
	std::vector<Point3Df> pt3dInliers;
	std::vector<Edge2Df> ln2dInliers;
	std::vector<Edge3Df> ln3dInliers;

	int it = 0;
	Transform3Df poseRansac;
	float bestReprojError(INFINITY);
	bool success(false);
	//while (it++ < iterationsCount)
	//{
	//	// Select n random values from data
	//	std::vector<unsigned int> indicesPt(imagePoints.size());
	//	std::vector<unsigned int> indicesLn(imageLines.size());
	//	std::iota(indicesPt.begin(), indicesPt.end(), 0);
	//	std::iota(indicesLn.begin(), indicesLn.end(), 0);
	//	std::shuffle(indicesPt.begin(), indicesPt.end(), rng);
	//	std::shuffle(indicesLn.begin(), indicesLn.end(), rng);
	//	for (int i = 0; i < minRequireData; i++)
	//	{
	//		pt2dInliers.push_back(imagePoints[indicesPt[i]]);
	//		pt3dInliers.push_back(worldPoints[indicesPt[i]]);
	//		ln2dInliers.push_back(imageLines[indicesLn[i]]);
	//		ln3dInliers.push_back(worldLines[indicesLn[i]]);
	//	}
	//	// Estimate pose
	//	estimate(pt2dInliers, pt3dInliers, ln2dInliers, ln3dInliers, poseRansac, initialPose);
	//	Eigen::Matrix3f R;
	//	Eigen::Vector3f t;
	//	for (int row = 0; row < 3; row++)
	//	{
	//		for (int col = 0; col < 3; col++)
	//			R(row, col) = poseRansac(row, col);
	//		t(row) = poseRansac(row, 3);
	//	}
	//	// Compute reproj_error for each point/line
	//	for (int i = minRequireData; i < imagePoints.size(); i++)
	//	{
	//		Point2Df pt2D = imagePoints[indicesPt[i]];
	//		Point3Df pt3D = worldPoints[indicesPt[i]];
	//		float error = getPointReprojError(pt2D, pt3D, R, t);
	//		if (error < maxReprojError)
	//		{
	//			pt2dInliers.push_back(pt2D);
	//			pt3dInliers.push_back(pt3D);
	//		}
	//	}
	//	for (int i = minRequireData; i < imageLines.size(); i++)
	//	{
	//		Edge2Df ln2D = imageLines[indicesLn[i]];
	//		Edge3Df ln3D = worldLines[indicesLn[i]];
	//		float error = getLineReprojError(ln2D, ln3D, R, t);
	//		if (error < maxReprojError)
	//		{
	//			ln2dInliers.push_back(ln2D);
	//			ln3dInliers.push_back(ln3D);
	//		}
	//	}
	//	// Check total point and line inliers
	//	if (pt2dInliers.size() + ln2dInliers.size() > minNbInliers)
	//	{
	//		// Estimate pose with all potential inliers
	//		estimate(pt2dInliers, pt3dInliers, ln2dInliers, ln3dInliers, poseRansac, initialPose);
	//		Eigen::Matrix3f R;
	//		Eigen::Vector3f t;
	//		for (int row = 0; row < 3; row++)
	//		{
	//			for (int col = 0; col < 3; col++)
	//				R(row, col) = poseRansac(row, col);
	//			t(row) = poseRansac(row, 3);
	//		}
	//		// Compute reproj error for other inliers
	//		float reproj_error(0.f);
	//		for (int i = 0; i < pt2dInliers.size(); i++)
	//			reproj_error += getPointReprojError(pt2dInliers[i], pt3dInliers[i], R, t);
	//		for (int i = 0; i < ln2dInliers.size(); i++)
	//			reproj_error += getLineReprojError(ln2dInliers[i], ln3dInliers[i], R, t);
	//		// Check against best pose
	//		if (reproj_error < bestReprojError)
	//		{
	//			success = true;
	//			bestReprojError = reproj_error;
	//			pose = poseRansac;
	//		}
	//	}
	//}

	//if (!success)
	//{
	//	LOG_WARNING("Not enough inliers to estimate pose with RANSAC");
	//	return FrameworkReturnCode::_ERROR_;
	//}

	// Retrieve final inliers
	//imagePoints_inliers.clear();
	//worldPoints_inliers.clear();
	imageLines_inliers.clear();
	worldLines_inliers.clear();
	pointInliers.clear();
	lineInliers.clear();

	estimate(imagePoints_inliers, worldPoints_inliers, imageLines, worldLines, pose, initialPose);

	Eigen::Matrix3f R;
	Eigen::Vector3f t;
	Transform3Df poseInv = pose.inverse();
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
			R(row, col) = poseInv(row, col);
		t(row) = poseInv(row, 3);
	}

	//for (int i = 0; i < imagePoints.size(); i++)
	//{
	//	Point2Df pt2D = imagePoints[i];
	//	Point3Df pt3D = worldPoints[i];
	//	float error = getPointReprojError(pt2D, pt3D, R, t);
	//	pointInliers.push_back(error < maxReprojError);
	//	if (pointInliers[i])
	//	{
	//		imagePoints_inliers.push_back(pt2D);
	//		worldPoints_inliers.push_back(pt3D);
	//	}
	//}
	for (int i = 0; i < imageLines.size(); i++)
	{
		Edge2Df ln2D = imageLines[i];
		Edge3Df ln3D = worldLines[i];
		float error = getLineReprojError(ln2D, ln3D, R, t);
		lineInliers.push_back(error < maxReprojError);
		if (lineInliers[i])
		{
			imageLines_inliers.push_back(ln2D);
			worldLines_inliers.push_back(ln3D);
		}
	}
	LOG_INFO("nbInliers: {}/{} points, {}/{} lines", imagePoints_inliers.size(), imagePoints.size(), imageLines_inliers.size(), imageLines.size());
	return FrameworkReturnCode::_SUCCESS;
}

void SolARPoseEstimationPnPL::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams)
{
	m_camDistortion(0, 0) = (double)distortionParams(0);
	m_camDistortion(1, 0) = (double)distortionParams(1);
	m_camDistortion(2, 0) = (double)distortionParams(2);
	m_camDistortion(3, 0) = (double)distortionParams(3);
	m_camDistortion(4, 0) = (double)distortionParams(4);

	m_camMatrix(0, 0) = (double)intrinsicParams(0, 0);
	m_camMatrix(0, 1) = (double)intrinsicParams(0, 1);
	m_camMatrix(0, 2) = (double)intrinsicParams(0, 2);
	m_camMatrix(1, 0) = (double)intrinsicParams(1, 0);
	m_camMatrix(1, 1) = (double)intrinsicParams(1, 1);
	m_camMatrix(1, 2) = (double)intrinsicParams(1, 2);
	m_camMatrix(2, 0) = (double)intrinsicParams(2, 0);
	m_camMatrix(2, 1) = (double)intrinsicParams(2, 1);
	m_camMatrix(2, 2) = (double)intrinsicParams(2, 2);
}

}
}
}