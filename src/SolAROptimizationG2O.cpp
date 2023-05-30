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

#include "SolAROptimizationG2O.h"
#include <core/Log.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include "xpcf/core/helpers.h"
#include <map>
#include <numeric>

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::G2O::SolAROptimizationG2O)

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
namespace MODULES {
namespace G2O {

SolAROptimizationG2O::SolAROptimizationG2O():ConfigurableBase(xpcf::toUUID<SolAROptimizationG2O>())
{
    addInterface<api::solver::map::IBundler>(this);
    declareInjectable<IPointCloudManager>(m_pointCloudManager);
    declareInjectable<ICameraParametersManager>(m_cameraParametersManager);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraphManager>(m_covisibilityGraphManager);
    declareInjectable<api::geom::I3DTransform>(m_transform3D);
    declareProperty("nbIterationsLocal", m_iterationsLocal);
    declareProperty("nbIterationsGlobal", m_iterationsGlobal);
	declareProperty("nbIterationsSim3", m_iterationsSim3);
    declareProperty("setVerbose", m_setVerbose);
    declareProperty("nbMaxFixedKeyframes", m_nbMaxFixedKeyframes);
    declareProperty("errorOutlier", m_errorOutlier);
	declareProperty("errorSim3", m_errorSim3);
    declareProperty("useSpanningTree", m_useSpanningTree);
    declareProperty("isRobust", m_isRobust);
	declareProperty("fixedMap", m_fixedMap);
	declareProperty("fixedKeyframes", m_fixedKeyframes);
	declareProperty("fixedScale", m_fixedScale);
    LOG_DEBUG("SolAROptimizationG2O constructor");
}

SolAROptimizationG2O::~SolAROptimizationG2O()
{
    LOG_DEBUG(" SolAROptimizationG2O destructor")
}

FrameworkReturnCode SolAROptimizationG2O::setMap(const SRef<datastructure::Map> map)
{
	m_pointCloudManager->setPointCloud(map->getConstPointCloud());
    m_cameraParametersManager->setCameraParametersCollection(map->getConstCameraParametersCollection());
	m_keyframesManager->setKeyframeCollection(map->getConstKeyframeCollection());
    m_covisibilityGraphManager->setCovisibilityGraph(map->getConstCovisibilityGraph());
    m_map = map;
	return FrameworkReturnCode::_SUCCESS;
}

g2o::SE3Quat toSE3Quat(const Transform3Df &pose)
{
    Eigen::Matrix<double, 3, 3> R;
    R << pose(0, 0), pose(0, 1), pose(0, 2),
        pose(1, 0), pose(1, 1), pose(1, 2),
        pose(2, 0), pose(2, 1), pose(2, 2);

    Eigen::Matrix<double, 3, 1> t(pose(0, 3), pose(1, 3), pose(2, 3));

    return g2o::SE3Quat(R, t);
}

Transform3Df toSolarPose(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
    Transform3Df pose;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            pose(row, col) = (float)eigMat(row, col);
        }
    }

    return pose;
}

double SolAROptimizationG2O::bundleAdjustment(const std::vector<uint32_t> & selectKeyframes)
{
    // get cloud points and keyframes to optimize
	int iterations;
	std::vector< SRef<Keyframe>> keyframes;
	std::vector<SRef<CloudPoint>> tmpCloudPoints, cloudPoints;
	std::set<uint32_t> idxFixedKeyFrames;
	std::set<uint32_t> idxKeyFrames;
	if (selectKeyframes.size() > 0) {
        iterations = m_iterationsLocal;
		LOG_DEBUG("Local bundle adjustment");
		std::set<uint32_t> idxLocalCloudPoints;
		for (auto const &it_kf : selectKeyframes) {
			SRef<Keyframe> keyframe;
			if (m_keyframesManager->getKeyframe(it_kf, keyframe) != FrameworkReturnCode::_SUCCESS)
				continue;
			keyframes.push_back(keyframe);
			const std::map<uint32_t, uint32_t>& mapPointVisibility = keyframe->getVisibility();
			for (auto const &it_pc : mapPointVisibility) {
				idxLocalCloudPoints.insert(it_pc.second);
			}
		}		
		for (auto it : selectKeyframes) {
			idxKeyFrames.insert(it);
		}
		// get ids of fixed keyframes			
		for (auto const &index : idxLocalCloudPoints) {
			SRef<CloudPoint> mapPoint;
			if (m_pointCloudManager->getPoint(index, mapPoint) != FrameworkReturnCode::_SUCCESS)
				continue;
			tmpCloudPoints.push_back(mapPoint);
			const std::map<uint32_t, uint32_t> &kpVisibility = mapPoint->getVisibility();
			for (auto const &it : kpVisibility) {
				if (idxKeyFrames.find(it.first) == idxKeyFrames.end())
					idxFixedKeyFrames.insert(it.first);
			}
		}
	}		
	else if (m_useSpanningTree) {
        iterations = m_iterationsGlobal;
		LOG_INFO("Global bundle adjustment based on spanning tree");
		// get all keyframes
		m_keyframesManager->getAllKeyframes(keyframes);
		for (const auto &kf : keyframes)
			idxKeyFrames.insert(kf->getId());

		// get the maximal spanning tree
		std::vector<std::tuple<uint32_t, uint32_t, float>> edgesSpanningTree;
		float totalWeights;
		m_covisibilityGraphManager->maximalSpanningTree(edgesSpanningTree, totalWeights);

		// get cloud points belong to maximal spanning tree
		std::set<uint32_t> idxCloudPoints;
		for (const auto &edge : edgesSpanningTree) {
			SRef<Keyframe> kf1, kf2;
			if (m_keyframesManager->getKeyframe(std::get<0>(edge), kf1) != FrameworkReturnCode::_SUCCESS)
				continue;
			if (m_keyframesManager->getKeyframe(std::get<1>(edge), kf2) != FrameworkReturnCode::_SUCCESS)
				continue;
			std::map<uint32_t, uint32_t> kf1_visibilites = kf1->getVisibility();
			std::map<uint32_t, uint32_t> kf2_visibilites = kf2->getVisibility();
			std::map<uint32_t, int> countNbSeenCP;
			// get common cloud points of two keyframes
			for (const auto &it : kf1_visibilites)
				countNbSeenCP[it.second]++;
			for (const auto &it : kf2_visibilites)
				countNbSeenCP[it.second]++;
			for (const auto &it : countNbSeenCP)
				if (it.second >= 2)
					idxCloudPoints.insert(it.first);
		}
		for (const auto &it : idxCloudPoints) {
			SRef<CloudPoint> point;
			if (m_pointCloudManager->getPoint(it, point) != FrameworkReturnCode::_SUCCESS)
				continue;
			tmpCloudPoints.push_back(point);
		}
	}
	else {
		iterations = m_iterationsGlobal;
		LOG_INFO("Global bundle adjustment");
		// get all keyframes
        if (m_keyframesManager->getAllKeyframes(keyframes) == FrameworkReturnCode::_SUCCESS) {
            for (const auto &kf : keyframes)
                idxKeyFrames.insert(kf->getId());
        }
		// get all point cloud
		m_pointCloudManager->getAllPoints(tmpCloudPoints);
	}

	// filter cloud point more than 1 visibility
	for (const auto& it : tmpCloudPoints)
		if (it->getVisibility().size() > 1)
			cloudPoints.push_back(it);

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	// Set keyFrame vertices
    uint32_t maxKfId(0);
	for (int i = 0; i < keyframes.size(); i++) {
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(toSE3Quat(keyframes[i]->getPose().inverse()));
		const uint32_t &kfId = keyframes[i]->getId();
		vSE3->setId(kfId);
        if (m_fixedKeyframes || keyframes[i]->isFixedPose())
			vSE3->setFixed(true);
		else
			vSE3->setFixed(kfId == 0);
		optimizer.addVertex(vSE3);
		if (kfId > maxKfId)
			maxKfId = kfId;
	}

	// Set Fixed KeyFrame vertices if global bundle adjustment
	if (selectKeyframes.size() > 0) {		
		for (auto const &it : idxFixedKeyFrames) {
			SRef<Keyframe> localFixedKeyframe;
			if (m_keyframesManager->getKeyframe(it, localFixedKeyframe) != FrameworkReturnCode::_SUCCESS)
				continue;
			g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
			vSE3->setEstimate(toSE3Quat(localFixedKeyframe->getPose().inverse()));
			vSE3->setId(it);
			vSE3->setFixed(true);
			optimizer.addVertex(vSE3);
			if (it > maxKfId)
				maxKfId = it;
		}
	}

	const float thHuber2D = m_errorOutlier;
	int nbObservations(0);
	std::vector<g2o::EdgeSE3ProjectXYZ*> allEdges;
	std::vector<SRef<CloudPoint>> pointEdges;
	// Set MapPoint vertices
	for (int i = 0; i < cloudPoints.size(); i++) {
		const SRef<CloudPoint> &mapPoint = cloudPoints[i];
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Eigen::Matrix<double, 3, 1>(mapPoint->getX(), mapPoint->getY(), mapPoint->getZ()));
		const int id = maxKfId + 1 + mapPoint->getId();
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		if (m_fixedMap)
			vPoint->setFixed(true);
		else
			vPoint->setFixed(false);
		optimizer.addVertex(vPoint);

		const std::map<unsigned int, unsigned int> &kpVisibility = mapPoint->getVisibility();

		//Set edges between each cloud point and keyframes
		for (auto const &it : kpVisibility) {
			int idxKf = it.first;
			int idxKp = it.second;
			if ((idxKeyFrames.find(idxKf) == idxKeyFrames.end()) && (idxFixedKeyFrames.find(idxKf) == idxFixedKeyFrames.end()))
				continue;
			SRef<Keyframe> kf;
			if (m_keyframesManager->getKeyframe(idxKf, kf) != FrameworkReturnCode::_SUCCESS)
				continue;
			const Keypoint &kp = kf->getUndistortedKeypoint(idxKp);
			Eigen::Matrix<double, 2, 1> obs;
			obs << kp.getX(), kp.getY();
			g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idxKf)));
			e->setMeasurement(obs);
			const float edgeWeight = kp.getResponse();
			//e->setInformation(Eigen::Matrix2d::Identity() * edgeWeight);
			e->setInformation(Eigen::Matrix2d::Identity());
			if (m_isRobust) {
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber2D);
			}		
            SRef<CameraParameters> camParams;
            if (m_cameraParametersManager->getCameraParameters(kf->getCameraID(), camParams) != FrameworkReturnCode :: _SUCCESS)
            {
                LOG_WARNING("Camera parameteres with id {} does not exists in the camera parameters manager", kf->getCameraID());
                continue;
            }
            e->fx = camParams->intrinsic(0, 0);
            e->fy = camParams->intrinsic(1, 1);
            e->cx = camParams->intrinsic(0, 2);
            e->cy = camParams->intrinsic(1, 2);

			optimizer.addEdge(e);
			nbObservations++;
			allEdges.push_back(e);
			pointEdges.push_back(mapPoint);
		}
	}

	LOG_DEBUG("Nb keyframes: {}", keyframes.size());
	LOG_DEBUG("Nb cloud points: {}", cloudPoints.size());
	LOG_DEBUG("Nb observations: {}", nbObservations);

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.setVerbose(m_setVerbose);	

	if (m_fixedKeyframes) {
		if (m_fixedMap)
			return 0;
		optimizer.computeActiveErrors();
		LOG_DEBUG("Chi2 error before performing structure only BA: {}", optimizer.chi2());
		g2o::StructureOnlySolver<3> structure_only_ba;
		g2o::OptimizableGraph::VertexContainer points;
		for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
			g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
			if (v->dimension() == 3)
				points.push_back(v);
		}
		structure_only_ba.calc(points, iterations);
		LOG_DEBUG("Chi2 error after performing structure only BA: {}", optimizer.chi2());
	}
	else {
		optimizer.optimize(iterations / 2);
		// Filter outliers
		for (const auto &edge : allEdges) {
			if (edge->chi2() > m_errorOutlier * m_errorOutlier || !edge->isDepthPositive()) {
				edge->setLevel(1);
			}
			edge->setRobustKernel(0);
		}
		// Optimize again without the outliers
		optimizer.initializeOptimization(0);
		optimizer.optimize(iterations / 2);
	}
	// Recover optimized data
	//Keyframes
	for (int i = 0; i < keyframes.size(); i++) {
		const uint32_t &kfId = keyframes[i]->getId();
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kfId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		keyframes[i]->setPose(toSolarPose(SE3quat).inverse());
	}

	//Points
	for (int i = 0; i < cloudPoints.size(); i++) {
		const SRef<CloudPoint> &mapPoint = cloudPoints[i];
		const int id = maxKfId + 1 + mapPoint->getId();
		g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(id));
		Eigen::Matrix<double, 3, 1> xyz = vPoint->estimate();
		mapPoint->setX((float)xyz(0));
		mapPoint->setY((float)xyz(1));
		mapPoint->setZ((float)xyz(2));
	}

	//Update re-projection error of point cloud
	std::map<uint32_t, std::vector<double>> projErrors;
	for (int i = 0; i < allEdges.size(); i++) {
		g2o::EdgeSE3ProjectXYZ* e = allEdges[i];
		if (e->chi2() > m_errorOutlier * m_errorOutlier || !e->isDepthPositive())
			pointEdges[i]->setInvalid();
		else
			projErrors[pointEdges[i]->getId()].push_back(std::sqrt(e->chi2()));
	}
	double totalErr = 0.0;
	int nbErr = 0;
	for (const auto &it : projErrors) {
		SRef<CloudPoint> mapPoint;
		if (m_pointCloudManager->getPoint(it.first, mapPoint) != FrameworkReturnCode::_SUCCESS)
			continue;
		double error = std::accumulate(it.second.begin(), it.second.end(), 0.0);
		mapPoint->setReprojError(error / it.second.size());
		totalErr += error;
		nbErr += it.second.size();
	}
	return totalErr / nbErr;
}

double SolAROptimizationG2O::optimizeSim3(const SRef<Keyframe>& keyframe1, const SRef<Keyframe>& keyframe2, const std::vector<DescriptorMatch>& matches, const std::vector<Point3Df> & pts3D1, const std::vector<Point3Df> & pts3D2, Transform3Df & pose)
{
	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();
	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);
	// Init vertex 0 for sim3
	Transform3Df sim3_K1_k2 = keyframe1->getPose().inverse() * pose * keyframe2->getPose();
	Eigen::Matrix3f scaleMatrix;
	Eigen::Matrix3f rot;
	Eigen::Vector3f translation;
	float scale;
	sim3_K1_k2.computeScalingRotation(&scaleMatrix, &rot);
	scale = scaleMatrix(0, 0);
	translation = sim3_K1_k2.translation() / scale;
	g2o::Sim3 g2oS12(rot.cast<double>(), translation.cast<double>(), static_cast<double>(scale));

    SRef<CameraParameters> camParamsK1, camParamsK2;
    if (m_cameraParametersManager->getCameraParameters(keyframe1->getCameraID(), camParamsK1) != FrameworkReturnCode :: _SUCCESS)
    {
        LOG_WARNING("Camera parameteres with id {} does not exists in the camera parameters manager", keyframe1->getCameraID());
        return -1;
    }
    if (m_cameraParametersManager->getCameraParameters(keyframe2->getCameraID(), camParamsK2) != FrameworkReturnCode :: _SUCCESS)
    {
        LOG_WARNING("Camera parameteres with id {} does not exists in the camera parameters manager", keyframe2->getCameraID());
        return -1;
    }
	g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
	vSim3->_fix_scale = (bool)m_fixedScale;
	vSim3->setEstimate(g2oS12);
	vSim3->setId(0);
	vSim3->setFixed(false);
    vSim3->_principle_point1[0] = camParamsK1->intrinsic(0, 2);
    vSim3->_principle_point1[1] = camParamsK1->intrinsic(1, 2);
    vSim3->_focal_length1[0] = camParamsK1->intrinsic(0, 0);
    vSim3->_focal_length1[1] = camParamsK1->intrinsic(1, 1);
    vSim3->_principle_point2[0] = camParamsK2->intrinsic(0, 2);
    vSim3->_principle_point2[1] = camParamsK2->intrinsic(1, 2);
    vSim3->_focal_length2[0] = camParamsK2->intrinsic(0, 0);
    vSim3->_focal_length2[1] = camParamsK2->intrinsic(1, 1);
	optimizer.addVertex(vSim3);

	// Set MapPoint vertices
	const int N = pts3D1.size();
	std::vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
	std::vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
	std::vector<size_t> vnIndexEdge;
	vnIndexEdge.reserve(2 * N);
	vpEdges12.reserve(2 * N);
	vpEdges21.reserve(2 * N);	

	Transform3Df pose1 = keyframe1->getPose().inverse(); 
	Transform3Df pose2 = keyframe2->getPose().inverse(); 
	std::vector<Point3Df> pts3D1Transformed, pts3D2Transformed;
	m_transform3D->transform(pts3D1, pose1, pts3D1Transformed);
	m_transform3D->transform(pts3D2, pose2, pts3D2Transformed);

	const float deltaHuber = m_errorOutlier;
	for (int i = 0; i < N; i++) {
		const int id1 = 2 * i + 1;
		const int id2 = 2 * (i + 1);

		g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
		Eigen::Matrix<double, 3, 1> v1;
		v1 << pts3D1Transformed[i].getX(), pts3D1Transformed[i].getY(), pts3D1Transformed[i].getZ();
		vPoint1->setEstimate(v1);
		vPoint1->setId(id1);
		vPoint1->setFixed(true);
		optimizer.addVertex(vPoint1);

		/// use project function to see the result  of the reproj error !
		g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
		Eigen::Matrix<double, 3, 1> v2;
		v2 << pts3D2Transformed[i].getX(), pts3D2Transformed[i].getY(), pts3D2Transformed[i].getZ();
		vPoint2->setEstimate(v2);
		vPoint2->setId(id2);
		vPoint2->setFixed(true);
		optimizer.addVertex(vPoint2);

		const Keypoint& kp1 = keyframe1->getKeypoint(matches[i].getIndexInDescriptorA());
		const Keypoint& kp2 = keyframe2->getKeypoint(matches[i].getIndexInDescriptorB());

		// Set edge x1 = S12*X2		
		Eigen::Matrix<double, 2, 1> obs1;
		obs1 << kp1.getX(), kp1.getY();
		g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
		e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
		e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e12->setMeasurement(obs1);
		e12->setInformation(Eigen::Matrix2d::Identity());
		if (m_isRobust) {
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e12->setRobustKernel(rk);
			rk->setDelta(deltaHuber);
		}
		optimizer.addEdge(e12);

		// Set edge x2 = S21*X1
		Eigen::Matrix<double, 2, 1> obs2;
		obs2 << kp2.getX(), kp2.getY();
		g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
		e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
		e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
		e21->setMeasurement(obs2);
		e21->setInformation(Eigen::Matrix2d::Identity());
		if (m_isRobust) {
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e12->setRobustKernel(rk);
			rk->setDelta(deltaHuber);
		}
		optimizer.addEdge(e21);

		vpEdges12.push_back(e12);
		vpEdges21.push_back(e21);
		vnIndexEdge.push_back(i);

	}
	const int sim3MinInliers   = 10;
	optimizer.initializeOptimization();
	optimizer.setVerbose(m_setVerbose);
	// set number of iterations as argument
	optimizer.optimize(m_iterationsSim3);
	int sim3Inliers = vpEdges12.size();
	for (size_t i = 0; i < vpEdges12.size(); i++){
		g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
		g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
		if (!e12 || !e21)
			continue;
		if (std::sqrt(e12->chi2()) > m_errorSim3){
			size_t idx = vnIndexEdge[i];
			optimizer.removeEdge(e12);
			optimizer.removeEdge(e21);
			vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
			vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
			sim3Inliers--;
		}
	}

	if (sim3Inliers > sim3MinInliers) {
		LOG_INFO("Number of Sim3 inliers {}\n: ", sim3Inliers);
		LOG_INFO("Inliers-based Sim3 optimization\n: ");
		optimizer.initializeOptimization();
		optimizer.setVerbose(m_setVerbose);
		// set number of iterations as argument
		optimizer.optimize(m_iterationsSim3/2);
	}
	g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
	g2oS12 = vSim3_recov->estimate();

	// optimized pose
	pose.linear() = static_cast<float>(g2oS12.scale()) * g2oS12.rotation().toRotationMatrix().cast<float>();
	pose.translation() = static_cast<float>(g2oS12.scale()) * g2oS12.translation().cast<float>();

	pose = keyframe1->getPose() * pose * keyframe2->getPose().inverse();

	return optimizer.chi2();
}

}
}
}
