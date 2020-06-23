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
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <map>
#include <numeric>

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::G2O::SolAROptimizationG2O)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {

SolAROptimizationG2O::SolAROptimizationG2O():ConfigurableBase(xpcf::toUUID<SolAROptimizationG2O>())
{
	addInterface<api::solver::map::IBundler>(this);
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
	declareProperty("nbIterations", m_iterations);
	declareProperty("setVerbose", m_setVerbose);
	declareProperty("nbMaxFixedKeyframes", m_nbMaxFixedKeyframes);
	LOG_DEBUG("SolAROptimizationG2O constructor");
}

SolAROptimizationG2O::~SolAROptimizationG2O()
{
    LOG_DEBUG(" SolAROptimizationG2O destructor")
}

xpcf::XPCFErrorCode SolAROptimizationG2O::onConfigured()
{
    LOG_DEBUG(" SolAROptimizationG2O onConfigured");

    return xpcf::_SUCCESS;
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

double SolAROptimizationG2O::solve(CamCalibration & K, CamDistortion & D, const std::vector<uint32_t>& selectKeyframes)
{
	// Local KeyFrames
	std::set<unsigned int> idxLocalKeyFrames;
	for (auto it : selectKeyframes) {
		idxLocalKeyFrames.insert(it);
	}
	
	// Local MapPoints seen in Local KeyFrames
	std::set<uint32_t> idxLocalCloudPoints;
	std::vector< SRef<Keyframe>> localKeyframes;
	for (auto const &it_kf : idxLocalKeyFrames) {
		SRef<Keyframe> localKeyframe;		
		m_keyframesManager->getKeyframe(it_kf, localKeyframe);
		localKeyframes.push_back(localKeyframe);
		const std::map<uint32_t, uint32_t>& mapPointVisibility = localKeyframe->getVisibility();
		for (auto const &it_pc : mapPointVisibility) {
			idxLocalCloudPoints.insert(it_pc.second);
		}
	}

	// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
	std::set<uint32_t> idxFixedKeyFrames;
	std::vector<SRef<CloudPoint>> localCloudPoints;
	for (auto const &index : idxLocalCloudPoints) {
		SRef<CloudPoint> mapPoint;
		m_pointCloudManager->getPoint(index, mapPoint);
		localCloudPoints.push_back(mapPoint);
		const std::map<uint32_t, uint32_t> &kpVisibility = mapPoint->getVisibility();
		for (auto const &it : kpVisibility) {
			if (idxLocalKeyFrames.find(it.first) == idxLocalKeyFrames.end())
				idxFixedKeyFrames.insert(it.first);
		}
		if (idxFixedKeyFrames.size() >= m_nbMaxFixedKeyframes)
			break;
	}

	LOG_DEBUG("Nb Local Keyframes: {}", idxLocalKeyFrames.size());
	LOG_DEBUG("Nb Fixed Keyframes: {}", idxFixedKeyFrames.size());
	LOG_DEBUG("Nb Local pointclos: {}", idxLocalCloudPoints.size());

	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	// Set Local KeyFrame vertices
	int maxKfId(0);
	for (int i = 0; i < localKeyframes.size(); i++){
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(toSE3Quat(localKeyframes[i]->getPose().inverse()));
		const uint32_t &kfId = localKeyframes[i]->getId();
		vSE3->setId(kfId);
		vSE3->setFixed(kfId == 0);
		optimizer.addVertex(vSE3);
		if (kfId > maxKfId)
			maxKfId = kfId;
	}

	// Set Fixed KeyFrame vertices
	for (auto const &it : idxFixedKeyFrames){
		SRef<Keyframe> localFixedKeyframe;
		m_keyframesManager->getKeyframe(it, localFixedKeyframe);
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(toSE3Quat(localFixedKeyframe->getPose().inverse()));
		vSE3->setId(it);
		vSE3->setFixed(true);
		optimizer.addVertex(vSE3);
		if (it > maxKfId)
			maxKfId = it;
	}

	const float thHuber2D = sqrt(5.99);
	int nbObservations(0);
	std::vector<g2o::EdgeSE3ProjectXYZ*> allEdges;
	std::vector<uint32_t> pointEdges;
	// Set MapPoint vertices
	for (int i = 0; i < localCloudPoints.size(); i++){
		const SRef<CloudPoint> &mapPoint = localCloudPoints[i];
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Eigen::Matrix<double, 3, 1>(mapPoint->getX(), mapPoint->getY(), mapPoint->getZ()));
		const int id = maxKfId + 1 + mapPoint->getId();
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		const std::map<unsigned int, unsigned int> &kpVisibility = mapPoint->getVisibility();

		//Set edges between each cloud point and keyframes
		for (auto const &it : kpVisibility) {
			int idxKf = it.first;
			int idxKp = it.second;
			if ((idxLocalKeyFrames.find(idxKf) == idxLocalKeyFrames.end()) && (idxFixedKeyFrames.find(idxKf) == idxFixedKeyFrames.end()))
				continue;
			SRef<Keyframe> kf;
			m_keyframesManager->getKeyframe(idxKf, kf);
			const Keypoint &kp = kf->getKeypoints()[idxKp];
			Eigen::Matrix<double, 2, 1> obs;
			obs << kp.getX(), kp.getY();

			g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idxKf)));
			e->setMeasurement(obs);
			e->setInformation(Eigen::Matrix2d::Identity());

			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			e->setRobustKernel(rk);
			rk->setDelta(thHuber2D);

			e->fx = K(0, 0);
			e->fy = K(1, 1);
			e->cx = K(0, 2);
			e->cy = K(1, 2);
			optimizer.addEdge(e);
			nbObservations++;
			allEdges.push_back(e);
			pointEdges.push_back(mapPoint->getId());
		}
	}

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.setVerbose(m_setVerbose);
	optimizer.optimize(m_iterations);
	// Recover optimized data	
	//Keyframes
	for (int i = 0; i < localKeyframes.size(); i++){
		const uint32_t &kfId = localKeyframes[i]->getId();
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kfId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		localKeyframes[i]->setPose(toSolarPose(SE3quat).inverse());
	}

	//Points
	for (int i = 0; i < localCloudPoints.size(); i++)
	{
		const SRef<CloudPoint> &mapPoint = localCloudPoints[i];
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
		uint32_t cloudPoint_id = pointEdges[i];
		projErrors[cloudPoint_id].push_back(std::sqrt(e->chi2()));
	}
	for (const auto &it : projErrors) {
		SRef<CloudPoint> mapPoint;
		m_pointCloudManager->getPoint(it.first, mapPoint);
		mapPoint->setReprojError(std::accumulate(it.second.begin(), it.second.end(), 0.0) / it.second.size());
	}
	return optimizer.chi2() / nbObservations;
}

double SolAROptimizationG2O::optimizeSpanningTree(CamCalibration & K, CamDistortion & D)
{
	// get all keyframes
	std::vector<SRef<Keyframe>> keyframes;
	m_keyframesManager->getAllKeyframes(keyframes);

	// get the maximal spanning tree
	std::vector<std::tuple<uint32_t, uint32_t, float>> edgesSpanningTree;
	float totalWeights;
	m_covisibilityGraph->maximalSpanningTree(edgesSpanningTree, totalWeights);

	// get cloud points belong to maximal spanning tree	
	std::set<uint32_t> idxCloudPoints;
	for (const auto &edge : edgesSpanningTree) {
		SRef<Keyframe> kf1, kf2;
		m_keyframesManager->getKeyframe(std::get<0>(edge), kf1);
		m_keyframesManager->getKeyframe(std::get<1>(edge), kf2);
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
	std::vector<SRef<CloudPoint>> cloudPoints;
	for (const auto &it : idxCloudPoints) {
		SRef<CloudPoint> point;
		m_pointCloudManager->getPoint(it, point);
		cloudPoints.push_back(point);
	}
	
	// Setup optimizer
	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

	auto solver = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
	optimizer.setAlgorithm(solver);

	// Set keyFrames vertices
	int maxKfId(0);
	for (int i = 0; i < keyframes.size(); i++) {
		g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(toSE3Quat(keyframes[i]->getPose().inverse()));
		const uint32_t &kfId = keyframes[i]->getId();
		vSE3->setId(kfId);
		vSE3->setFixed(kfId == 0);
		optimizer.addVertex(vSE3);
		if (kfId > maxKfId)
			maxKfId = kfId;
	}

	const float thHuber2D = sqrt(5.99);
	int nbObservations(0);
	std::vector<g2o::EdgeSE3ProjectXYZ*> allEdges;
	std::vector<uint32_t> pointEdges;
	// Set MapPoint vertices
	for (int i = 0; i < cloudPoints.size(); i++) {
		const SRef<CloudPoint> &mapPoint = cloudPoints[i];
		g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Eigen::Matrix<double, 3, 1>(mapPoint->getX(), mapPoint->getY(), mapPoint->getZ()));
		const int id = maxKfId + 1 + mapPoint->getId();
		vPoint->setId(id);
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);

		const std::map<unsigned int, unsigned int> &kpVisibility = mapPoint->getVisibility();

		//Set edges between each cloud point and keyframes
		for (auto const &it : kpVisibility) {
			int idxKf = it.first;
			int idxKp = it.second;
			SRef<Keyframe> kf;
			m_keyframesManager->getKeyframe(idxKf, kf);
			const Keypoint &kp = kf->getKeypoints()[idxKp];
			Eigen::Matrix<double, 2, 1> obs;
			obs << kp.getX(), kp.getY();

			g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idxKf)));
			e->setMeasurement(obs);
			e->setInformation(Eigen::Matrix2d::Identity());

			//g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			//e->setRobustKernel(rk);
			//rk->setDelta(thHuber2D);

			e->fx = K(0, 0);
			e->fy = K(1, 1);
			e->cx = K(0, 2);
			e->cy = K(1, 2);
			optimizer.addEdge(e);
			nbObservations++;
			allEdges.push_back(e);
			pointEdges.push_back(mapPoint->getId());
		}
	}

	LOG_INFO("Nb keyframes: {}", keyframes.size());
	LOG_INFO("Nb cloud points: {}", cloudPoints.size());
	LOG_INFO("Nb observations: {}", nbObservations);

	// Optimize!
	optimizer.initializeOptimization();
	optimizer.setVerbose(m_setVerbose);
	optimizer.optimize(m_iterations);
	// Recover optimized data	
	//Keyframes
	for (int i = 0; i < keyframes.size(); i++) {
		const uint32_t &kfId = keyframes[i]->getId();
		g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kfId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		keyframes[i]->setPose(toSolarPose(SE3quat).inverse());
	}

	//Points
	for (int i = 0; i < cloudPoints.size(); i++)
	{
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
		uint32_t cloudPoint_id = pointEdges[i];
		projErrors[cloudPoint_id].push_back(std::sqrt(e->chi2()));
	}
	for (const auto &it : projErrors) {
		SRef<CloudPoint> mapPoint;
		m_pointCloudManager->getPoint(it.first, mapPoint);
		mapPoint->setReprojError(std::accumulate(it.second.begin(), it.second.end(), 0.0) / it.second.size());
	}
	return optimizer.chi2() / nbObservations;
}

}
}
}
