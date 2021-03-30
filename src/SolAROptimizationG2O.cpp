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
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
    declareProperty("nbIterationsLocal", m_iterationsLocal);
    declareProperty("nbIterationsGlobal", m_iterationsGlobal);
    declareProperty("setVerbose", m_setVerbose);
    declareProperty("nbMaxFixedKeyframes", m_nbMaxFixedKeyframes);
    declareProperty("errorOutlier", m_errorOutlier);
    declareProperty("useSpanningTree", m_useSpanningTree);
    declareProperty("isRobust", m_isRobust);
	declareProperty("fixedMap", m_fixedMap);
	declareProperty("fixedKeyframes", m_fixedKeyframes);
    LOG_DEBUG("SolAROptimizationG2O constructor");
}

SolAROptimizationG2O::~SolAROptimizationG2O()
{
    LOG_DEBUG(" SolAROptimizationG2O destructor")
}

FrameworkReturnCode SolAROptimizationG2O::setMapper(const SRef<api::solver::map::IMapper> map)
{
	map->getPointCloudManager(m_pointCloudManager);
	map->getKeyframesManager(m_keyframesManager);
	map->getCovisibilityGraph(m_covisibilityGraph);
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

double SolAROptimizationG2O::bundleAdjustment(CamCalibration & K, ATTRIBUTE(maybe_unused) CamDistortion & D, const std::vector<uint32_t> & selectKeyframes)
{
	// get cloud points and keyframes to optimize
	int iterations;
	std::vector< SRef<Keyframe>> keyframes;
	std::vector<SRef<CloudPoint>> cloudPoints;
	std::set<uint32_t> idxFixedKeyFrames;
	std::set<uint32_t> idxKeyFrames;
	if (selectKeyframes.size() > 0) {
        iterations = m_iterationsLocal;
		LOG_DEBUG("Local bundle adjustment");
		std::set<uint32_t> idxLocalCloudPoints;
		for (auto const &it_kf : selectKeyframes) {
			SRef<Keyframe> keyframe;
			m_keyframesManager->getKeyframe(it_kf, keyframe);
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
			cloudPoints.push_back(mapPoint);
			const std::map<uint32_t, uint32_t> &kpVisibility = mapPoint->getVisibility();
			for (auto const &it : kpVisibility) {
				if (idxKeyFrames.find(it.first) == idxKeyFrames.end())
					idxFixedKeyFrames.insert(it.first);
			}
			if (idxFixedKeyFrames.size() >= m_nbMaxFixedKeyframes)
				break;
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
		for (const auto &it : idxCloudPoints) {
			SRef<CloudPoint> point;
			if (m_pointCloudManager->getPoint(it, point) != FrameworkReturnCode::_SUCCESS)
				continue;
			cloudPoints.push_back(point);
		}
	}
	else {
		iterations = m_iterationsGlobal;
		LOG_INFO("Global bundle adjustment");
		// get all keyframes
		m_keyframesManager->getAllKeyframes(keyframes);
		for (const auto &kf : keyframes)
			idxKeyFrames.insert(kf->getId());
		// get all point cloud
		m_pointCloudManager->getAllPoints(cloudPoints);
	}

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
		if (m_fixedKeyframes)
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
			m_keyframesManager->getKeyframe(it, localFixedKeyframe);
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
	std::vector<uint32_t> pointEdges;
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
			m_keyframesManager->getKeyframe(idxKf, kf);
			const Keypoint &kp = kf->getUndistortedKeypoint(idxKp);
			Eigen::Matrix<double, 2, 1> obs;
			obs << kp.getX(), kp.getY();

			g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

			e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
			e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(idxKf)));
			e->setMeasurement(obs);
			e->setInformation(Eigen::Matrix2d::Identity());

			if (m_isRobust) {
				g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
				e->setRobustKernel(rk);
				rk->setDelta(thHuber2D);
			}

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
			if (edge->chi2() > m_errorOutlier * m_errorOutlier) {
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
		uint32_t cloudPoint_id = pointEdges[i];
		projErrors[cloudPoint_id].push_back(std::sqrt(e->chi2()));
	}
	for (const auto &it : projErrors) {
		SRef<CloudPoint> mapPoint;
		if (m_pointCloudManager->getPoint(it.first, mapPoint) != FrameworkReturnCode::_SUCCESS)
			continue;
		mapPoint->setReprojError(std::accumulate(it.second.begin(), it.second.end(), 0.0) / it.second.size());
	}
	return optimizer.chi2() / nbObservations;
}

}
}
}
