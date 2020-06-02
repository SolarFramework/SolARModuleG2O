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


#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <boost/log/core.hpp>
#include <boost/filesystem.hpp>

#include "xpcf/xpcf.h"
#include "api/display/I3DPointsViewer.h"
#include "api/solver/map/IBundler.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::storage;
namespace xpcf = org::bcom::xpcf;
namespace fs = boost::filesystem;

int main(int argc, char ** argv) {
    LOG_ADD_LOG_TO_CONSOLE();

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

	const std::string path_config = "SolARG2OBundler_conf.xml";
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
	if (xpcfComponentManager->load(path_config.c_str()) != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration {}", path_config)
			return -1;
	}
	
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
	auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
	LOG_INFO("Loaded components");
	
	std::string scene = "room15";
    const std::string path_poses        = "../../SolARTestModuleG2OBundler/" + scene + "Bundle/" + scene + "Poses.txt";
    const std::string path_points3d     = "../../SolARTestModuleG2OBundler/" + scene + "Bundle/" + scene + "Pts3D.txt";;
    const std::string path_points2d     = "../../SolARTestModuleG2OBundler/" + scene + "Bundle/" + scene + "Pts2D.txt";
    const std::string path_calibration  = "../../SolARTestModuleG2OBundler/" + scene + "Bundle/" + scene + "Calibration.txt";
    const std::string path_distortion   = "../../SolARTestModuleG2OBundler/" + scene + "Bundle/" + scene + "Distortion.txt";

	CamCalibration  intrinsic;
    CamDistortion   distortion;

	auto load2DPoints = [&](const std::string & path_measures) {
		int N;
		std::ifstream ox(path_measures);
		if (!ox.is_open()) {
			LOG_ERROR(" can't read measurements file from: {}" , path_measures);
			return false;
		}
		else {
			LOG_INFO("Loading 2D points and visibilities");;
			ox >> N;
			for (int i = 0; i < N; ++i) {				
				std::string path_measure;
				ox >> path_measure;
				fs::path path2DPoints(path_measures);
				std::ifstream ox(path2DPoints.parent_path().string() + path_measure);
				if (!ox.is_open()) {
					LOG_ERROR( " can't find observation file from: {}", path_measure);
					return false;
				}
				else {
					int kp_no;
					ox >> kp_no;
					std::vector<Keypoint> points2D;
					points2D.resize(kp_no);
					for (int j = 0; j < kp_no; ++j) {
						float x, y;
						ox >> x;
						ox >> y;
						points2D[j] = Keypoint(j, x, y, 0.0, 0.0, 0.0, 0.0, 0);
					}
					SRef<Keyframe> keyframe = xpcf::utils::make_shared<Keyframe>();
					keyframe->setKeypoints(points2D);
					keyframesManager->addKeyframe(keyframe);
				}
			}
			return true;
		}
	};

	auto loadIntrinsic = [&](const std::string&path_calib) {
		LOG_INFO("Loading intrinsics");
		std::ifstream ox(path_calib);
		if (ox.is_open()) {
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					ox >> intrinsic(i, j);
				}
			}
		}
		else {
			LOG_INFO("can't read calirabtion file from", path_calib);
			return false;
		}
		return true;
	};

    auto loadDistortions = [&](const std::string&path_dist) {
		std::ifstream ox(path_dist);
		if (!ox.is_open()) {
            LOG_INFO("can't read distortion file from", path_dist);
			return false;
		}
		else {
			LOG_INFO("Loading intrinsic ");
			for (int i = 0; i < 5; ++i) {
                ox >> distortion[i];
			}
		}
		return true;
	};

	auto loadExtrinsics = [&](const std::string & path_poses) {
		std::ifstream ox(path_poses);
		if (!ox.is_open()) {
			LOG_ERROR( "can't find poses file from: ", path_poses);
			return false;
		}
		else {
			LOG_INFO("Loading poses ");
			int N;
			ox >> N;
			for (unsigned int i = 0; i < N; ++i) {
				Transform3Df pose;
				for (int ii = 0; ii < 3; ++ii) {
					for (int jj = 0; jj < 4; ++jj) {
						ox >> pose(ii, jj);
					}
				}
				pose(3, 0) = 0.0; pose(3, 1) = 0.0; pose(3, 2) = 0.0; pose(3, 3) = 1.0;
				SRef<Keyframe> keyframe;
				keyframesManager->getKeyframe(i, keyframe);
				keyframe->setPose(pose);
			}
		}
		return true;
	};

	auto load3DPoints = [&](const std::string & path_obs) {
		std::ifstream ox(path_obs);
		if (!ox.is_open()) {
			LOG_ERROR( "can't find cloud from: ", path_obs);
			return false;
		}
		else {
			LOG_INFO("Loading 3D points");
			int obs_no;
			ox >> obs_no;
			std::vector<SRef<Keyframe>> keyframes;
			keyframesManager->getAllKeyframes(keyframes);
			for (int i = 0; i < obs_no; ++i) {
				double x, y, z;
				ox >> x;
				ox >> y;
				ox >> z;

				std::map<unsigned int, unsigned int> visibility_temp;

				SRef<CloudPoint> point3D = xpcf::utils::make_shared<CloudPoint>(x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, visibility_temp);
				pointCloudManager->addPoint(point3D);
				int viz_no; 
				ox >> viz_no;
				for (int j = 0; j < viz_no; ++j) {
					int idxView, idxLoc;
					ox >> idxView;
					ox >> idxLoc;
					point3D->addVisibility(idxView, idxLoc);
					keyframes[idxView]->addVisibility(idxLoc, point3D->getId());
				}
			}
		}
		return true;

	};

    LOG_INFO("Load data to bundle");
    load2DPoints(path_points2d);
    loadExtrinsics(path_poses);
    loadIntrinsic(path_calibration);
    loadDistortions(path_distortion);
	load3DPoints(path_points3d);

	// get all keyframes
	std::vector<SRef<Keyframe>> keyframes;
	keyframesManager->getAllKeyframes(keyframes);
	// get all point cloud
	std::vector<SRef<CloudPoint>> refPointCloud;
	pointCloudManager->getAllPoints(refPointCloud);

    // get keyframe poses before BA to display
    std::vector<Transform3Df> keyframePosesBefore;
    std::vector<uint32_t> selectedKeyframes;
    for (auto const &it : keyframes) {
        selectedKeyframes.push_back(it->getId());
        keyframePosesBefore.push_back(it->getPose());
    }

    // get point cloud before BA to display
    std::vector<SRef<CloudPoint>> pointCloudBefore;
    for (auto const &it : refPointCloud) {
        pointCloudBefore.push_back(xpcf::utils::make_shared<CloudPoint>(it->getX(), it->getY(), it->getZ()));
    }

	LOG_INFO("Number of keyframes: {}", keyframes.size());
	LOG_INFO("Number of point cloud: {}", refPointCloud.size());
		
	LOG_INFO("Keyframe1 pose before: \n{}", keyframePosesBefore[1].matrix());
	LOG_INFO("Point cloud 1 before: \n{}", *refPointCloud[1]);

	LOG_INFO("Run bundle adjustment");
    double reproj_errorFinal = bundler->solve(intrinsic, distortion, selectedKeyframes);
	LOG_INFO("Reprojection error final: {}", reproj_errorFinal);

	std::vector<Transform3Df> keyframePosesAfter;
	for (auto const &it : keyframes) {
		keyframePosesAfter.push_back(it->getPose());
	}
	LOG_INFO("Map after bundle adjustment");
	LOG_INFO("Keyframe1 pose after: \n{}", keyframePosesAfter[1].matrix());
	LOG_INFO("Point cloud 1 after: \n{}", *refPointCloud[1]);
	
    while (true) {
        if (viewer3DPoints->display(refPointCloud, keyframePosesAfter[0], keyframePosesAfter, {}, pointCloudBefore, keyframePosesBefore) == FrameworkReturnCode::_STOP) {
			break;
        }
    }
 
	return 0;
}



