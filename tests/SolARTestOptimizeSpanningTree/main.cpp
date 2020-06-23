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

#include <boost/log/core.hpp>
 // ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
// ADD COMPONENTS HEADERS HERE
#include "api/input/devices/ICamera.h"
#include "api/solver/map/IMapper.h"
#include "api/display/I3DPointsViewer.h"
#include "api/solver/map/IBundler.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;


namespace xpcf = org::bcom::xpcf;

int main(int argc, char ** argv) {
    LOG_ADD_LOG_TO_CONSOLE();

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

	const std::string path_config = "SolAROptimizeSpanningTree_conf.xml";
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();
	if (xpcfComponentManager->load(path_config.c_str()) != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration {}", path_config)
			return -1;
	}
	
	auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
	auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
	auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
	auto bundler = xpcfComponentManager->resolve<api::solver::map::IBundler>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
	LOG_INFO("Loaded components");

	CamCalibration calibration = camera->getIntrinsicsParameters();
	CamDistortion distortion = camera->getDistortionParameters();

	// Load map from file
	if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
	}
	else {
		LOG_ERROR("Cannot load map to optimize");
		return 0;
	}

	// get all keyframes
	std::vector<SRef<Keyframe>> keyframes;
	keyframesManager->getAllKeyframes(keyframes);
	// get all point cloud
	std::vector<SRef<CloudPoint>> refPointCloud;
	pointCloudManager->getAllPoints(refPointCloud);

    // get keyframe poses before BA to display
    std::vector<Transform3Df> keyframePosesBefore;
    for (auto const &it : keyframes) {
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

	LOG_INFO("Run global bundle adjustment on the spanning tree");
	clock_t start, end;
	start = clock();
    double reproj_errorFinal = bundler->optimizeSpanningTree(calibration, distortion);
	end = clock();
	double duration = double(end - start) / CLOCKS_PER_SEC;
	LOG_INFO("Execution time : {}", duration);
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



