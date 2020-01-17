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

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::G2O::SolAROptimizationG2O)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace G2O {

SolAROptimizationG2O::SolAROptimizationG2O():ConfigurableBase(xpcf::toUUID<SolAROptimizationG2O>())
{

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

double SolAROptimizationG2O::solve( const std::vector<SRef<Keyframe>> & originalKeyframes,
                                    const std::vector<CloudPoint> & originalCloud,
                                    const  CamCalibration & originalCalib,
                                    const CamDistortion & originalDist,
                                    const std::vector<int> & selectKeyframes,
                                    std::vector<Transform3Df> & correctedPose,
                                    std::vector<CloudPoint>&correctedCloud,
                                    CamCalibration&correctedCalib,
                                    CamDistortion &correctedDist)
{
    return 0.0;
}

}
}
}
