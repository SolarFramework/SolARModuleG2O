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

#ifndef SOLARMODULEG2O_TRAITS_H
#define SOLARMODULEG2O_TRAITS_H

#include "xpcf/api/IComponentManager.h"

namespace SolAR {
namespace MODULES {
/**
 * @namespace SolAR::MODULES::G20
 * @brief <B>Provides a bundle adjustment component based on G20 library: https://github.com/OpenSLAM-org/openslam_g2o</B>
 * <TT>UUID: 8f94a3c5-79ed-4851-9502-98033eae3a3b</TT>
 *
 */
namespace G2O {
class SolAROptimizationG2O;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::G2O::SolAROptimizationG2O,
                             "870d89ba-bb5f-460a-a817-1fcb6473df70",
                             "SolAROptimizationG2O",
                             "Bundle adjustment optimization")

#endif // SOLARMODULEG2O_TRAITS_H
