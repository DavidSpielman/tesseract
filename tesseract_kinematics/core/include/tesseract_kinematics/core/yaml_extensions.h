/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions for kinematics types.
 *
 * @author Levi Armstrong
 * @date April 4, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_YAML_EXTENSIONS_H
#define TESSERACT_KINEMATICS_YAML_EXTENSIONS_H

#include <tesseract_kinematics/core/types.h>
#include <tesseract_common/fwd.h>
#include <yaml-cpp/yaml.h>

namespace YAML
{
template <>
struct convert<tesseract_kinematics::PositionerSampleResolution>
{
  static Node encode(const tesseract_kinematics::PositionerSampleResolution& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["value"] = rhs.value;
    if (rhs.min.has_value())
      node["min"] = rhs.min.value();
    if (rhs.max.has_value())
      node["max"] = rhs.max.value();
    return node;
  }

  static bool decode(const Node& node, tesseract_kinematics::PositionerSampleResolution& rhs)
  {
    if (YAML::Node n = node["name"])
      rhs.name = n.as<std::string>();
    else
      throw std::runtime_error("PositionerSampleResolution, missing 'name' entry!");

    if (YAML::Node n = node["value"])
      rhs.value = n.as<double>();
    else
      throw std::runtime_error("PositionerSampleResolution, missing 'value' entry!");

    if (YAML::Node n = node["min"])
      rhs.min = n.as<double>();

    if (YAML::Node n = node["max"])
      rhs.max = n.as<double>();

    return true;
  }

};
}  // namespace YAML

#endif  // TESSERACT_KINEMATICS_YAML_EXTENSIONS_H
