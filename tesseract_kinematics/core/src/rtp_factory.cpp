/**
 * @file rtp_factory.cpp
 * @brief Robot with Tool Positioner Inverse kinematics Factory implementation.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <tesseract_kinematics/core/rtp_factory.h>
#include <tesseract_kinematics/core/rtp_inv_kin.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/factory_utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_common/yaml_extensions.h>

#include <console_bridge/console.h>

namespace tesseract_kinematics
{

std::unique_ptr<InverseKinematics> RTPInvKinFactory::create(const std::string& solver_name,
                                                                const tesseract_scene_graph::SceneGraph& scene_graph,
                                                                const tesseract_scene_graph::SceneState& scene_state,
                                                                const KinematicsPluginFactory& plugin_factory,
                                                                const YAML::Node& config) const
{
  ForwardKinematics::UPtr fwd_kin;
  InverseKinematics::UPtr inv_kin;
  double m_reach{ 0 };
  std::map<std::string, JointSampleSpec> sample_res_map;
  SampleGridConfig grid;

  try
  {

    // Get tool sample resolution
    std::map<std::string, JointSampleSpec> sample_res_map;
    if (YAML::Node sample_res_node = config["tool_sample_resolution"])
    {
      sample_res_map =
          parseSampleResolutionMap(sample_res_node, scene_graph, "tool_sample_resolution");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'tool_sample_resolution' entry!");
    }

    // Get Tool Positioner
    if (YAML::Node positioner = config["tool_positioner"])
    {
      tesseract_common::PluginInfo p_info;
      if (YAML::Node n = positioner["class"])
        p_info.class_name = n.as<std::string>();
      else
        throw std::runtime_error("RTPInvKinFactory, 'tool_positioner' missing 'class' entry!");

      if (YAML::Node n = positioner["config"])
        p_info.config = n;

      fwd_kin = plugin_factory.createFwdKin(p_info.class_name, p_info, scene_graph, scene_state);
      if (fwd_kin == nullptr)
        throw std::runtime_error("RTPInvKinFactory, failed to create tool_positioner forward kinematics!");

      grid = toSampleGridConfig(sample_res_map, fwd_kin->getJointNames(), "tool_sample_resolution");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'tool_positioner' entry!");
    }

    // Get Manipulator
    if (YAML::Node manipulator = config["manipulator"])
    {
      tesseract_common::PluginInfo m_info;
      if (YAML::Node n = manipulator["class"])
        m_info.class_name = n.as<std::string>();
      else
        throw std::runtime_error("RTPInvKinFactory, 'manipulator' missing 'class' entry!");

      if (YAML::Node n = manipulator["config"])
        m_info.config = n;

      inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
      if (inv_kin == nullptr)
        throw std::runtime_error("RTPInvKinFactory, failed to create positioner inverse kinematics!");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'manipulator' entry!");
    }
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("RTPInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  // Get Reach
  if (YAML::Node n = config["manipulator_reach"])
  {
    m_reach = n.as<double>();
    return std::make_unique<RTPInvKin>(scene_graph,
                                       scene_state,
                                       std::move(inv_kin),
                                       m_reach,
                                       std::move(fwd_kin),
                                       grid.range,
                                       grid.resolution,
                                       solver_name);
  }

  // An absent manipulator_reach selects the constructor that derives the reach from the
  // manipulator chain instead.
  return std::make_unique<RTPInvKin>(scene_graph, scene_state, std::move(inv_kin), std::move(fwd_kin), grid.range, grid.resolution, solver_name);
}

PLUGIN_ANCHOR_IMPL(RTPInvKinFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::RTPInvKinFactory, RTPInvKinFactory);
