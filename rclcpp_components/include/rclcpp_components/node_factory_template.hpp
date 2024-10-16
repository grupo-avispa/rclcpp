// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__
#define RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__

#include <functional>
#include <memory>

#include "rclcpp_components/node_factory_rt.hpp"

#include "cactus_rt/tracing.h" // for tracing

namespace rclcpp_components
{

/// NodeFactoryTemplate is a convenience class for instantiating components.
/**
 * The NodeFactoryTemplate class can be used to provide the NodeFactory interface for
 * components that implement a single-argument constructor and `get_node_base_interface`.
 */
template<typename NodeT>
class NodeFactoryTemplate : public NodeFactoryRT
{
public:
  NodeFactoryTemplate() = default;
  virtual ~NodeFactoryTemplate() = default;

  /// Create an instance of a component
  /**
   * \param[in] options Additional options used in the construction of the component.
   */
  NodeInstanceWrapper
  create_node_instance(const rclcpp::NodeOptions & options, 
                      std::shared_ptr<cactus_rt::tracing::ThreadTracer> tracer = nullptr) override
  {
    // RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "Construyendo el nodo...");
    std::shared_ptr<NodeT> node;
    if (tracer != nullptr){
      node = std::make_shared<NodeT>(options, tracer);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
          "Tracer shared_ptr for %s was successfully set !!", node->get_name());
    }else{
      node = std::make_shared<NodeT>(options, nullptr);
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
          "Tracer shared_ptr for %s was NULL !!", node->get_name());
    }
    return NodeInstanceWrapper(
      node, std::bind(&NodeT::get_node_base_interface, node));
  }
};
}  // namespace rclcpp_components

#endif  // RCLCPP_COMPONENTS__NODE_FACTORY_TEMPLATE_HPP__
