/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_SELECTION_HANDLER_HPP_
#define TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_SELECTION_HANDLER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/interaction/selection_handler.hpp"

#include "tuw_graph_rviz_plugins/graph_display.hpp"
#include "tuw_graph_rviz_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
class VectorProperty;
class QuaternionProperty;
}  // namespace properties
}  // namespace rviz_common

namespace tuw_graph_rviz_plugins
{
namespace displays
{

class TUW_GRAPH_RVIZ_PLUGINS_PUBLIC GraphDisplaySelectionHandler : public
  rviz_common::interaction::SelectionHandler
{
public:
  void createProperties(
    const rviz_common::interaction::Picked & obj,
    rviz_common::properties::Property * parent_property) override;

  rviz_common::interaction::V_AABB getAABBs(const rviz_common::interaction::Picked & obj) override;

  void setMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr message);

private:
  GraphDisplaySelectionHandler(GraphDisplay * display, rviz_common::DisplayContext * context);

  GraphDisplay * display_;
  rviz_common::properties::StringProperty * frame_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;

  template<typename T, typename ... Args>
  friend typename std::shared_ptr<T>
  rviz_common::interaction::createSelectionHandler(Args ... arguments);
};

}  // namespace displays
}  // namespace tuw_graph_rviz_plugins

#endif  // TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_SELECTION_HANDLER_HPP_