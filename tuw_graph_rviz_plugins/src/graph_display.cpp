/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "tuw_graph_rviz_plugins/graph_display.hpp"

#include <memory>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/validate_floats.hpp"
#include "tuw_graph_rviz_plugins/graph_display_selection_handler.hpp"

namespace tuw_graph_rviz_plugins
{
namespace displays
{

GraphDisplay::GraphDisplay()
: axes_(nullptr), pose_valid_(false)
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrow.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));
}

void GraphDisplay::onInitialize()
{
  MFDClass::onInitialize();

  axes_ = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_,
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());

  updateShapeChoice();
  updateColorAndAlpha();
}

GraphDisplay::~GraphDisplay() = default;

void GraphDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
  setupSelectionHandler();
}

void GraphDisplay::setupSelectionHandler()
{
  coll_handler_ = rviz_common::interaction::createSelectionHandler
    <GraphDisplaySelectionHandler>(this, context_);
  coll_handler_->addTrackedObjects(axes_->getSceneNode());
}

void GraphDisplay::onDisable()
{
  MFDClass::onDisable();
  coll_handler_.reset();
}

void GraphDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();
  for(auto &l: lines_) l->setColor(color);
  context_->queueRender();
}

void GraphDisplay::updateAxisGeometry()
{
  axes_->set(
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  context_->queueRender();
}

void GraphDisplay::updateShapeChoice()
{
  updateShapeVisibility();

  context_->queueRender();
}

void GraphDisplay::updateShapeVisibility()
{
  if (pose_valid_) {
    axes_->getSceneNode()->setVisible(true);
  }
}

void GraphDisplay::processMessage(tuw_graph_msgs::msg::Graph::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->origin)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (
    !context_->getFrameManager()->transform(
      message->header, message->origin, position, orientation))
  {
    setMissingTransformToFixedFrame(message->header.frame_id);
    return;
  }
  setTransformOk();

  pose_valid_ = true;


  size_t id_line = 0;
  for (auto vertex: message->vertices){
    for(size_t i = 1; i < vertex.path.size(); i++){
      if(lines_.size() <= id_line)  {
        lines_.push_back(std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_));
        Ogre::ColourValue color = color_property_->getOgreColor();
        lines_.back()->setColor(color);
      }
      geometry_msgs::msg::Point &start = vertex.path[i-1];
      geometry_msgs::msg::Point &end   = vertex.path[i];
      lines_[id_line++]->setPoints(Ogre::Vector3(start.x, start.y, start.z), Ogre::Vector3(end.x, end.y, end.z));
    }
  }

  updateShapeVisibility();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  
  


  coll_handler_->setMessage(message);

  context_->queueRender();
}

void GraphDisplay::reset()
{
  MFDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}

}  // namespace displays
}  // namespace tuw_graph_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(tuw_graph_rviz_plugins::displays::GraphDisplay, rviz_common::Display)