#include "tuw_graph_rviz_plugins/graph_display.hpp"

#include <memory>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/shape.hpp"
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
: origin_axes_(nullptr), pose_valid_(false)
{

  origin_axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));

  origin_axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));

  vertices_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    this, SLOT(updateVerticesGeometry()));
  vertices_alpha_property_->setMin(0);
  vertices_alpha_property_->setMax(1);

  vertices_width_property_ = new rviz_common::properties::FloatProperty(
    "Width", 0.05f, "Vertices path width between edges.",
    this, SLOT(updateVerticesGeometry()));

  vertices_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrow.",
    this, SLOT(updateVerticesGeometry()));
}

void GraphDisplay::onInitialize()
{
  MFDClass::onInitialize();
  origin_axes_ = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_,
    origin_axes_length_property_->getFloat(),
    origin_axes_radius_property_->getFloat());
  updateAxisGeometry();
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
  coll_handler_->addTrackedObjects(origin_axes_->getSceneNode());
}

void GraphDisplay::onDisable()
{
  MFDClass::onDisable();
  coll_handler_.reset();
}

void GraphDisplay::updateAxisGeometry()
{
  origin_axes_->set(
    origin_axes_length_property_->getFloat(),
    origin_axes_radius_property_->getFloat());
  context_->queueRender();
}

void GraphDisplay::updateVerticesGeometry()
{
  float width = vertices_width_property_->getFloat();
  Ogre::ColourValue color = vertices_color_property_->getOgreColor();
  color.a = vertices_alpha_property_->getFloat();
  for(auto &path: paths_) {
    path->setColor(color.r, color.g, color.b, color.a);
    path->setLineWidth(width);
  }
  context_->queueRender();
}

void GraphDisplay::updateShapeVisibility()
{
  if (pose_valid_) {
    origin_axes_->getSceneNode()->setVisible(true);
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

  size_t id_vertex = 0;
  float width = vertices_width_property_->getFloat();
  Ogre::ColourValue color = vertices_color_property_->getOgreColor();
  color.a = vertices_alpha_property_->getFloat();
  for (auto vertex: message->vertices){
    if(paths_.size() <= id_vertex)  {
      /// create vertices
      paths_.push_back(std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_));
      paths_.back()->setColor(color.r, color.g, color.b, color.a);
      paths_.back()->setLineWidth(width);
    }
    paths_[id_vertex]->clear();
    for(size_t i = 0; i < vertex.path.size(); i++){
      geometry_msgs::msg::Point &p   = vertex.path[i];
      paths_[id_vertex]->addPoint(Ogre::Vector3(p.x, p.y, p.z));
    }
    id_vertex++;
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