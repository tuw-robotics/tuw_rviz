#include "tuw_graph_rviz_plugins/graph_display.hpp"

#include <OgreSceneNode.h>

#include <memory>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/line.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "tuw_graph_rviz_plugins/graph_display_selection_handler.hpp"

namespace tuw_graph_rviz_plugins
{
namespace displays
{
GraphDisplay::GraphDisplay() : origin_axes_(nullptr), pose_valid_(false)
{
  origin_axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  origin_axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  origin_axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  path_show_property_ = new rviz_common::properties::BoolProperty(
    "Show Path", true, "Shows path between nodes.", this, SLOT(updateShapeVisibility()));
  path_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Path alpha", 1, "Amount of transparency to apply to the path.", this,
    SLOT(updatePathGeometry()));
  path_alpha_property_->setMin(0);
  path_alpha_property_->setMax(1);
  path_color_property_ = new rviz_common::properties::ColorProperty(
    "Path color", QColor(0, 255, 0), "Color to draw path", this, SLOT(updatePathGeometry()));

  edge_arrow_property_ = new rviz_common::properties::FloatProperty(
    "Edge Arrow Size", 0.1f, "Size of the the edge arrow. 0 means off.", this,
    SLOT(updateShapeVisibility()));
  edge_show_property_ = new rviz_common::properties::BoolProperty(
    "Draw Edge", true, "Draws edges between nodes.", this, SLOT(updateShapeVisibility()));
  edge_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Edges alpha", 1, "Amount of transparency to apply to the edges.", this,
    SLOT(updateEdgesGeometry()));
  edge_alpha_property_->setMin(0);
  edge_alpha_property_->setMax(1);
  edge_color_property_ = new rviz_common::properties::ColorProperty(
    "Edges color", QColor(0, 255, 255), "Color to draw edges", this, SLOT(updateEdgesGeometry()));

  node_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Nodes alpha", 1, "Amount of transparency to apply to the nodes.", this,
    SLOT(updateNodesGeometry()));
  node_show_property_ = new rviz_common::properties::BoolProperty(
    "Draw Node", true, "Draws nodes.", this, SLOT(updateShapeVisibility()));
  node_alpha_property_->setMin(0);
  node_alpha_property_->setMax(1);
  node_color_property_ = new rviz_common::properties::ColorProperty(
    "Nodes color", QColor(255, 0, 255), "Color to draw nodes", this, SLOT(updateNodesGeometry()));
  node_size_property_ = new rviz_common::properties::FloatProperty(
    "Nodes size", 0.15f, "Node size.", this, SLOT(updateNodesGeometry()));
}

void GraphDisplay::onInitialize()
{
  MFDClass::onInitialize();
  origin_axes_ = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_, origin_axes_length_property_->getFloat(),
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
  coll_handler_ =
    rviz_common::interaction::createSelectionHandler<GraphDisplaySelectionHandler>(this, context_);
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
    origin_axes_length_property_->getFloat(), origin_axes_radius_property_->getFloat());
  context_->queueRender();
}

void GraphDisplay::updateNodesGeometry()
{
  float size = node_size_property_->getFloat();
  Ogre::ColourValue color = node_color_property_->getOgreColor();
  color.a = node_alpha_property_->getFloat();
  for (auto & [id, node] : nodes_) {
    node.shape->setColor(color.r, color.g, color.b, color.a);
    node.shape->setScale(Ogre::Vector3(size, size, size));
  }
  context_->queueRender();
}

void GraphDisplay::updateEdgesGeometry()
{
  Ogre::ColourValue color_edge = edge_color_property_->getOgreColor();
  color_edge.a = edge_alpha_property_->getFloat();
  Ogre::ColourValue color_path = path_color_property_->getOgreColor();
  color_path.a = path_alpha_property_->getFloat();
  for (auto &[id, edge] : edges_) {
    edge.line->setColor(color_edge);
    edge.arrowL->setColor(color_edge);
    edge.arrowR->setColor(color_edge);
    for (auto & line : edge.path) {
      line->setColor(color_path);
    }
  }
  context_->queueRender();
}

void GraphDisplay::updateShapeVisibility()
{
  if (pose_valid_) {
    origin_axes_->getSceneNode()->setVisible(true);
  }
  bool draw_edge_lines = edge_show_property_->getBool();
  bool draw_edge_arrows = (draw_edge_lines && (edge_arrow_property_->getFloat() > 0.));
  bool draw_path_lines = path_show_property_->getBool();
  for (auto &[id, edge] : edges_) {
    edge.line->setVisible(draw_edge_lines);
    edge.arrowL->setVisible(draw_edge_arrows);
    edge.arrowR->setVisible(draw_edge_arrows);
    for (auto & line : edge.path) {
      line->setVisible(draw_path_lines);
    }
  }

  Ogre::ColourValue color_node = node_color_property_->getOgreColor();
  color_node.a = (node_show_property_->getBool() ? node_alpha_property_->getFloat() : 0.);
  for (auto & [id, node] : nodes_) {
    node.shape->setColor(color_node);
  }
}

void GraphDisplay::processMessage(tuw_graph_msgs::msg::Graph::ConstSharedPtr message)
{
  static tuw_graph_msgs::msg::Graph msg;
  /// Prevent double processing of same messages
  if (msg == *message) {
    return;
  }
  msg = *message;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(msg.header, msg.origin, position, orientation)) {
    setMissingTransformToFixedFrame(msg.header.frame_id);
    return;
  }
  setTransformOk();
  pose_valid_ = true;

  edges_.clear();
  nodes_.clear();
  Ogre::ColourValue color_node = node_color_property_->getOgreColor();
  color_node.a = node_alpha_property_->getFloat();
  Ogre::Vector3 size_node(
    node_size_property_->getFloat(), node_size_property_->getFloat(),
    node_size_property_->getFloat());
  for (const auto & node : msg.nodes) {
    const geometry_msgs::msg::Point & p = node.pose.position;
    NodeDisplay node_display;
    node_display.shape = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Cube, scene_manager_, scene_node_);
    node_display.shape->setColor(color_node);
    node_display.shape->setScale(size_node);
    node_display.shape->setPosition(Ogre::Vector3(p.x, p.y, p.z));
    nodes_.insert(std::make_pair(node.id, std::move(node_display)));
  }

  Ogre::ColourValue color_edge = edge_color_property_->getOgreColor();
  color_edge.a = edge_alpha_property_->getFloat();
  Ogre::Vector3 start, end, diff, unit, shaft, offsetL, offsetR, startL, startR;
  for (const auto & edge : msg.edges) {
    if (auto it = nodes_.find(edge.start); it == nodes_.end()) {
      continue;
    } else {
      start = it->second.shape->getPosition();
    }
    if (auto it = nodes_.find(edge.end); it == nodes_.end()) {
      continue;
    } else {
      end = it->second.shape->getPosition();
    }
    EdgeDisplay edge_display;
    edge_display.line = std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_);
    edge_display.line->setColor(color_edge);
    edge_display.line->setPoints(start, end);
    unit = end - start;
    unit.normalise();
    if (edge_arrow_property_->getFloat() > 0.) {
      shaft = unit * edge_arrow_property_->getFloat();
      double a = 20. * M_PI / 180.;
      offsetL = Ogre::Vector3(
        shaft.x * cos(+a) + shaft.y * -sin(+a), shaft.x * sin(+a) + shaft.y * cos(+a), shaft.z);
      offsetR = Ogre::Vector3(
        shaft.x * cos(-a) + shaft.y * -sin(-a), shaft.x * sin(-a) + shaft.y * cos(-a), shaft.z);
      edge_display.arrowL = std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_);
      edge_display.arrowL->setColor(color_edge);
      startL = end - offsetL;
      edge_display.arrowL->setPoints(startL, end);
      edge_display.arrowR = std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_);
      edge_display.arrowR->setColor(color_edge);
      startR = end - offsetR;
      edge_display.arrowR->setPoints(startR, end);
    }

    {
      /// the path starts with the start node and ends with the end node.
      /// edge.start -> ... edge.path ... -> edge.end
      Ogre::ColourValue color_path = path_color_property_->getOgreColor();
      color_path.a = path_alpha_property_->getFloat();
      Ogre::Vector3 p0 = start, p1;
      for (const auto & pose : edge.path) {
        const geometry_msgs::msg::Point & p = pose.position;
        p1 = Ogre::Vector3(p.x, p.y, p.z);
        auto line = std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_);
        line->setPoints(p0, p1);
        line->setColor(color_path);
        edge_display.path.push_back(std::move(line));
        p0 = p1;
      }
      auto line = std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_);
      line->setPoints(p0, end);
      line->setColor(color_path);
      edge_display.path.push_back(std::move(line));
    }

    edges_.insert(std::make_pair(edge.id, std::move(edge_display)));
  }

  coll_handler_->setMessage(message);
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
