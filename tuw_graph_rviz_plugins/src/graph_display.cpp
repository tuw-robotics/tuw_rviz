#include "tuw_graph_rviz_plugins/graph_display.hpp"

#include <memory>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/line.hpp"
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

      edge_alpha_property_ = new rviz_common::properties::FloatProperty(
          "Edges alpha", 1, "Amount of transparency to apply to the edges.",
          this, SLOT(updateEdgesGeometry()));
      edge_alpha_property_->setMin(0);
      edge_alpha_property_->setMax(1);

      edge_color_property_ = new rviz_common::properties::ColorProperty(
          "Edges color", QColor(0, 255, 255), "Color to draw path",
          this, SLOT(updateEdgesGeometry()));

      node_alpha_property_ = new rviz_common::properties::FloatProperty(
          "Nodes alpha", 1, "Amount of transparency to apply to the nodes.",
          this, SLOT(updateNodesGeometry()));
      node_alpha_property_->setMin(0);
      node_alpha_property_->setMax(1);

      node_size_property_ = new rviz_common::properties::FloatProperty(
          "Nodes size", 0.15f, "Node size.",
          this, SLOT(updateNodesGeometry()));

      node_color_property_ = new rviz_common::properties::ColorProperty(
          "Nodes color", QColor(255, 0, 255), "Color to draw nodes",
          this, SLOT(updateNodesGeometry()));
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
      coll_handler_ = rviz_common::interaction::createSelectionHandler<GraphDisplaySelectionHandler>(this, context_);
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

    void GraphDisplay::updateNodesGeometry()
    {
      float size = node_size_property_->getFloat();
      Ogre::ColourValue color = node_color_property_->getOgreColor();
      color.a = node_alpha_property_->getFloat();
      for (auto &node : nodes_)
      {
        node->setColor(color.r, color.g, color.b, color.a);
        node->setScale(Ogre::Vector3(size, size, size));
      }
      context_->queueRender();
    }

    void GraphDisplay::updateEdgesGeometry()
    {
      Ogre::ColourValue color = edge_color_property_->getOgreColor();
      color.a = edge_alpha_property_->getFloat();
      for (auto &edge : path_)
      {
        edge->setColor(color.r, color.g, color.b, color.a);
      }
      context_->queueRender();
    }

    void GraphDisplay::updateShapeVisibility()
    {
      if (pose_valid_)
      {
        origin_axes_->getSceneNode()->setVisible(true);
      }
    }

    void GraphDisplay::processMessage(tuw_graph_msgs::msg::Graph::ConstSharedPtr message)
    {

      //Ogre::Vector3 position(0., 0., 0.);
      //Ogre::Quaternion orientation(0., 0., 0., 1.);
      setTransformOk();

      if(true){
        Ogre::ColourValue color = edge_color_property_->getOgreColor();
        color.a = edge_alpha_property_->getFloat();
        size_t count_lines = 0;
        geometry_msgs::msg::Point start, end;
        for (size_t i = 0; i < message->edges.size(); i++)
        {
          const auto &edge = message->edges[i]; 
          if(edge.nodes.size() != 2) RCUTILS_LOG_INFO("An edge needs two nodes!");
            for(auto &node: message->nodes) {
              if (node.id == edge.nodes[0]) {
                start = node.position;
                break;
              }
            }; 

            for (size_t j = 0; j < edge.path.size(); j++){
                if(path_.size() <= count_lines){
                  path_.push_back(std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_));
                  path_.back()->setColor(color);
                }
                end = edge.path[j];
                path_[count_lines]->setPoints(Ogre::Vector3(start.x, start.y, start.z), Ogre::Vector3(end.x, end.y, end.z));
                count_lines++;
                start = end;
            }
            for(auto &node: message->nodes) {
              if (node.id == edge.nodes[1]) {
                end = node.position;
                break;
              }
            }; 
            if(path_.size() <= count_lines){
              path_.push_back(std::make_unique<rviz_rendering::Line>(scene_manager_, scene_node_));
              path_.back()->setColor(color);
            }
            path_[count_lines]->setPoints(Ogre::Vector3(start.x, start.y, start.z), Ogre::Vector3(end.x, end.y, end.z));
            count_lines++;
        }
        if(path_.size() > count_lines) path_.resize(count_lines);
      }
      if(true){
        float size = node_size_property_->getFloat();
        Ogre::ColourValue color = node_color_property_->getOgreColor();
        for (size_t i = 0; i < message->nodes.size(); i++)
        {
          if (nodes_.size() <= i)
          {
            /// create vertices
            nodes_.push_back(std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_));
            nodes_.back()->setColor(color.r, color.g, color.b, color.a);
            nodes_.back()->setScale(Ogre::Vector3(size, size, size));
          }
          const geometry_msgs::msg::Point &p =  message->nodes[i].position;
          nodes_[i]->setPosition(Ogre::Vector3(p.x, p.y, p.z));
        }
        if(nodes_.size() > message->nodes.size()) nodes_.resize(message->nodes.size());
      }

      pose_valid_ = true;
      updateShapeVisibility();

      //scene_node_->setPosition(position);
      //scene_node_->setOrientation(orientation);
      //scene_node_->setVisible(true);

      coll_handler_->setMessage(message);

      context_->queueRender();
    }

    void GraphDisplay::reset()
    {
      MFDClass::reset();
      pose_valid_ = false;
      updateShapeVisibility();
    }

  } // namespace displays
} // namespace tuw_graph_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(tuw_graph_rviz_plugins::displays::GraphDisplay, rviz_common::Display)