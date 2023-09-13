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

      edge_alpha_property_ = new rviz_common::properties::FloatProperty(
          "Edges alpha", 1, "Amount of transparency to apply to the edges.",
          this, SLOT(updateVerticesGeometry()));
      edge_alpha_property_->setMin(0);
      edge_alpha_property_->setMax(1);

      edge_width_property_ = new rviz_common::properties::FloatProperty(
          "Edges width", 0.05f, "Edges path width",
          this, SLOT(updateVerticesGeometry()));

      edge_color_property_ = new rviz_common::properties::ColorProperty(
          "Edges color", QColor(0, 255, 255), "Color to draw path",
          this, SLOT(updateVerticesGeometry()));

      node_alpha_property_ = new rviz_common::properties::FloatProperty(
          "Nodes alpha", 1, "Amount of transparency to apply to the nodes.",
          this, SLOT(updateVerticesGeometry()));
      node_alpha_property_->setMin(0);
      node_alpha_property_->setMax(1);

      node_size_property_ = new rviz_common::properties::FloatProperty(
          "Nodes size", 0.05f, "Node size.",
          this, SLOT(updateVerticesGeometry()));

      node_color_property_ = new rviz_common::properties::ColorProperty(
          "Nodes color", QColor(255, 0, 255), "Color to draw nodes",
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

    void GraphDisplay::updateVerticesGeometry()
    {
      float width = edge_width_property_->getFloat();
      Ogre::ColourValue color = edge_color_property_->getOgreColor();
      color.a = edge_alpha_property_->getFloat();
      for (auto &path : paths_)
      {
        path->setColor(color.r, color.g, color.b, color.a);
        path->setLineWidth(width);
      }
      context_->queueRender();
    }

    void GraphDisplay::updateShapeVisibility()
    {
      if (pose_valid_)
      {
        origin_axes_->getSceneNode()->setVisible(true);
        for(auto &path: paths_) path->getSceneNode()->setVisible(true);
        //for(auto &node: nodes_) node->getSceneNode()->setVisible(true);
      }
    }

    void GraphDisplay::processMessage(tuw_graph_msgs::msg::Graph::ConstSharedPtr message)
    {

      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      setTransformOk();

      pose_valid_ = true;
      if(false){
        float width = edge_width_property_->getFloat();
        Ogre::ColourValue color = edge_color_property_->getOgreColor();
        color.a = edge_alpha_property_->getFloat();
        for (size_t i = 0; i < message->edges.size(); i++)
        {
          if (paths_.size() <= i)
          {
            /// create vertices
            paths_.push_back(std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_));
            paths_.back()->setColor(color.r, color.g, color.b, color.a);
            paths_.back()->setLineWidth(width);
          }
          paths_[i]->clear();
          for (size_t j = 0; j < message->edges[i].path.size(); j++)
          {
            const geometry_msgs::msg::Point &p = message->edges[i].path[j];
            paths_[i]->addPoint(Ogre::Vector3(p.x, p.y, p.z));
          }
        }
        if(paths_.size() > message->edges.size()) paths_.resize(message->edges.size());
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

  } // namespace displays
} // namespace tuw_graph_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(tuw_graph_rviz_plugins::displays::GraphDisplay, rviz_common::Display)