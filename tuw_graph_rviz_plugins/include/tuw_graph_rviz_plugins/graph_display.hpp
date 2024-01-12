#ifndef TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
#define TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_

#include <map>
#include <memory>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "tuw_graph_msgs/msg/graph.hpp"
#include "tuw_graph_msgs/msg/state.hpp"
#include "tuw_graph_rviz_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class BillboardLine;
class Line;
class Axes;
class Shape;
class Arrow;
class MovableText;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace tuw_graph_rviz_plugins
{
namespace displays
{

class GraphDisplaySelectionHandler;

typedef std::shared_ptr<GraphDisplaySelectionHandler> GraphDisplaySelectionHandlerPtr;

/** @brief Accumulates and displays the pose from a tuw_graph_msgs::Graph message. */
class TUW_GRAPH_RVIZ_PLUGINS_PUBLIC GraphDisplay
: public rviz_common::MessageFilterDisplay<tuw_graph_msgs::msg::Graph>
{
  Q_OBJECT

public:
  GraphDisplay();

  ~GraphDisplay() override;
  void onInitialize() override;
  void reset() override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
  void onEnable() override;
  void onDisable() override;
  void processMessage(tuw_graph_msgs::msg::Graph::ConstSharedPtr message) override;

private Q_SLOTS:
  void updateAxisGeometry();
  void updateAxesVisibility();
  void updateEdgesGeometry();
  void updateEdgesColor();
  void updateEdgesVisibility();
  void updatePathsGeometry();
  void updatePathsColor();
  void updatePathsVisibility();
  void updateNodesGeometry();
  void updateNodesColor();
  void updateNodesVisibility();
  void updateNodesIdColor();
  void updateNodesIdVisibility();

private:
  void setupSelectionHandler();

    rclcpp::Subscription<tuw_graph_msgs::msg::State>::SharedPtr sub_graph_state_;

  struct EdgeDisplay{
    uint64_t id;
    std::unique_ptr<rviz_rendering::Line> line0;
    std::unique_ptr<rviz_rendering::Line> line1;
    std::unique_ptr<rviz_rendering::Line> arrowL;
    std::unique_ptr<rviz_rendering::Line> arrowR;
    std::vector<std::unique_ptr<rviz_rendering::Line>> path;
    Ogre::Vector3 start, end, midpoint_offset;
  };

  struct NodeDisplay{
    uint64_t id;
    std::unique_ptr<rviz_rendering::Shape> shape;
    std::unique_ptr<rviz_rendering::MovableText> text;
    Ogre::SceneNode * scene_node;
  };

  std::map<int64_t, NodeDisplay> nodes_;
  std::map<int64_t, EdgeDisplay> edges_;
  std::unique_ptr<rviz_rendering::Axes> origin_axes_;
  bool pose_valid_;
  GraphDisplaySelectionHandlerPtr coll_handler_;

  rviz_common::properties::BoolProperty * node_id_show_property_;
  rviz_common::properties::ColorProperty * node_id_color_property_;

  rviz_common::properties::BoolProperty * edge_show_property_;
  rviz_common::properties::FloatProperty * edge_overlay_offset_property_;
  rviz_common::properties::ColorProperty * edge_color_property_;
  rviz_common::properties::FloatProperty * edge_alpha_property_;
  rviz_common::properties::FloatProperty * edge_arrow_size_property_;

  rviz_common::properties::BoolProperty * path_show_property_;
  rviz_common::properties::ColorProperty * path_color_property_;
  rviz_common::properties::FloatProperty * path_alpha_property_;

  rviz_common::properties::BoolProperty * node_show_property_;
  rviz_common::properties::ColorProperty * node_color_property_;
  rviz_common::properties::FloatProperty * node_alpha_property_;
  rviz_common::properties::FloatProperty * node_size_property_;

  rviz_common::properties::FloatProperty * origin_axes_length_property_;
  rviz_common::properties::FloatProperty * origin_axes_radius_property_;

  friend class GraphDisplaySelectionHandler;
};

}  // namespace displays
}  // namespace tuw_graph_rviz_plugins

#endif  // TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
