#ifndef TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
#define TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_

#include <memory>

#include "tuw_graph_msgs/msg/graph.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/interaction/forwards.hpp"


#include "tuw_graph_rviz_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class BillboardLine;
class Axes;
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
class TUW_GRAPH_RVIZ_PLUGINS_PUBLIC GraphDisplay : public
  rviz_common::MessageFilterDisplay<tuw_graph_msgs::msg::Graph>
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
  void updateShapeVisibility();
  void updateAxisGeometry();
  void updateVerticesGeometry();

private:
  void setupSelectionHandler();


  std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> paths_;
  std::unique_ptr<rviz_rendering::Axes> origin_axes_;
  bool pose_valid_;
  GraphDisplaySelectionHandlerPtr coll_handler_;

  rviz_common::properties::ColorProperty * vertices_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * vertices_width_property_;

  rviz_common::properties::FloatProperty * origin_axes_length_property_;
  rviz_common::properties::FloatProperty * origin_axes_radius_property_;

  friend class GraphDisplaySelectionHandler;
};

}  // namespace displays
}  // namespace tuw_graph_rviz_plugins

#endif  // TUW_GRAPH_RVIZ_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_