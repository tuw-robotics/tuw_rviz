#pragma once

#include <deque>
#include <memory>

#include <QtCore>

#include "tuw_object_map_msgs/msg/object_map.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
    class Shape;
}

namespace tuw_object_map_rviz_plugins::displays
{
    // TODO: Replace RosTopicDisplay with MessageFilterDisplay
    class ObjectMapDisplay : public rviz_common::RosTopicDisplay<tuw_object_map_msgs::msg::ObjectMap>
    {
        Q_OBJECT

    public:
        ObjectMapDisplay();

        ~ObjectMapDisplay();

        void processMessage(const tuw_object_map_msgs::msg::ObjectMap::ConstSharedPtr msg) override;
        void drawPlantWineRow(const tuw_object_map_msgs::msg::Object &msg);
        void drawTransitGravel(const tuw_object_map_msgs::msg::Object &msg);

        const tuw_object_map_msgs::msg::Object translateToOrigin(const tuw_object_map_msgs::msg::Object &msg);
        void orientShape(std::shared_ptr<rviz_rendering::Shape> shape, const Ogre::Vector3 &start, const Ogre::Vector3 &end);
    protected:
        void reset();

    private:
        Ogre::Vector3 origin;
        std::deque<std::shared_ptr<rviz_rendering::Shape>> visuals_;
    };
}
