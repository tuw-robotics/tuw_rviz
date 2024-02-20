#include "tuw_object_map_rviz_plugins/object_map_display.hpp"

#include <memory>
#include <string>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/validate_floats.hpp"

namespace tuw_object_map_rviz_plugins::displays
{
    ObjectMapDisplay::ObjectMapDisplay()
    {
        this->origin = Ogre::Vector3(563938, 5183454, 296);
    }

    ObjectMapDisplay::~ObjectMapDisplay() = default;

    void ObjectMapDisplay::reset()
    {
        visuals_.clear();
    }

    void ObjectMapDisplay::processMessage(tuw_object_map_msgs::msg::ObjectMap::ConstSharedPtr msg)
    {
        visuals_.clear();

        for (const auto &object : msg->objects)
        {
            // TODO: Properly validate floats

            switch (object.type)
            {
            case tuw_object_map_msgs::msg::Object::TYPE_PLANT_WINE_ROW:
                drawPlantWineRow(object);
                break;
            case tuw_object_map_msgs::msg::Object::TYPE_TRANSIT_GRAVEL:
                drawTransitGravel(object);
                break;

            default:
                std::cout << "Message " << object.id << " contained contained unknown object type " << object.type << std::endl;
                setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString("Message contained unknown object type: %1").arg(object.type));
                break;
            }
        }
    }

    void ObjectMapDisplay::drawPlantWineRow(const tuw_object_map_msgs::msg::Object &msg)
    {
        const tuw_object_map_msgs::msg::Object translated = this->translateToOrigin(msg);

        // Error Checking
        if (translated.map_points.size() < 2)
        {
            std::cout << "Message " << msg.id << " contained too little map points" << std::endl;
            setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString("Message %1 contained too little map points").arg(msg.id));
            return;
        }

        // Actual drawing
        std::shared_ptr<rviz_rendering::Shape> visual = std::make_shared<rviz_rendering::Shape>(
            rviz_rendering::Shape::Cube, context_->getSceneManager(), scene_node_);

        auto rowStart = rviz_common::pointMsgToOgre(translated.map_points[0]);
        auto rowEnd = rviz_common::pointMsgToOgre(translated.map_points[1]);
        auto middle = (rowStart + rowEnd) / 2;
        auto direction = rowEnd - rowStart;

        visual->setPosition(middle);
        this->orientShape(visual, rowStart, rowEnd);
        visual->setScale(Ogre::Vector3(direction.length(), translated.bondary_radius[0], 0.1));
        visual->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        visuals_.push_back(visual);
    }

    void ObjectMapDisplay::drawTransitGravel(const tuw_object_map_msgs::msg::Object &msg)
    {
        const tuw_object_map_msgs::msg::Object translated = this->translateToOrigin(msg);

        // Error Checking
        if (translated.map_points.size() < 2)
        {
            std::cout << "Message " << msg.id << " contained too little map points" << std::endl;
            setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", QString("Message %1 contained too little map points").arg(msg.id));
            return;
        }

        // Actual drawing
        for (size_t i = 1; i < translated.map_points.size(); i++)
        {
            std::shared_ptr<rviz_rendering::Shape> visual = std::make_shared<rviz_rendering::Shape>(
                rviz_rendering::Shape::Cube, context_->getSceneManager(), scene_node_);

            auto rowStart = rviz_common::pointMsgToOgre(translated.map_points[i - 1]);
            auto rowEnd = rviz_common::pointMsgToOgre(translated.map_points[i]);
            auto middle = (rowStart + rowEnd) / 2;
            auto direction = rowEnd - rowStart;

            visual->setPosition(middle);
            this->orientShape(visual, rowStart, rowEnd);
            visual->setScale(Ogre::Vector3(direction.length(), translated.bondary_radius[i], 0.1));
            visual->setColor(0.1f, 0.0f, 0.0f, 1.0f);
            visuals_.push_back(visual);
        }
    }

    void ObjectMapDisplay::orientShape(std::shared_ptr<rviz_rendering::Shape> shape, const Ogre::Vector3 &start, const Ogre::Vector3 &end)
    {
        auto direction = end - start;

        Ogre::Quaternion yaw = Ogre::Quaternion::IDENTITY;
        yaw.FromAngleAxis(Ogre::Radian(atan2(direction.y, direction.x)), Ogre::Vector3::UNIT_Z);
        Ogre::Quaternion pitch = Ogre::Quaternion::IDENTITY;
        pitch.FromAngleAxis(Ogre::Radian(atan2(direction.z, direction.x)), Ogre::Vector3::UNIT_Y);
        Ogre::Quaternion orientation = yaw * pitch;

        shape->setOrientation(orientation);
    }

    const tuw_object_map_msgs::msg::Object ObjectMapDisplay::translateToOrigin(const tuw_object_map_msgs::msg::Object &msg)
    {
        tuw_object_map_msgs::msg::Object translated = msg;

        for (auto &point : translated.map_points)
        {
            point.x -= this->origin.x;
            point.y -= this->origin.y;
            point.z -= this->origin.z;
        }

        return translated;
    }
}

#include <pluginlib/class_list_macros.hpp> // NOLINT

PLUGINLIB_EXPORT_CLASS(tuw_object_map_rviz_plugins::displays::ObjectMapDisplay, rviz_common::Display)