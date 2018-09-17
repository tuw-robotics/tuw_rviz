#ifndef OBJECTDETECTIONDOORVISUAL_H
#define OBJECTDETECTIONDOORVISUAL_H

#include <ObjectDetection/ObjectDetectionVisual.h>
#include <ObjectDetection/DoorVisual.h>

namespace tuw_object_rviz
{

//TODO: redundancy (also defined in tuw_door_detection/tuw_object_publisher)
enum DoorObjectShapeVariables {
  width = 0,
  height = 1,
  angle_w = 2,
  angle_d = 3,
  leaves = 4,
  clockwise = 5
};

class ObjectDetectionDoorVisual : public ObjectDetectionVisual
{
public:
     ObjectDetectionDoorVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
     ~ObjectDetectionDoorVisual ();
     void setMessage ( const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg );
     void setColor ( Ogre::ColourValue color );
     void setStyle ( Styles style );
private:
    boost::shared_ptr<DoorVisual> door_visual_;
};

} // end namespace tuw_object_rviz

#endif // OBJECTDETECTIONDOORVISUAL_H
