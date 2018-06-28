#ifndef OBJECTDETECTIONDOORVISUAL_H
#define OBJECTDETECTIONDOORVISUAL_H

#include <ObjectDetection/ObjectDetectionVisual.h>
#include <ObjectDetection/DoorVisual.h>

namespace tuw_object_rviz
{

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
