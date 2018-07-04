#ifndef OBJECTDETECTIONTrafficConeVISUAL_H
#define OBJECTDETECTIONTrafficConeVISUAL_H

#include <ObjectDetection/ObjectDetectionVisual.h>
#include <ObjectDetection/TrafficConeVisual.h>

namespace tuw_object_rviz
{

class ObjectDetectionTrafficConeVisual : public ObjectDetectionVisual
{
public:
     ObjectDetectionTrafficConeVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
     ~ObjectDetectionTrafficConeVisual ();
     void setMessage ( const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg );
     void setColor ( Ogre::ColourValue color );
     void setStyle ( Styles style );
private:
    boost::shared_ptr<TrafficConeVisual> traffic_cone_visual_;
};

} // end namespace tuw_object_rviz

#endif // OBJECTDETECTIONTrafficConeVISUAL_H
