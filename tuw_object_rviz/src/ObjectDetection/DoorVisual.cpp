/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rviz/mesh_loader.h>
#include <ros/console.h>
#include <ros/package.h>

#include <OgreSceneManager.h>
#include <OgreSubEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreAnimation.h>

#include "ObjectDetection/DoorVisual.h"


namespace fs = boost::filesystem;

namespace tuw_object_rviz {

/*
 * Generic Person Visual
 */
DoorVisual::DoorVisual(const DoorVisualDefaultArgs& args) :
        m_sceneManager(args.sceneManager)
{
    m_parentSceneNode = args.parentNode;
    m_sceneNode = args.parentNode->createChildSceneNode();

    Ogre::Vector3 scale(1,1,1);
    m_sceneNode->setScale(scale);
    R_do = Ogre::Matrix3(1,0,0,
                         0,1,0,
                         0,0,1);
}

DoorVisual::~DoorVisual() {
    m_sceneManager->destroySceneNode(m_sceneNode->getName());
};

void DoorVisual::setPosition(const Ogre::Vector3& position) {
    m_sceneNode->setPosition(position);
}

const Ogre::Vector3& DoorVisual::getPosition() const {
    return m_sceneNode->getPosition();
}

void DoorVisual::setOrientation(const Ogre::Quaternion& orientation) {
    m_sceneNode->setOrientation(orientation);
}

const Ogre::Quaternion& DoorVisual::getOrientation() const {
    return m_sceneNode->getOrientation();
}

void DoorVisual::setScalingFactor(double scalingFactor) {
    m_sceneNode->setScale(scalingFactor, scalingFactor, scalingFactor);
}

void DoorVisual::setVisible(bool visible) {
    m_sceneNode->setVisible(visible, true);
}

Ogre::SceneNode* DoorVisual::getParentSceneNode() {
    return m_parentSceneNode;
}

void DoorVisual::setHeight(double height)
{
  m_height = height;
}

void DoorVisual::setWidth(double width)
{
  m_width = width;
}

void DoorVisual::setOpeningAngle(double oangle, bool clockwise)
{
  m_oangle = oangle;
  m_clockwise = clockwise;
  double c;
  double s;
  if (clockwise) {
    c = cos(-oangle);
    s = sin(-oangle);
  } else {
    c = cos(oangle);
    s = sin(oangle);
  }
  R_do = Ogre::Matrix3(c,-s,0,
                       s,c,0,
                       0,0,1);
}

Ogre::Matrix3 DoorVisual::getRotationMat()
{
  return R_do;
}

void DoorVisual::update(float deltaTime) {}

/*
 * CylinderDoorVisual
 */
//CylinderDoorVisual::CylinderDoorVisual(const DoorVisualDefaultArgs& args) : DoorVisual(args)
//{
//    m_bodyShape = new rviz::Shape(rviz::Shape::Cylinder, args.sceneManager, m_sceneNode);
//    m_headShape = new rviz::Shape(rviz::Shape::Sphere, args.sceneManager, m_sceneNode);

//    const float headDiameter = 0.4;
//    const float totalHeight = getHeight();
//    const float cylinderHeight = totalHeight - headDiameter;

//    m_bodyShape->setScale(Ogre::Vector3(headDiameter, headDiameter, cylinderHeight));
//    m_headShape->setScale(Ogre::Vector3(headDiameter, headDiameter, headDiameter));

//    m_bodyShape->setPosition(Ogre::Vector3(0, 0, cylinderHeight / 2 - totalHeight / 2));
//    m_headShape->setPosition(Ogre::Vector3(0, 0, totalHeight / 2 - headDiameter / 2 ));
//}

//CylinderDoorVisual::~CylinderDoorVisual() {
//    delete m_bodyShape;
//    delete m_headShape;
//}

//void CylinderDoorVisual::setColor(const Ogre::ColourValue& c) {
//    m_bodyShape->setColor(c);
//    m_headShape->setColor(c);
//    m_color = c;
//}

//Ogre::ColourValue& CylinderDoorVisual::getColor()
//{
//  return m_color;
//}

//double CylinderDoorVisual::getHeight() {
//    return 1.75;
//}

/*
 * Bounding Box Visual
 */

BoundingBoxDoorVisual::BoundingBoxDoorVisual ( const DoorVisualDefaultArgs& args, double height, double width, double scalingFactor ) : DoorVisual(args)
{
    m_width = width; m_height = height; m_scalingFactor = scalingFactor; m_lineWidth = 0.03;
    m_wireframe = NULL;
    m_baseframe = NULL;
    m_thickness = 0.1;
    generateWireframe();
}

BoundingBoxDoorVisual::~BoundingBoxDoorVisual() {
    delete m_wireframe;
    delete m_baseframe;
    m_wireframe = NULL;
    m_baseframe = NULL;
}

void BoundingBoxDoorVisual::setColor(const Ogre::ColourValue& c) {
    m_wireframe->setColor(c.r, c.g, c.b, c.a);
    m_color = c;
}

Ogre::ColourValue& BoundingBoxDoorVisual::getColor()
{
  return m_color;
}

double BoundingBoxDoorVisual::getHeight() {
    return m_height;
}

void BoundingBoxDoorVisual::setLineWidth(double lineWidth) {
    m_wireframe->setLineWidth(lineWidth);
}

void BoundingBoxDoorVisual::generateWireframe() {
    delete m_wireframe;
    m_wireframe = new rviz::BillboardLine(m_sceneManager, m_sceneNode);

    m_wireframe->setLineWidth(m_lineWidth);
    m_wireframe->setMaxPointsPerLine(2);
    m_wireframe->setNumLines(12);

    double w = m_width, h = m_height;
    Ogre::Vector3 bottomLeft(0, -w, 0), bottomRight(0, 0, 0), topLeft(0, -w, h), topRight(0, 0, h);
    Ogre::Vector3 rear(m_thickness, 0, 0);

    // Front quad
                                m_wireframe->addPoint(bottomLeft);          m_wireframe->addPoint(bottomRight);
    m_wireframe->newLine();     m_wireframe->addPoint(bottomRight);         m_wireframe->addPoint(topRight);
    m_wireframe->newLine();     m_wireframe->addPoint(topRight);            m_wireframe->addPoint(topLeft);
    m_wireframe->newLine();     m_wireframe->addPoint(topLeft);             m_wireframe->addPoint(bottomLeft);

    // Rear quad
    m_wireframe->newLine();     m_wireframe->addPoint(bottomLeft + rear);   m_wireframe->addPoint(bottomRight + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(bottomRight + rear);  m_wireframe->addPoint(topRight + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(topRight + rear);     m_wireframe->addPoint(topLeft + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(topLeft + rear);      m_wireframe->addPoint(bottomLeft + rear);

    // Four connecting lines between front and rear
    m_wireframe->newLine();     m_wireframe->addPoint(bottomLeft);          m_wireframe->addPoint(bottomLeft + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(bottomRight);         m_wireframe->addPoint(bottomRight + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(topRight);            m_wireframe->addPoint(topRight + rear);
    m_wireframe->newLine();     m_wireframe->addPoint(topLeft);             m_wireframe->addPoint(topLeft + rear);

    m_wireframe->setPosition(Ogre::Vector3(0, 0, 0));
//    Ogre::Quaternion orientation;
//    orientation.FromRotationMatrix(R_do);
//    m_wireframe->setOrientation(orientation);
}

void BoundingBoxDoorVisual::generateBaseframe() {
    delete m_baseframe;

    m_baseframe = new rviz::BillboardLine(m_sceneManager, m_sceneNode);
    m_baseframe->setLineWidth(m_lineWidth);
    m_baseframe->setMaxPointsPerLine(2);
    m_baseframe->setPosition(Ogre::Vector3(0,0,0));
    Ogre::Vector3 offset = Ogre::Vector3(0,-m_width,0);
    double factor = 5.0 * M_PI/180.0;
    double lines_per_angle = ceil(m_oangle / factor);
    double angle_dec = 0;
    m_baseframe->setNumLines(static_cast<int>(lines_per_angle));
    int sign = m_clockwise == true ? 1 : -1;
    for (int i=0; i < lines_per_angle; i++) {
      Ogre::Vector3 start_v = offset;
      Ogre::Vector3 end_v = offset;
      double s_theta = sin(sign * angle_dec);
      double c_theta = cos(sign * angle_dec);
      Ogre::Matrix3 R_start = Ogre::Matrix3(c_theta,-s_theta,0,
                    s_theta,c_theta,0,
                    0,  0,  1);
      angle_dec += factor;
      s_theta = sin(sign * angle_dec);
      c_theta = cos(sign * angle_dec);
      Ogre::Matrix3 R_end = Ogre::Matrix3(c_theta,-s_theta,0,
                    s_theta,c_theta,0,
                    0,  0,  1);
      start_v = R_start * start_v;
      end_v = R_end * end_v;
      m_baseframe->addPoint(start_v);
      m_baseframe->addPoint(end_v);
      m_baseframe->newLine();
    }
}

/*
 * Mesh Person Visual
 */

//MeshDoorVisual::MeshDoorVisual(const DoorVisualDefaultArgs& args) : DoorVisual(args), entity_(NULL), m_animationState(NULL), m_walkingSpeed(1.0)
//{
//    m_childSceneNode = m_sceneNode->createChildSceneNode();
//    m_childSceneNode->setVisible(false);

//    std::string meshResource = "package://" ROS_PACKAGE_NAME "/media/animated_walking_man.mesh";

//    /// This is required to load referenced skeletons from package:// path
//    fs::path model_path(meshResource);
//    fs::path parent_path(model_path.parent_path());

//    Ogre::ResourceLoadingListener *newListener = new RosPackagePathResourceLoadingListener(parent_path),
//                                  *oldListener = Ogre::ResourceGroupManager::getSingleton().getLoadingListener();

//    Ogre::ResourceGroupManager::getSingleton().setLoadingListener(newListener);
//    bool loadFailed = rviz::loadMeshFromResource(meshResource).isNull();
//    Ogre::ResourceGroupManager::getSingleton().setLoadingListener(oldListener);

//    delete newListener;


//    // Create scene entity
//    static size_t count = 0;
//    std::stringstream ss;
//    ss << "mesh_person_visual" << count++;
//    std::string id = ss.str();

//    entity_ = m_sceneManager->createEntity(id, meshResource);
//    m_childSceneNode->attachObject(entity_);

//    // set up animation
//    setAnimationState("");

//    // set up material
//    ss << "Material";
//    Ogre::MaterialPtr default_material = Ogre::MaterialManager::getSingleton().create( ss.str(), "rviz" );
//    default_material->setReceiveShadows(false);
//    default_material->getTechnique(0)->setLightingEnabled(true);
//    default_material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );
//    materials_.insert( default_material );
//    entity_->setMaterial( default_material );

//    // set position
//    Ogre::Quaternion quat1; quat1.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3(0,1,0));
//    Ogre::Quaternion quat2; quat2.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3(0,0,1));
//    m_childSceneNode->setOrientation(quat1 * quat2);

//    double scaleFactor = 0.243 * 1.75;
//    m_childSceneNode->setScale(Ogre::Vector3(scaleFactor, scaleFactor, scaleFactor));
//    m_childSceneNode->setPosition(Ogre::Vector3(0, 0, -1));

//    m_childSceneNode->setVisible(true);
//}

//MeshDoorVisual::~MeshDoorVisual() {
//    m_sceneManager->destroyEntity( entity_ );

//    // destroy all the materials we've created
//    std::set<Ogre::MaterialPtr>::iterator it;
//    for ( it = materials_.begin(); it!=materials_.end(); it++ )
//    {
//        Ogre::MaterialPtr material = *it;
//        if (!material.isNull())
//        {
//          material->unload();
//          Ogre::MaterialManager::getSingleton().remove(material->getName());
//        }
//    }
//    materials_.clear();

//    m_sceneManager->destroySceneNode(m_childSceneNode->getName());
//}

//void MeshDoorVisual::setColor(const Ogre::ColourValue& c) {

//    m_color = c;

//    Ogre::SceneBlendType blending;
//    bool depth_write;

//    if ( c.a < 0.9998 )
//    {
//      blending = Ogre::SBT_TRANSPARENT_ALPHA;
//      depth_write = false;
//    }
//    else
//    {
//      blending = Ogre::SBT_REPLACE;
//      depth_write = true;
//    }

//    std::set<Ogre::MaterialPtr>::iterator it;
//    for( it = materials_.begin(); it != materials_.end(); it++ )
//    {
//      Ogre::Technique* technique = (*it)->getTechnique( 0 );

//      technique->setAmbient( c.r*0.5, c.g*0.5, c.b*0.5 );
//      technique->setDiffuse( c.r, c.g, c.b, c.a );
//      technique->setSceneBlending( blending );
//      technique->setDepthWriteEnabled( depth_write );
//      technique->setLightingEnabled( true );
//    }
//}

//Ogre::ColourValue& MeshDoorVisual::getColor()
//{
//  return m_color;
//}

//void MeshDoorVisual::setAnimationState(const std::string& nameOfAnimationState) {
//    Ogre::AnimationStateSet *animationStates = entity_->getAllAnimationStates();
//    if(animationStates != NULL)
//    {
//      Ogre::AnimationStateIterator animationsIterator = animationStates->getAnimationStateIterator();
//      while (animationsIterator.hasMoreElements())
//      {
//        Ogre::AnimationState *animationState = animationsIterator.getNext();
//        if(animationState->getAnimationName() == nameOfAnimationState || nameOfAnimationState.empty()) {
//          animationState->setLoop(true);
//          animationState->setEnabled(true);
//          m_animationState = animationState;
//          return;
//        }
//      }

//      // Not found. Set first animation state then.
//      ROS_WARN_STREAM_ONCE("Person mesh animation state " << nameOfAnimationState << " does not exist in mesh!");
//      setAnimationState("");
//    }
//}

//void MeshDoorVisual::setWalkingSpeed(float walkingSpeed) {
//    m_walkingSpeed = walkingSpeed;
//}


//void MeshDoorVisual::update(float deltaTime) {
//    if(m_animationState) {
//        m_animationState->addTime(0.7 * deltaTime * m_walkingSpeed);
//    }
//}


} // end of namespace spencer_tracking_rviz_plugin
