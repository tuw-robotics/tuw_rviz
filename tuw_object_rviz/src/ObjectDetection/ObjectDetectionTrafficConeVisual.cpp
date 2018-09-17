/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ObjectDetection/ObjectDetectionTrafficConeVisual.h>

namespace tuw_object_rviz
{
ObjectDetectionTrafficConeVisual::ObjectDetectionTrafficConeVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) : ObjectDetectionVisual(scene_manager, parent_node)
{
  traffic_cone_visual_.reset(new TrafficConeVisualImpl(
                       TrafficConeVisualDefaultArgs(scene_manager_, frame_node_)));
}

ObjectDetectionTrafficConeVisual::~ObjectDetectionTrafficConeVisual()
{
  // empty destructor since parent destructor destroys frame node anyway
}

void ObjectDetectionTrafficConeVisual::setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg)
{
  // call parent class set Message to also display covariance, center, ids
  ObjectDetectionVisual::setMessage(msg);

  Ogre::Vector3 position = Ogre::Vector3(msg->object.pose.position.x, msg->object.pose.position.y, msg->object.pose.position.z);
  position = transform_ * position;
  position.z = 0;  // fix on ground z=0

  double c_90 = cos(M_PI/2.0);
  double s_90 = sin(M_PI/2.0);
  Ogre::Matrix3 R_x(1,0,0,
                    0,c_90,-s_90,
                    0,s_90,c_90);
  Ogre::Quaternion q_flip;
  q_flip.FromRotationMatrix(R_x);

  Ogre::Quaternion orientation = Ogre::Quaternion(msg->object.pose.orientation.w, msg->object.pose.orientation.x,
                                                  msg->object.pose.orientation.y, msg->object.pose.orientation.z);
  orientation = q_flip * orientation;
  double radius = msg->object.shape_variables[0];
  double height = msg->object.shape_variables[1];
  int color = (int) msg->object.shape_variables[2];
  traffic_cone_visual_->setRadius(radius);
  traffic_cone_visual_->setHeight(height);
  Ogre::ColourValue c(0,0,0,1.0);
  if (color == 1) // blue
  {
    c.b = 1.0;
  }
  else if (color == 2) // yellow
  {
    c.r = 1.0;
    c.g = 1.0;
  }
  else if (color == 3)
  {
    c.r = 1.0;
  }

  traffic_cone_visual_->setColor(c);
  traffic_cone_visual_->setPosition(position + Ogre::Vector3(0, 0, height/2.0));
  traffic_cone_visual_->setOrientation(orientation);
  traffic_cone_visual_->update(0.0);
}

// Color is passed through to the pose Shape object.
void ObjectDetectionTrafficConeVisual::setColor(Ogre::ColourValue color)
{
  ObjectDetectionVisual::setColor(color);
  //traffic_cone_visual_->setColor(color);
}

void ObjectDetectionTrafficConeVisual::setStyle(Styles style)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  Ogre::ColourValue color;
  TrafficConeVisualDefaultArgs default_args(scene_manager_, frame_node_);

  position = traffic_cone_visual_->getPosition();
  orientation = traffic_cone_visual_->getOrientation();
  color = traffic_cone_visual_->getColor();

  switch (style)
  {
    case STYLE_BOUNDING_BOXES:
      traffic_cone_visual_.reset(new TrafficConeVisualImpl(default_args));
      break;
    default:
      break;
  }

  traffic_cone_visual_->setOrientation(orientation);
  traffic_cone_visual_->setPosition(position);
  traffic_cone_visual_->setColor(color);

}

}  // end namespace tuw_object_rviz
