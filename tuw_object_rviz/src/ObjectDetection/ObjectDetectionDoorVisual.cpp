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

#include <ObjectDetection/ObjectDetectionDoorVisual.h>

namespace tuw_object_rviz
{
ObjectDetectionDoorVisual::ObjectDetectionDoorVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) : ObjectDetectionVisual(scene_manager, parent_node)
{
  door_visual_.reset(new BoundingBoxDoorVisual(
                       DoorVisualDefaultArgs(scene_manager_, frame_node_)));
}

ObjectDetectionDoorVisual::~ObjectDetectionDoorVisual()
{
  // empty destructor since parent destructor destroys frame node anyway
}

void ObjectDetectionDoorVisual::setMessage(const tuw_object_msgs::ObjectWithCovariance::ConstPtr& msg)
{
  // call parent class set Message to also display covariance, center, ids
  ObjectDetectionVisual::setMessage(msg);

  Ogre::Vector3 position = Ogre::Vector3(msg->object.pose.position.x, msg->object.pose.position.y, msg->object.pose.position.z);

  position = transform_ * position;
  position.z = 0;  // fix on ground z=0

  Ogre::Quaternion orientation = Ogre::Quaternion(msg->object.pose.orientation.w, msg->object.pose.orientation.x,
                                                  msg->object.pose.orientation.y, msg->object.pose.orientation.z);
  double width = msg->object.shape_variables[DoorObjectShapeVariables::width];
  double height = msg->object.shape_variables[DoorObjectShapeVariables::height];
  double d_angle = msg->object.shape_variables[DoorObjectShapeVariables::angle_d];
  bool clock_wise = ((int) msg->object.shape_variables[DoorObjectShapeVariables::clockwise] == 0) ? true : false;
  door_visual_->setWidth(width);
  door_visual_->setHeight(height);
  door_visual_->setOpeningAngle(d_angle, clock_wise);
  Ogre::Quaternion rotation_local_q;
  rotation_local_q.FromRotationMatrix(door_visual_->getRotationMat());
  orientation = transform_.extractQuaternion() * orientation * rotation_local_q;
  door_visual_->setPosition(position);
  door_visual_->setOrientation(orientation);
  boost::shared_ptr<tuw_object_rviz::HasWireframe> dv_bb = boost::dynamic_pointer_cast<tuw_object_rviz::HasWireframe>(door_visual_);
  if (dv_bb)
  {
    dv_bb->generateWireframe();
  }
  boost::shared_ptr<tuw_object_rviz::HasBaseframe> base_f_bb =  boost::dynamic_pointer_cast<tuw_object_rviz::HasBaseframe>(door_visual_);
  if (base_f_bb)
  {
    base_f_bb->generateBaseframe();
  }
}

// Color is passed through to the pose Shape object.
void ObjectDetectionDoorVisual::setColor(Ogre::ColourValue color)
{
  ObjectDetectionVisual::setColor(color);
  door_visual_->setColor(color);
}

void ObjectDetectionDoorVisual::setStyle(Styles style)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  Ogre::ColourValue color;
  DoorVisualDefaultArgs default_args(scene_manager_, frame_node_);

  position = door_visual_->getPosition();
  orientation = door_visual_->getOrientation();
  color = door_visual_->getColor();

  switch (style)
  {
    case STYLE_BOUNDING_BOXES:
      door_visual_.reset(new BoundingBoxDoorVisual(default_args));
      break;
    default:
        break;
  }

  door_visual_->setOrientation(orientation);
  door_visual_->setPosition(position);
  door_visual_->setColor(color);

}

}  // end namespace tuw_object_rviz
