/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by George Todoran <george.todoran@tuwien.ac.at>        *
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

#ifndef PATH_VISUAL_H
#define PATH_VISUAL_H

#include <nav_msgs/Path.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/arrow.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
}

namespace tuw_nav_rviz
{

// Declare the visual class for this display.
class PathVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  PathVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~PathVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const nav_msgs::Path::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way PathVisual is
  // only responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color of the visual, which is an user-editable
  // parameter and therefore don't come from the Path message.
  void setPathColor( Ogre::ColourValue color );
  void setOrientColor( Ogre::ColourValue color );

  // Set the shape of the visual, which is an user-editable
  // parameter and therefore don't come from the Path message.
  void setShape( rviz::Shape::Type shape_type );

  // Set the scale of the visual, which is an user-editable
  // parameter and therefore don't come from the Path message.
  void setPathScale( float scale );
  void setOrientScale( float scale );
  
  void setOrientDeltaS( double deltaS );

private:
  // The objects implementing the actual shape
  std::vector<boost::shared_ptr<rviz::Shape> > pathPtsXY_;
  std::vector<boost::shared_ptr<rviz::Arrow> > pathPtsTheta_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  // The Shape object's color
  Ogre::ColourValue colorPath_;
  Ogre::ColourValue colorOrient_;
  
  // The Shape object's shape type
  rviz::Shape::Type shape_type_;
  
  // The Shape object's scale
  float scalePath_;
  float scaleOrient_;
  
  double deltaSOrient_;
};

} // end namespace tuw_nav_rviz

#endif // PATH_VISUAL_H
