/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LINES_SEGMENTS_2D_VISUAL_H
#define LINES_SEGMENTS_2D_VISUAL_H

#include <tuw_geometry_msgs/LineSegments.h>
#include <rviz/ogre_helpers/billboard_line.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
//class Arrow;
//class Shape;
//class Line;
class BillboardLine;
}

namespace tuw_geometry_rviz
{

// Declare the visual class for this display.
class LineSegments2DVisual
{
public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    LineSegments2DVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~LineSegments2DVisual();

    // Configure the visual to show the data in the message.
    void setMessage ( const tuw_geometry_msgs::LineSegments::ConstPtr& msg );

    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean.  This way LineSegments2DVisual is
    // only responsible for visualization.
    void setFramePosition ( const Ogre::Vector3& position );
    void setFrameOrientation ( const Ogre::Quaternion& orientation );

    // Set the scale of the visual, which is an user-editable
    // parameter and therefore don't come from the PoseWithCovarianceStamped message.
    void setScaleSegments ( float scale );

    // Set the color of the visual's Pose, which is an user-editable
    // parameter and therefore don't come from the PoseWithCovarianceStamped message.
    void setColorSegments ( Ogre::ColourValue color );

    void setWidthSegments ( float width );

    // Set the color of the visual's variance, which is an user-editable
    // parameter and therefore don't come from the PoseWithCovarianceStamped message.
    //void setColorVariance( Ogre::ColourValue color );

private:
    // The object implementing the actual BillboardLine
    boost::shared_ptr<rviz::BillboardLine> linesegments_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the Imu message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;

};

} // end namespace tuw_pose_rviz_plugin

#endif // LINES_SEGMENTS_2D_VISUAL_H
