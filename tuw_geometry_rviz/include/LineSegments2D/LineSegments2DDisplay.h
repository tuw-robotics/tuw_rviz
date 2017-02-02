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

#ifndef LINES_SEGMENTS_2D_DISPLAY_H
#define LINES_SEGMENTS_2D_DISPLAY_H

#ifndef Q_MOC_RUN
#include <tuw_geometry_msgs/LineSegments.h>
#include <rviz/message_filter_display.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_geometry_rviz
{

class LineSegments2DVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class LineSegments2DDisplay: public rviz::MessageFilterDisplay<tuw_geometry_msgs::LineSegments>
{
    Q_OBJECT
public:
    // Constructor.  pluginlib::ClassLoader creates instances by calling
    // the default constructor, so make sure you have one.
    LineSegments2DDisplay();
    virtual ~LineSegments2DDisplay();

    // Overrides of protected virtual functions from Display.  As much
    // as possible, when Displays are not enabled, they should not be
    // subscribed to incoming data and should not show anything in the
    // 3D view.  These functions are where these connections are made
    // and broken.
protected:
    virtual void onInitialize();

    // A helper to clear this display back to the initial state.
    virtual void reset();

    // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
    void updateScaleSegments();
    void updateColorSegments();
    void updateWidthSegments();
private:
    // Function to handle an incoming ROS message.
    void processMessage ( const tuw_geometry_msgs::LineSegments::ConstPtr& msg );

    // Storage of the visual
    boost::shared_ptr<LineSegments2DVisual> visual_;

    // User-editable property variables.
    rviz::ColorProperty* property_color_segments_;
    rviz::FloatProperty* property_scale_segments_;
    rviz::FloatProperty* property_width_segments_;
    
};

} // end namespace tuw_pose_rviz_plugin

#endif // LINES_SEGMENTS_2D_DISPLAY_H
// %EndTag(FULL_SOURCE)%
