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

#ifndef OBJECT_DETECTION_DISPLAY_H
#define OBJECT_DETECTION_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
#include <tuw_object_msgs/ObjectDetection.h>
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
class BoolProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace tuw_object_rviz
{

class ObjectDetectionVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
class ObjectDetectionDisplay: public rviz::MessageFilterDisplay<tuw_object_msgs::ObjectDetection>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ObjectDetectionDisplay();
  virtual ~ObjectDetectionDisplay();

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
  void objectVisualTypeChanged();
  virtual void stylesChanged();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const tuw_object_msgs::ObjectDetection::ConstPtr& msg );

  // Storage of the visual
  std::vector<boost::shared_ptr<ObjectDetectionVisual> > visual_;

  // Properties
  rviz::BoolProperty* render_covariances_property_;
  rviz::BoolProperty* render_ids_property_;
  rviz::BoolProperty* render_sensor_type_property_;
  rviz::ColorProperty* color_property_;
};

} // end namespace tuw_object_rviz

#endif // OBJECT_DETECTION_DISPLAY_H
// %EndTag(FULL_SOURCE)%
