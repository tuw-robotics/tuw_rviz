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

#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "PoseWithCovarianceStamped/PoseWithCovarianceStampedVisual.h"

namespace tuw_geometry_rviz {

PoseWithCovarianceVisual::PoseWithCovarianceVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the MarkerDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // We create the visual objects within the frame node so that we can
    // set thier position and direction relative to their header frame.
    pose_.reset ( new rviz::Arrow( scene_manager_, frame_node_ ) );
    variance_.reset ( new rviz::Shape ( rviz::Shape::Sphere, scene_manager_, frame_node_ ) );
}

PoseWithCovarianceVisual::~PoseWithCovarianceVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void PoseWithCovarianceVisual::setMessage ( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) {
    Ogre::Vector3 position = Ogre::Vector3 ( msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z );
    Ogre::Quaternion orientation = Ogre::Quaternion ( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w );

    // Arrow points in -Z direction, so rotate the orientation before display.
    // TODO: is it safe to change Arrow to point in +X direction?
    Ogre::Quaternion rotation1 = Ogre::Quaternion ( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y );
    Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Degree( -180 ), Ogre::Vector3::UNIT_X );
    orientation = rotation2 * rotation1 * orientation;

    pose_->setPosition( position );
    pose_->setOrientation( orientation );

    Ogre::Matrix3 C = Ogre::Matrix3 ( msg->pose.covariance[6*0 + 0], msg->pose.covariance[6*0 + 1], msg->pose.covariance[6*0 + 5],
                                        msg->pose.covariance[6*1 + 0], msg->pose.covariance[6*1 + 1], msg->pose.covariance[6*1 + 5],
                                        msg->pose.covariance[6*5 + 0], msg->pose.covariance[6*5 + 1], msg->pose.covariance[6*5 + 5] );
    Ogre::Real eigenvalues[3];
    Ogre::Vector3 eigenvectors[3];
    C.EigenSolveSymmetric(eigenvalues, eigenvectors);
    if ( eigenvalues[0] < 0 ) {
        ROS_WARN ( "[PoseWithCovarianceVisual setMessage] eigenvalue[0]: %f < 0 ",  eigenvalues[0] );
        eigenvalues[0] = 0;
    }
    if ( eigenvalues[1] < 0 ) {
        ROS_WARN ( "[PoseWithCovarianceVisual setMessage] eigenvalue[1]: %f < 0 ",  eigenvalues[1] );
        eigenvalues[1] = 0;
    }
    if ( eigenvalues[2] < 0 ) {
        ROS_WARN ( "[PoseWithCovarianceVisual setMessage] eigenvalue[2]: %f < 0 ",  eigenvalues[2] );
        eigenvalues[2] = 0;
    }

    variance_->setColor ( color_variance_ );
    variance_->setPosition ( position );
    variance_->setOrientation ( Ogre::Quaternion ( eigenvectors[0], eigenvectors[1], eigenvectors[2] ) );
    variance_->setScale ( Ogre::Vector3 ( 2*sqrt(eigenvalues[0]), 2*sqrt(eigenvalues[1]), 2*sqrt(eigenvalues[2]) ) );
}

// Position is passed through to the SceneNode.
void PoseWithCovarianceVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void PoseWithCovarianceVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Scale is passed through to the pose Shape object.
void PoseWithCovarianceVisual::setScalePose ( float scale ) {
    pose_->setScale ( Ogre::Vector3 ( scale, scale, scale ) );
    scale_pose_ = scale;
}

// Color is passed through to the pose Shape object.
void PoseWithCovarianceVisual::setColorPose ( Ogre::ColourValue color ) {
    pose_->setColor ( color );
    color_pose_ = color;
}

// Color is passed through to the variance Shape object.
void PoseWithCovarianceVisual::setColorVariance ( Ogre::ColourValue color ) {
    variance_->setColor ( color );
    color_variance_ = color;
}

} // end namespace marker_rviz
