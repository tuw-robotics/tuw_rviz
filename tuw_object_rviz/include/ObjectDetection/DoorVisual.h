#ifndef DOORVISUAL_H
#define DOORVISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <OgreSceneNode.h>
#include <OgreAnimation.h>
#include <OgreSharedPtr.h>
#include <OgreEntity.h>

#include <resource_retriever/retriever.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <ObjectDetection/PersonVisual.h>
#include <ObjectDetection/TextVisual.h>

namespace fs = boost::filesystem;

namespace tuw_object_rviz {

    class HasWireframe {
      public:
        virtual void generateWireframe() = 0;
    };

    class HasBaseframe {
      public:
        virtual void generateBaseframe() = 0;
    };

    // Default arguments that need to be supplied to all types of DoorVisual
    struct DoorVisualDefaultArgs {
        DoorVisualDefaultArgs(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) : sceneManager(sceneManager), parentNode(parentNode) {}
        Ogre::SceneManager* sceneManager;
        Ogre::SceneNode* parentNode;
    };

    /// Base class for all person visualization types
    class DoorVisual {
    public:
        DoorVisual(const DoorVisualDefaultArgs& args);

        virtual ~DoorVisual();

        void setPosition(const Ogre::Vector3& position);

        const Ogre::Vector3& getPosition() const;

        void setOrientation(const Ogre::Quaternion& orientation);

        const Ogre::Quaternion& getOrientation() const;

        virtual void setScalingFactor(double scalingFactor);

        void setVisible(bool visible);

        Ogre::SceneNode* getParentSceneNode();

        virtual void setHeight(double height);

        virtual void setWidth(double width);

        virtual void setOpeningAngle(double oangle, bool clockwise);

        virtual void update(float deltaTime);

        virtual Ogre::Matrix3 getRotationMat();

        virtual void setColor(const Ogre::ColourValue& c) = 0;

        virtual Ogre::ColourValue& getColor() = 0;

        virtual double getHeight() = 0;

    protected:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode *m_sceneNode, *m_parentSceneNode;
        Ogre::ColourValue m_color;
        Ogre::Matrix3 R_do;
        bool m_clockwise;
        double m_oangle;
        double m_width, m_height;
    };

    /// Visualization of a person as a wireframe bounding box
    class BoundingBoxDoorVisual : public DoorVisual, public tuw_object_rviz::HasLineWidth, public tuw_object_rviz::HasWireframe, public tuw_object_rviz::HasBaseframe {
    public:
        BoundingBoxDoorVisual ( const DoorVisualDefaultArgs& args, double height = 1.75, double width = 0.6, double scalingFactor = 1.0 );

        virtual ~BoundingBoxDoorVisual();

        virtual void setColor(const Ogre::ColourValue& c);

        virtual Ogre::ColourValue& getColor();

        virtual double getHeight();

        virtual void setLineWidth(double lineWidth);

        virtual void generateWireframe();

        virtual void generateBaseframe();
        /*
        virtual void setScalingFactor(double scalingFactor);
        */

    private:
        rviz::BillboardLine *m_wireframe;
        rviz::BillboardLine *m_baseframe;
        double m_scalingFactor, m_lineWidth;
        double m_thickness;
    };

//    /// Visualization of a person as cylinder (body) + sphere (head)
//    class CylinderDoorVisual : public DoorVisual {
//    public:
//        CylinderDoorVisual(const DoorVisualDefaultArgs& args);

//        virtual ~CylinderDoorVisual();

//        virtual void setColor(const Ogre::ColourValue& c);

//        virtual Ogre::ColourValue& getColor();

//        virtual double getHeight();

//    private:
//        rviz::Shape *m_bodyShape, *m_headShape;
//    };

//    /// Visualization of a person as a mesh (walking human)
//    class MeshDoorVisual : public DoorVisual {

//    private:
//        Ogre::SceneNode *m_childSceneNode;
//        Ogre::Entity* entity_;
//        Ogre::AnimationState* m_animationState;
//        std::set<Ogre::MaterialPtr> materials_;
//        float m_walkingSpeed;
//    public:
//        MeshDoorVisual ( const DoorVisualDefaultArgs& args );

//        virtual ~MeshDoorVisual();

//        virtual void update(float deltaTime);

//        virtual void setColor(const Ogre::ColourValue& c);

//        virtual Ogre::ColourValue& getColor();

//        void setAnimationState(const std::string& nameOfAnimationState);

//        void setWalkingSpeed(float walkingSpeed);

//        virtual double getHeight() {
//            return 1.75;
//        }

//        virtual void setScalingFactor(double scalingFactor) {
//            // Not supported (for some reason causes the mesh to be mirrored vertically).
//        }
//    };

}

#endif // DOORVISUAL_H
