#ifndef TrafficConeVISUAL_H
#define TrafficConeVISUAL_H

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

namespace fs = boost::filesystem;

namespace tuw_object_rviz {

    // Default arguments that need to be supplied to all types of TrafficConeVisual
    struct TrafficConeVisualDefaultArgs {
        TrafficConeVisualDefaultArgs(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) : sceneManager(sceneManager), parentNode(parentNode) {}
        Ogre::SceneManager* sceneManager;
        Ogre::SceneNode* parentNode;
    };

    /// Base class for all person visualization types
    class TrafficConeVisual {
    public:
        TrafficConeVisual(const TrafficConeVisualDefaultArgs& args);

        virtual ~TrafficConeVisual();

        void setPosition(const Ogre::Vector3& position);

        const Ogre::Vector3& getPosition() const;

        void setOrientation(const Ogre::Quaternion& orientation);

        const Ogre::Quaternion& getOrientation() const;

        virtual void setScalingFactor(double scalingFactor);

        void setVisible(bool visible);

        Ogre::SceneNode* getParentSceneNode();

        virtual void setHeight(double height);

        virtual void setRadius(double radius);

        virtual void update(float deltaTime);

        virtual void setColor(const Ogre::ColourValue& c) = 0;

        virtual Ogre::ColourValue& getColor() = 0;

        virtual double getHeight() = 0;

    protected:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode *m_sceneNode, *m_parentSceneNode;
        Ogre::ColourValue m_color;
        double m_width, m_height, m_radius;
    };

    /// Visualization of a person as a wireframe bounding box
    class TrafficConeVisualImpl : public TrafficConeVisual {
    public:
        TrafficConeVisualImpl ( const TrafficConeVisualDefaultArgs& args, double height = 1.75, double radius = 0.6);

        virtual ~TrafficConeVisualImpl();

        virtual void setColor(const Ogre::ColourValue& c);

        virtual Ogre::ColourValue& getColor();

        virtual double getHeight();

        virtual void setHeight(double height);

        virtual void setRadius(double radius);

        virtual void update(float deltaTime);
        /*
        virtual void setScalingFactor(double scalingFactor);
        */

    private:
        rviz::Shape *m_bodyShape;
        double m_height;
        double m_radius;
    };

//    /// Visualization of a person as cylinder (body) + sphere (head)
//    class CylinderTrafficConeVisual : public TrafficConeVisual {
//    public:
//        CylinderTrafficConeVisual(const TrafficConeVisualDefaultArgs& args);

//        virtual ~CylinderTrafficConeVisual();

//        virtual void setColor(const Ogre::ColourValue& c);

//        virtual Ogre::ColourValue& getColor();

//        virtual double getHeight();

//    private:
//        rviz::Shape *m_bodyShape, *m_headShape;
//    };

//    /// Visualization of a person as a mesh (walking human)
//    class MeshTrafficConeVisual : public TrafficConeVisual {

//    private:
//        Ogre::SceneNode *m_childSceneNode;
//        Ogre::Entity* entity_;
//        Ogre::AnimationState* m_animationState;
//        std::set<Ogre::MaterialPtr> materials_;
//        float m_walkingSpeed;
//    public:
//        MeshTrafficConeVisual ( const TrafficConeVisualDefaultArgs& args );

//        virtual ~MeshTrafficConeVisual();

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

#endif // TrafficConeVISUAL_H
