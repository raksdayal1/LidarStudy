#ifndef TEST_H
#define TEST_H

#include <iostream>
#include <string>
#include <unistd.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
    class RosDiffDriveControl : public ModelPlugin
    {
    public:
        RosDiffDriveControl();
        ~RosDiffDriveControl();

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

    private:
        void ControlCallback(const geometry_msgs::TwistPtr &msg);
        virtual void OnUpdate();

        event::ConnectionPtr _updateConnection;

        physics::ModelPtr model;
        physics::WorldPtr world;
        physics::PhysicsEnginePtr physics;
        physics::LinkPtr link;
        physics::JointPtr rightwheel, leftwheel;

        boost::shared_ptr<tf::TransformBroadcaster> odomtransform;
        //tf::TransformBroadcaster odomtransform;

        ros::NodeHandlePtr node;
        ros::Subscriber ControlSub;

        std::string modelName, nodeName, controlTopicName;
        float wheel_separation, wheel_radius;
        float LinearSpeed, RotateSpeed;

        bool NEWDATA, WAITING4DATA;

    };

};

#endif // TEST_H
