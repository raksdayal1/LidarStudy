#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include <unistd.h>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <std_srvs/Trigger.h>

using namespace gazebo;

class Gimbal : public ModelPlugin
{
public:
    Gimbal();
    ~Gimbal();

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();

private:
    bool GimbalReset(std_srvs::TriggerRequest &req,
                     std_srvs::TriggerResponse &res);
    virtual void OnUpdate();

    event::ConnectionPtr _updateConnection;

    common::PID yaw_pid;
    physics::ModelPtr model;
    std::string modelName;
    physics::LinkPtr link;
    std::string linkName;
    physics::JointPtr yawjoint;

    ros::NodeHandlePtr node;
    std::string nodeName;
    ros::ServiceServer GimbalTrigger;
    std::string GimbalServ;

    bool ResetTrigger;

    double roll_parent, pitch_parent, yaw_parent;
    double roll_gim, pitch_gim, yaw_gim;
    double roll_set, pitch_set, yaw_set;

    boost::shared_ptr<tf::TransformListener> odomlistener;
};

#endif // GIMBAL_H
