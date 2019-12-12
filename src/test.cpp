#include "test.h"

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RosDiffDriveControl)

RosDiffDriveControl::RosDiffDriveControl()
{
}

RosDiffDriveControl::~RosDiffDriveControl()
{
}

void RosDiffDriveControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    this->model = _model;
    this->modelName = _model->GetName();
    this->nodeName = this->modelName + "_ControlNode";
    this->controlTopicName = "/" + this->modelName + "/DiffDriveControlCmd";

    if(_sdf->HasElement("RightWheelJoint") && _sdf->HasElement("LeftWheelJoint"))
    {
        sdf::ElementPtr elem_right = _sdf->GetElement("RightWheelJoint");
        this->rightwheel = this->model->GetJoint(elem_right->Get<std::string>());

        sdf::ElementPtr elem_left = _sdf->GetElement("LeftWheelJoint");
        this->leftwheel = this->model->GetJoint(elem_left->Get<std::string>());

        //ROS_INFO("Right joint name is %s", elem_right->Get<std::string>());
        //ROS_INFO("Left joint name is %s", elem_left->Get<std::string>());
        //std::cout << "Right joint name is " << elem_right->Get<std::string>() << std::endl;
        //std::cout << "Left joint name is " << elem_left->Get<std::string>() << std::endl;

    }
    else
    {
        //ROS_ERROR("Unable to find Wheel Joints sdf information");
        //std::cout << "Unable to find Wheel Joints sdf information";
        return;
    }

    if(_sdf->HasElement("WheelSeparation"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WheelSeparation");
        this->wheel_separation = elem->Get<double>();

        //ROS_INFO("Wheel distance = " );//+ std::to_string(this->wheel_separation)   );
        //std::cout << "Wheel distance = " << this->wheel_separation << std::endl;
    }
    else
    {
        //ROS_ERROR("WheelSeparation element not defined");
        //std::cout << "WheelSeparation element not defined" << std::endl;
        return;
    }

    if(_sdf->HasElement("WheelRadius"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WheelRadius");
        this->wheel_radius = elem->Get<double>();

        //ROS_INFO("Wheel radius = " << this->wheel_radius);
        //std::cout << "Wheel radius = " << this->wheel_radius << std::endl;

    }
    else
    {
        //ROS_ERROR("")
        //std::cout << "WheelRadius element not defined" << std::endl;
        return;
    }

    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->nodeName);
    }

    this->odomtransform = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    this->node.reset(new ros::NodeHandle(this->nodeName));
    this->ControlSub = this->node->subscribe(this->controlTopicName, 1000, &RosDiffDriveControl::ControlCallback, this);

    this->NEWDATA = false;
    this->WAITING4DATA = true;

}

void RosDiffDriveControl::Init()
{
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&RosDiffDriveControl::OnUpdate, this));
}

void RosDiffDriveControl::ControlCallback(const geometry_msgs::TwistPtr &msg)
{
    if(this->WAITING4DATA){

        this->LinearSpeed = msg->linear.x;
        this->RotateSpeed = msg->angular.z;

        //std::cout << "Lin speed = " << this->LinearSpeed << std::endl;
        //std::cout << "Rot speed = " << this->RotateSpeed << std::endl;

        this->NEWDATA = true;
        this->WAITING4DATA = false;

    }
}

void RosDiffDriveControl::OnUpdate()
{
    // calculate the odometry information
    tf::Quaternion qt;
    tf::Vector3 vt;

    ignition::math::Pose3d robotpose;
    robotpose = this->model->WorldPose();
    vt = tf::Vector3(robotpose.Pos().X(), robotpose.Pos().Y(), robotpose.Pos().Z());
    qt = tf::Quaternion(robotpose.Rot().X(), robotpose.Rot().Y(),
                        robotpose.Rot().Z(), robotpose.Rot().W());

    tf::Transform base_to_world(qt,vt);

    std::string frame, child_frame;
    frame = "odom";
    child_frame = "base_frame";
    ros::Time current_time = ros::Time::now();

    //std::cout << "sending transform" << std::endl;

    odomtransform->sendTransform( tf::StampedTransform( base_to_world,
                                                      current_time,
                                                      frame,
                                                      child_frame) );


    if(this->NEWDATA){

        // calculate wheel speeds
        float LeftWheelSpeed(0), RightWheelSpeed(0);

        LeftWheelSpeed = this->LinearSpeed -
                ((this->wheel_separation * this->RotateSpeed)/(this->wheel_radius));
        RightWheelSpeed = this->LinearSpeed +
                ((this->wheel_separation * this->RotateSpeed)/(this->wheel_radius));

        //std::cout << "Left wheel speed = " << LeftWheelSpeed << std::endl;
        //std::cout << "Right wheel speed = " << RightWheelSpeed << std::endl;

        this->leftwheel->SetVelocity(0, LeftWheelSpeed);

        this->rightwheel->SetVelocity(0, RightWheelSpeed);

        this->NEWDATA = false;
        this->WAITING4DATA = true;

    }

    usleep(1000);
    ros::spinOnce();
}
