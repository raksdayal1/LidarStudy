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

        ROS_INFO_STREAM("Right joint name is " << elem_right->Get<std::string>());
        ROS_INFO_STREAM("Left joint name is " << elem_left->Get<std::string>());

    }
    else
    {
        ROS_ERROR("Unable to find Wheel Joints sdf information");
        return;
    }

    if(_sdf->HasElement("WheelSeparation"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WheelSeparation");
        this->wheel_separation = elem->Get<double>();

        ROS_INFO_STREAM("Wheel distance = " << this->wheel_separation);//+ std::to_string(this->wheel_separation)   );
        //std::cout << "Wheel distance = " << this->wheel_separation << std::endl;
    }
    else
    {
        ROS_ERROR("WheelSeparation element not defined");
        //std::cout << "<WheelSeparation> element not defined" << std::endl;
        return;
    }

    if(_sdf->HasElement("WheelRadius"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WheelRadius");
        this->wheel_radius = elem->Get<double>();

        ROS_INFO_STREAM("Wheel radius = " << this->wheel_radius);
    }
    else
    {
        ROS_ERROR("<WheelRadius> element not defined");
        return;
    }

    if(_sdf->HasElement("WorldFrame"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WorldFrame");
        this->worldframename = elem->Get<std::string>();

        ROS_INFO_STREAM("DiffDrive plugin <WorldFrame> set as " << this->worldframename);
    }
    else
    {
        ROS_ERROR("DiffDrive plugin <WorldFrame> element not defined");
        return;
    }

    if(_sdf->HasElement("ModelFrame"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("ModelFrame");
        this->modelframename = elem->Get<std::string>();

        ROS_INFO_STREAM("DiffDrive plugin <ModelFrame> set as " << this->modelframename);

    }
    else
    {
        ROS_ERROR("DiffDrive plugin <ModelFrame> element not defined");
        return;
    }


    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->nodeName);
    }

    ROS_INFO("Differential drive plugin ready");
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

    ros::Time current_time = ros::Time::now();

    odomtransform->sendTransform( tf::StampedTransform( base_to_world,
                                                      current_time,
                                                      this->worldframename,
                                                      this->modelframename) );


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
