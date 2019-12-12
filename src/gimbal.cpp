#include "gimbal.h"

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Gimbal)
Gimbal::Gimbal()
{
}

Gimbal::~Gimbal()
{
}

void Gimbal::Init()
{
    this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Gimbal::OnUpdate, this));
}

void Gimbal::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->model = _model;
    this->modelName = this->model->GetName();

    if(_sdf->HasElement("link"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link");
        this->linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(this->linkName);
    } else {
        ROS_ERROR("Gimbal plugin missing <link>. Plugin will not run");
        return;
    }

    if(_sdf->HasElement("yawjoint"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("yawjoint");
        this->yawjoint = this->model->GetJoint(elem->Get<std::string>());
    } else {
        ROS_ERROR("Gimbal plugin missing <yawjoint>. Plugin will not run");
        return;
    }

    if(!_sdf->HasElement("serviceName"))
    {
        this->GimbalServ = this->modelName + "/gimbalreset";
        ROS_INFO_STREAM("Gimbal plugin missing <serviceName>, defaults to" << this->GimbalServ);
    } else {
        this ->GimbalServ = this->modelName + "/" + _sdf->Get<std::string>("serviceName");
    }

    if(_sdf->HasElement("WorldFrame"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WorldFrame");
        this->worldframename = elem->Get<std::string>();

        ROS_INFO_STREAM("Gimbal plugin <WorldFrame> set as " << this->worldframename);
    }
    else
    {
        ROS_ERROR("Gimbal plugin <WorldFrame> element not defined");
        return;
    }

    if(_sdf->HasElement("ParentFrame"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("ParentFrame");
        this->parentframename = elem->Get<std::string>();

        ROS_INFO_STREAM("Gimbal plugin <ParentFrame> set as " << this->parentframename);
    }
    else
    {
        ROS_ERROR("Gimbal plugin <ParentFrame> element not defined");
        return;
    }

    if(_sdf->HasElement("ModelFrame"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("ModelFrame");
        this->modelframename = elem->Get<std::string>();

        ROS_INFO_STREAM("Gimbal plugin <ModelFrame> set as " << this->modelframename);

    }
    else
    {
        ROS_ERROR("Gimbal plugin <ModelFrame> element not defined");
        return;
    }


    this->nodeName = "/";
    if(!ros::isInitialized()){
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, this->nodeName);
    }

    this->node.reset(new ros::NodeHandle(this->nodeName));
    this->GimbalTrigger = this->node->advertiseService(this->GimbalServ, &Gimbal::GimbalReset, this);  //.advertiseService("trigger_on",callback);

    this->ResetTrigger = true;
    this->odomlistener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
    this->childtransform =boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    //yaw pid
    this->yaw_pid.SetPGain(0.5);
    this->yaw_pid.SetIGain(0.001);
    this->yaw_pid.SetDGain(0.0011);
    this->yaw_pid.SetIMin(-100);
    this->yaw_pid.SetIMax(100);
    this->yaw_pid.SetCmdMin(-5);
    this->yaw_pid.SetCmdMax(5);

}

bool Gimbal::GimbalReset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{

    this->ResetTrigger = true;
    res.success = true;
    res.message = "Gimbal reset to current model orientation";
    return true;
}

void Gimbal::OnUpdate()
{
    // Calculate transformation between parent and model
    tf::Quaternion qt;
    tf::Vector3 vt;

    ignition::math::Vector3d childrelativepos;
    ignition::math::Quaterniond childrelativerot;

    childrelativepos = -this->model->WorldPose().Pos() + this->link->WorldPose().Pos();
    childrelativerot = -this->model->WorldPose().Rot() + this->link->WorldPose().Rot();

    //std::cout << childrelativepos.X() <<std::endl;
    //std::cout << childrelativepos.Y() <<std::endl;
    //std::cout << childrelativepos.Z() <<std::endl;

    vt = tf::Vector3(childrelativepos.X(),
                     childrelativepos.Y(),
                     childrelativepos.Z());

    qt = tf::Quaternion(childrelativerot.X(),
                        childrelativerot.Y(),
                        childrelativerot.Z(),
                        childrelativerot.W());


    //tf::Matrix3x3 test(qt); //get rotation matrix of the parent model
    //double test_roll, test_pitch, test_yaw;
    //test.getRPY(test_roll, test_pitch, test_yaw);
    //std::cout << "Roll = " << test_roll*180/M_PI << ", Pitch = " << test_pitch*180/M_PI << ", Yaw = " << test_yaw*180/M_PI << std::endl;
    //std::cout << "==========================================" << std::endl;

    tf::Transform lidar_to_parent(qt, vt);
    this->childtransform->sendTransform(tf::StampedTransform(lidar_to_parent,
                                                             ros::Time::now(),
                                                             this->parentframename,
                                                             this->modelframename));


    // Get the transform between world frame and the parent to which lidar is attached
    tf::StampedTransform transform;

    try{
        this->odomlistener->lookupTransform(this->worldframename, this->parentframename,
                                            ros::Time(0), transform);
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();

    }

    tf::Matrix3x3 m(transform.getRotation()); //get rotation matrix of the parent model
    m.getRPY(roll_parent, pitch_parent, yaw_parent); //get roll,pitch,yaw from rot matrix

    // Get rotation data of gimbal from gazebo
    roll_gim = this->link->WorldPose().Rot().Roll();
    pitch_gim = this->link->WorldPose().Rot().Pitch();
    yaw_gim = this->link->WorldPose().Rot().Yaw();

    if(this->ResetTrigger){

        this->yaw_set = this->yaw_parent;// set the setpoint of gimbal to orientation of parent

        this->yaw_pid.SetCmd(0);

        this->ResetTrigger = false;
    }

    while((this->yaw_set - yaw_gim) < -3.14159265)
    {
        this->yaw_set += 2*3.14159265;
    }

    while((this->yaw_set - yaw_gim) > 3.14159265)
    {
        this->yaw_set -= 2*3.14159265;
    }
    //std::cout << transform.getRotation().inverse() << std::endl;
    //tf::Matrix3x3 s(transform.getRotation()), m(transform.getRotation().inverse());
    //double roll, pitch, yaw;
    //double rollinv, pitchinv, yawinv;
    //s.getRPY(roll, pitch, yaw);
    //m.getRPY(rollinv, pitchinv, yawinv);

    //std::cout << roll <<","<<pitch <<","<<yaw*180/3.14159265 << std::endl;
    //std::cout << rollinv <<","<< pitchinv <<","<<yawinv*180/3.14159265 << std::endl;
    //std::cout << "-----------------" << std::endl;
    //this->yawjoint->SetPosition(0, 0, true);


    this->yaw_pid.Update((this->yaw_set - this->yaw_gim)*180/3.14159265, 0.001);
    this->yawjoint->SetVelocity(0, this->yaw_pid.GetCmd());
}

