#include "createmapper.h"

Mapper::Mapper(int argc, char **argv)
{

    if(!ros::isInitialized()){
        //int argc = 0;
        //char** argv = NULL;
        ros::init(argc, argv, "MapperNode");
    }
    this->mapnode.reset(new ros::NodeHandle("MapperNode"));

    this->lidarscan_sub = this->mapnode->subscribe("/pioneer2dxLidar/vlpscan", 1000, &Mapper::Lidar_Cb, this);

    this->map_pub = this->mapnode->advertise<sensor_msgs::PointCloud2>("/lidarmap", 1000);

    this->transform = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    ros::spin();

}

Mapper::~Mapper()
{

}

void Mapper::Lidar_Cb(const sensor_msgs::PointCloud2 &msg)
{
    //std::cout << msg.header.frame_id << std::endl;

    sensor_msgs::PointCloud pcl_cloud_in, pcl_cloud_out;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pcl_cloud_in);
    tf::StampedTransform tf_data;
    try {

        this->transform->lookupTransform("odom", "lidar", ros::Time(0), tf_data);
        this->transform->transformPointCloud("odom", pcl_cloud_in, pcl_cloud_out);

        sensor_msgs::convertPointCloudToPointCloud2(pcl_cloud_out,this->pc2_cloud_out);



        this->map_pub.publish(pc2_cloud_out);


    } catch (tf::TransformException &ex) {

        ROS_ERROR("%s", ex.what());

        ros::Duration(1.0).sleep();
    }





    /*
    sensor_msgs::convertPointCloudToPointCloud2(pcl_cloud_out,this->pc2_cloud_out);

    this->map_pub.publish(pc2_cloud_out);
*/
    /*
    std::cout << "X = " << tf_data.getOrigin().x() << std::endl;
    std::cout << "Y = " << tf_data.getOrigin().y() << std::endl;
    std::cout << "Z = " << tf_data.getOrigin().z() << std::endl;

    tf::Matrix3x3 m(tf_data.getRotation());
    std::cout << "[" << m.getRow(0).getX() <<"," << m.getRow(0).getY() << "," <<m.getRow(0).getZ() << std::endl;
    std::cout << m.getRow(1).getX() <<"," << m.getRow(1).getY() << "," <<m.getRow(1).getZ() << std::endl;
    std::cout << m.getRow(2).getX() <<"," << m.getRow(2).getY() << "," <<m.getRow(2).getZ() << "]" << std::endl;

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

     std::cout << "Roll = " << roll*180/M_PI << std::endl;
     std::cout << "Pitch = " << pitch*180/M_PI << std::endl;
     std::cout << "Yaw = " << yaw*180/M_PI << std::endl;
    std::cout <<"===============================" << std::endl;
    */

}

