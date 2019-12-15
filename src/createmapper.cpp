#include "createmapper.h"

Mapper::Mapper(int argc, char **argv)
{

    if(!ros::isInitialized()){
        //int argc = 0;
        //char** argv = NULL;
        ros::init(argc, argv, "MapperNode");
    }
    this->GOTFIRST = false;
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
    sensor_msgs::PointCloud pcl_cloud_in, pcl_cloud_out;
    tf::StampedTransform tf_data;

    // convert the ros msg in PC2 to ros msg PC1
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pcl_cloud_in);

    try {
        // Look up the transform between odom frame and lidar frame
        this->transform->lookupTransform("odom", "lidar", ros::Time(0), tf_data);

        // transform data obtained in lidar frame in ros PC1 to world frame (data again in ROS PC1)
        this->transform->transformPointCloud("odom", pcl_cloud_in, pcl_cloud_out);

        // Convert the ros PC1 to ros PC2
        sensor_msgs::convertPointCloudToPointCloud2(pcl_cloud_out,this->pc2_cloud_out);

        /*
        // Convert the transformed ros PC1 data in odom frame into pcl pointcloud XYZRGB data
        pcl::fromROSMsg(this->pc2_cloud_out, this->Temp_scan_cloud);


        if(!this->GOTFIRST)
        {
            // First message is passed into fullcloud
            this->Temp_full_cloud = this->Temp_scan_cloud;
            this->GOTFIRST = true;
        }
        else {
            // Append new message to full cloud
            this->Temp_full_cloud += this->Temp_full_cloud + this->Temp_scan_cloud;

        }

        // convert back to ros PC2
        pcl::toROSMsg(this->Temp_full_cloud, this->FullyCloud);
*/
        // Publish the message
        this->map_pub.publish(pc2_cloud_out);


    } catch (tf::TransformException &ex) {

        ROS_ERROR("%s", ex.what());

        ros::Duration(1.0).sleep();
    }

}

