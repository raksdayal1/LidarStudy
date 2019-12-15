#ifndef CREATEMAPPER_H
#define CREATEMAPPER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>

class Mapper
{
public:
    Mapper(int argc, char **argv);
    ~Mapper();

public:
    void Lidar_Cb(const sensor_msgs::PointCloud2 &msg);
    ros::NodeHandlePtr mapnode;
    ros::Subscriber lidarscan_sub;
    ros::Publisher map_pub;

    boost::shared_ptr<tf::TransformListener> transform;
    sensor_msgs::PointCloud2 pc2_cloud_out;

};


#endif // CREATEMAPPER_H
