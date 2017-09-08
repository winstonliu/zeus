#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void RunGroundSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size () != 0)
    {
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);
        extract.filter(*cloud_p);
    }
    else
    {
        std::cerr << "ERROR: Could not estimate a planar model for the given dataset." << std::endl;
    }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    sensor_msgs::PointCloud2 output_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    RunGroundSegmentation(cloud, cloud_filtered);

    if (cloud_filtered->points.size() != 0)
    {
        pcl::toROSMsg(*cloud_filtered, output_cloud);
        pub.publish(output_cloud);
    }
    else
    {
        std::cerr << "ERROR: Could not estimate a planar model for the given dataset." << std::endl;
    }
}

int main(int argc, char* argv[])
{
    ros::init (argc, argv, "pcl_ground_segmentation");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("velodyne_points", 100, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("lane_detection/ground_segmentation", 100);

    ros::spin();
}
