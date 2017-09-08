#include <iostream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "PCD filename required as argument." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud)) {
        PCL_ERROR ("Couldn't read pcd file\n");
        return(-1);
    }

    RunGroundSegmentation(cloud, cloud_p);

    std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write("output.pcd", *cloud_p, false);

    return (0);
}
