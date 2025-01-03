#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <cmath>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*input, *cloud);

    // Downsample the cloud using VoxelGrid filter
    pcl::VoxelGrid<PointT> vg;
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust the leaf size for downsampling
    vg.filter(*cloud_filtered);

    // Remove points within a radius of 0.8m from the origin
    pcl::CropBox<PointT> cropBoxFilter1;
    cropBoxFilter1.setInputCloud(cloud_filtered);
    cropBoxFilter1.setMin(Eigen::Vector4f(-0.01, -0.01, -0.01, 1.0));
    cropBoxFilter1.setMax(Eigen::Vector4f(0.01, 0.01, 0.01, 1.0));
    cropBoxFilter1.setNegative(true); // Remove points inside the box
    PointCloudT::Ptr cloud_cropped(new PointCloudT);
    cropBoxFilter1.filter(*cloud_cropped);

    // Keep points within x ± 5m, y ± 10m, z ± 5m
    pcl::CropBox<PointT> cropBoxFilter2;
    cropBoxFilter2.setInputCloud(cloud_cropped);
    cropBoxFilter2.setMin(Eigen::Vector4f(-0.5, -1.2, -5.0, 1.0));
    cropBoxFilter2.setMax(Eigen::Vector4f(5.0, 1.2, 5.0, 1.0));
    PointCloudT::Ptr cloud_filtered_final(new PointCloudT);
    cropBoxFilter2.filter(*cloud_filtered_final);

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered_final);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.5); // Increased tolerance for larger clusters
    ec.setMinClusterSize(15); // Minimum cluster size
    ec.setMaxClusterSize(10000); // Maximum cluster size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_final);
    ec.extract(cluster_indices);

    PointCloudT::Ptr cloud_clustered(new PointCloudT);
    int current_cluster = 0;

    for (const auto& indices : cluster_indices) {
        // Choose a color for the current cluster
        uint8_t r = 0, g = 0, b = 0;
        switch (current_cluster % 3) {
            case 0: r = 0; g = 0; b = 255; break;   // Blue
            case 1: r = 255; g = 255; b = 0; break; // Yellow
            case 2: r = 0; g = 255; b = 0; break;   // Green
        }

        // Calculate centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_filtered_final, indices, centroid);
        
        // Calculate distance from LiDAR origin to the centroid of the cluster
        double distance = sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
        ROS_INFO("Cluster %d distance: %f meters", current_cluster, distance);

        // Color the points of the current cluster
        for (const auto& idx : indices.indices) {
            PointT point = cloud_filtered_final->points[idx];
            point.r = r;
            point.g = g;
            point.b = b;
            cloud_clustered->points.push_back(point);
        }
        current_cluster++;
    }

    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;

    // Convert back to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_clustered, output);
    output.header.frame_id = input->header.frame_id;
    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "euclidean_cluster");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, pointcloud_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_cloud", 1);

    ros::spin();
    return 0;
}
