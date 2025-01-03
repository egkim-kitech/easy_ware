#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <cmath>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher pub;
ros::Publisher marker_pub;

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
    cropBoxFilter1.setMin(Eigen::Vector4f(-0.7, -0.7, -1.5, 1.0));
    cropBoxFilter1.setMax(Eigen::Vector4f(0.7, 0.7, 1.5, 1.0));
    cropBoxFilter1.setNegative(true); // Remove points inside the box
    PointCloudT::Ptr cloud_cropped(new PointCloudT);
    cropBoxFilter1.filter(*cloud_cropped);

    // Keep points within x ± 5m, y ± 10m, z ± 5m
    pcl::CropBox<PointT> cropBoxFilter2;
    cropBoxFilter2.setInputCloud(cloud_cropped);
    cropBoxFilter2.setMin(Eigen::Vector4f(-0.5, -10.5, -0.3, 1.0));
    cropBoxFilter2.setMax(Eigen::Vector4f(15.0, 10.5, 5.0, 1.0));
    PointCloudT::Ptr cloud_filtered_final(new PointCloudT);
    cropBoxFilter2.filter(*cloud_filtered_final);

    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered_final);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.3); // Increased tolerance for larger clusters
    ec.setMinClusterSize(15); // Minimum cluster size
    ec.setMaxClusterSize(800); // Maximum cluster size
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_final);
    ec.extract(cluster_indices);

    PointCloudT::Ptr cloud_clustered(new PointCloudT);
    visualization_msgs::MarkerArray marker_array;
    int current_cluster = 0;

    for (const auto& indices : cluster_indices) {
        // Extract the points of the current cluster
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(indices));
        pcl::ExtractIndices<PointT> extract;
        PointCloudT::Ptr cloud_cluster(new PointCloudT);
        extract.setInputCloud(cloud_filtered_final);
        extract.setIndices(cluster_indices_ptr);
        extract.setNegative(false);
        extract.filter(*cloud_cluster);

        // Calculate centroid of the cluster
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        
        // Calculate distance from LiDAR origin to the centroid of the cluster
        double distance = sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] + centroid[2] * centroid[2]);
        ROS_INFO("Cluster %d distance: %f meters", current_cluster, distance);

        // Find the min and max points of the cluster manually
        PointT min_pt, max_pt;
        min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
        max_pt.x = max_pt.y = max_pt.z = std::numeric_limits<float>::lowest();
        
        for (const auto& point : cloud_cluster->points) {
            if (point.x < min_pt.x) min_pt.x = point.x;
            if (point.y < min_pt.y) min_pt.y = point.y;
            if (point.z < min_pt.z) min_pt.z = point.z;
            if (point.x > max_pt.x) max_pt.x = point.x;
            if (point.y > max_pt.y) max_pt.y = point.y;
            if (point.z > max_pt.z) max_pt.z = point.z;
        }

        // Create a marker for the bounding box
        visualization_msgs::Marker marker;
        marker.header.frame_id = input->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "clusters";
        marker.id = current_cluster;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
        marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
        marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = max_pt.x - min_pt.x;
        marker.scale.y = max_pt.y - min_pt.y;
        marker.scale.z = max_pt.z - min_pt.z;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker_array.markers.push_back(marker);

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
    marker_pub.publish(marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "euclidean_cluster_box");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, pointcloud_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("clustered_cloud3", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::spin();
    return 0;
}