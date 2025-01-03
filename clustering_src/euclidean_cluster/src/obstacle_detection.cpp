#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
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
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cmath>
#include <limits>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher transformed_cloud_pub;
ros::Publisher filtered_cloud_pub;
ros::Publisher marker_pub;
ros::Publisher obstacle_detection_pub;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*input, *cloud);

    // livox_frame에서 30도 pitch 기울기를 적용하여 변환
    Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
    float pitch_angle = 30.0 * M_PI / 180.0;
    transform_eigen.rotate(Eigen::AngleAxisf(pitch_angle, Eigen::Vector3f::UnitY()));
    PointCloudT::Ptr cloud_transformed(new PointCloudT);
    pcl::transformPointCloud(*cloud, *cloud_transformed, transform_eigen);

    // 기울기만 적용된 포인트 클라우드를 퍼블리시
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*cloud_transformed, transformed_cloud_msg);
    transformed_cloud_msg.header.frame_id = "livox_frame";
    transformed_cloud_msg.header.stamp = ros::Time::now();
    transformed_cloud_pub.publish(transformed_cloud_msg);

    // Downsample the cloud using VoxelGrid filter
    pcl::VoxelGrid<PointT> vg;
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    vg.setInputCloud(cloud_transformed);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);

    // 첫 번째 CropBox 필터로 특정 범위의 포인트 제거
    pcl::CropBox<PointT> cropBoxFilter1;
    cropBoxFilter1.setInputCloud(cloud_filtered);
    cropBoxFilter1.setMin(Eigen::Vector4f(0.0, -1.0, -1.5, 1.0));
    cropBoxFilter1.setMax(Eigen::Vector4f(5.0, 1.0, 0.5, 1.0));
    PointCloudT::Ptr cloud_cropped(new PointCloudT);
    cropBoxFilter1.filter(*cloud_cropped);

    // 두 번째 CropBox 필터로 추가 범위의 포인트 제거
    pcl::CropBox<PointT> cropBoxFilter2;
    cropBoxFilter2.setInputCloud(cloud_cropped);
    cropBoxFilter2.setMin(Eigen::Vector4f(-0.3, -0.35, -1.65, 1.0));
    cropBoxFilter2.setMax(Eigen::Vector4f(0.7, 0.35, 0.1, 1.0));
    cropBoxFilter2.setNegative(true); // 범위 내의 포인트를 제거
    PointCloudT::Ptr cloud_filtered_final(new PointCloudT);
    cropBoxFilter2.filter(*cloud_filtered_final);

    // 필터링된 포인트 클라우드 퍼블리시
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*cloud_filtered_final, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "livox_frame";
    filtered_cloud_msg.header.stamp = ros::Time::now();
    filtered_cloud_pub.publish(filtered_cloud_msg);

    // 클러스터링을 적용
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered_final);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered_final);
    ec.extract(cluster_indices);

    visualization_msgs::MarkerArray marker_array;
    int current_cluster = 0;
    bool obstacle_detected = false;

    for (const auto& indices : cluster_indices) {
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
        double distance = centroid[0] ;
        ROS_INFO("Distance: %f meters", distance);

        // 장애물 감지
        if (distance <= 3.0) {
            obstacle_detected = true;
        }

        // 클러스터의 최소 및 최대 좌표 수동 계산
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
        marker.header.frame_id = "livox_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "clusters";
        marker.id = current_cluster;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
        marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
        marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
        marker.scale.x = max_pt.x - min_pt.x;
        marker.scale.y = max_pt.y - min_pt.y;
        marker.scale.z = max_pt.z - min_pt.z;
        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker_array.markers.push_back(marker);
        current_cluster++;
    }

    marker_pub.publish(marker_array);

    std_msgs::Float64MultiArray obstacle_msg;
    obstacle_msg.data.push_back(obstacle_detected ? 1.0 : 0.0);
    obstacle_detection_pub.publish(obstacle_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_frame_processing");
    ros::NodeHandle nh("~");

    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    obstacle_detection_pub = nh.advertise<std_msgs::Float64MultiArray>("obstacle_detection", 1);

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, pointcloud_callback);

    ros::spin();
    return 0;
}
