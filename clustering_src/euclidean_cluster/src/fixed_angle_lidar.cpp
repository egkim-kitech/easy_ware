#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>  // TransformBroadcaster 헤더 추가
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h> // tf2와 Eigen 변환을 위한 헤더 추가
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
#include <std_msgs/Float64MultiArray.h>  // Float64MultiArray 헤더 추가
#include <iostream>
#include <cmath>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Publisher marker_pub;
ros::Publisher transformed_cloud_pub; // 변환된 포인트 클라우드를 퍼블리시하기 위한 퍼블리셔
ros::Publisher not_filter_point_pub;  // 필터링 전 포인트 클라우드를 퍼블리시하기 위한 퍼블리셔
ros::Publisher obstacle_detection_pub; // 장애물 감지를 위한 퍼블리셔 추가
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf_listener;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::fromROSMsg(*input, *cloud);

    // Transform the cloud from livox_frame to base_link
    PointCloudT::Ptr cloud_transformed(new PointCloudT);
    try {
        geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("cluster_frame", input->header.frame_id, ros::Time(0), ros::Duration(1.0));
        Eigen::Affine3d transform_eigen;
        transform_eigen = tf2::transformToEigen(transformStamped.transform);
        pcl::transformPointCloud(*cloud, *cloud_transformed, transform_eigen); // pcl의 transformPointCloud 사용
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // 'cloud_transformed' 클라우드를 'not_filter_point'로 발행
    sensor_msgs::PointCloud2 not_filter_point_msg;
    pcl::toROSMsg(*cloud_transformed, not_filter_point_msg);
    not_filter_point_msg.header.frame_id = "cluster_frame";
    not_filter_point_msg.header.stamp = ros::Time::now();
    not_filter_point_pub.publish(not_filter_point_msg);

    // Downsample the cloud using VoxelGrid filter
    pcl::VoxelGrid<PointT> vg;
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    vg.setInputCloud(cloud_transformed);
    vg.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust the leaf size for downsampling
    vg.filter(*cloud_filtered);

    // Remove points within a radius of 0.8m from the origin
    pcl::CropBox<PointT> cropBoxFilter1;
    cropBoxFilter1.setInputCloud(cloud_filtered);
    cropBoxFilter1.setMin(Eigen::Vector4f(-0.4, -0.4, -1.5, 1.0));
    cropBoxFilter1.setMax(Eigen::Vector4f(0.7, 0.4, 3.5, 1.0));
    cropBoxFilter1.setNegative(true); // Remove points inside the box
    PointCloudT::Ptr cloud_cropped(new PointCloudT);
    cropBoxFilter1.filter(*cloud_cropped);

    // Apply pass-through filter based on base_link frame (CropBox Filter)
    pcl::CropBox<PointT> cropBoxFilter2;
    cropBoxFilter2.setInputCloud(cloud_cropped);
    cropBoxFilter2.setMin(Eigen::Vector4f(0.0, -0.75, 0.1, 1.0));  // base_link 기준으로 필터 범위 설정
    cropBoxFilter2.setMax(Eigen::Vector4f(20.0, 0.75, 20.0, 1.0));
    PointCloudT::Ptr cloud_filtered_final(new PointCloudT);
    cropBoxFilter2.filter(*cloud_filtered_final);

    // 변환된 포인트 클라우드 중 CropBox 필터가 적용된 클라우드를 퍼블리시
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*cloud_filtered_final, transformed_cloud_msg);
    transformed_cloud_msg.header.frame_id = "cluster_frame";
    transformed_cloud_msg.header.stamp = ros::Time::now();
    transformed_cloud_pub.publish(transformed_cloud_msg);

    // Check if cloud_filtered_final is empty before proceeding with clustering
    if (cloud_filtered_final->empty()) {
        return;
    }

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
    bool obstacle_detected = false; // 장애물 감지 플래그

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

        // Calculate distance from base_link origin to the centroid of the cluster
        double distance = sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1] );
        ROS_INFO("Cluster %d distance: %f meters", current_cluster, distance);

        // 장애물이 (n)m 이하인 경우 감지
        if (distance <= 1.8) {
            obstacle_detected = true;
        }

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
        marker.header.frame_id = "cluster_frame";
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
    output.header.frame_id = "cluster_frame";

    marker_pub.publish(marker_array);

    // 장애물 감지 결과 발행
    std_msgs::Float64MultiArray obstacle_msg;
    if (obstacle_detected) {
        obstacle_msg.data.push_back(1.0); // 장애물이 감지되었음을 나타내는 1.0
        ROS_INFO("Obstacle detected within 3 meters!");
    } else {
        obstacle_msg.data.push_back(0.0); // 장애물이 감지되지 않음을 나타내는 0.0
    }
    obstacle_detection_pub.publish(obstacle_msg);
}

void publish_transform(const ros::TimerEvent&, double z_value, double p_angle) {
    static tf2_ros::TransformBroadcaster br;  // 브로드캐스터 초기화
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "cluster_frame";
    transformStamped.child_frame_id = "livox_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = z_value;  // Launch 파일에서 설정된 z 값 사용

    // Roll 회전 적용
    tf2::Quaternion quaternion;
    double pitch_angle = p_angle * M_PI / 180.0;  
    quaternion.setRPY(0.0, pitch_angle, 0.0);

    // 쿼터니언을 Transform에 적용
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster_with_clustering");
    ros::NodeHandle nh("~");

    // Launch 파일에서 매개변수 받아오기
    double z_value, p_angle;
    nh.param("z_value", z_value, 1.5);  // 기본값 1.6
    nh.param("p_angle", p_angle, 30.0);  // 기본값 30도

    // tf 리스너 초기화
    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, pointcloud_callback);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);  // 변환된 포인트 클라우드 퍼블리셔 추가
    not_filter_point_pub = nh.advertise<sensor_msgs::PointCloud2>("not_filter_point", 1);  // 필터링 전 포인트 클라우드 퍼블리셔 추가
    obstacle_detection_pub = nh.advertise<std_msgs::Float64MultiArray>("obstacle_detection", 1); // 장애물 감지를 위한 퍼블리셔 추가

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(publish_transform, _1, z_value, p_angle));

    ros::spin();
    return 0;
}
