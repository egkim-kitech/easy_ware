#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 초기화 변수
double initial_pitch_correction = 0.0;  // 초기 Pitch 보정 값을 저장할 변수
double initial_roll_correction = 0.0;   // 초기 Roll 보정 값을 저장할 변수
double yaw_angle = 0.0;                 // 현재 Yaw 각도를 저장할 변수
ros::Time last_time;

ros::Publisher fix_cloud_pub;
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf_listener;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = imu_msg->header.stamp;
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "livox_frame";

    // LiDAR의 위치 (필요에 따라 수정)
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 1.6;

    // IMU에서 선속도 데이터를 가져옴
    double accel_x = imu_msg->linear_acceleration.x;
    double accel_y = imu_msg->linear_acceleration.y;
    double accel_z = imu_msg->linear_acceleration.z;

    // 현재 pitch 및 roll 각도를 계산 (라디안 단위)
    double current_roll = std::atan2(accel_y, accel_z);
    double current_pitch = std::atan2(-accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z));

    // 지속적인 Pitch 및 Roll 보정을 적용하여 현재 각도를 0으로 만듦
    double corrected_pitch_angle = current_pitch - initial_pitch_correction;
    double corrected_roll_angle = current_roll - initial_roll_correction;

    // 현재 시간
    ros::Time current_time = imu_msg->header.stamp;

    // 시간 차이 계산
    if (last_time.toSec() > 0) {
        double dt = (current_time - last_time).toSec();

        // 각속도(angular_velocity) 기반으로 yaw 회전 각도를 계산
        double angular_velocity_yaw = imu_msg->angular_velocity.z;  // yaw 축의 각속도
        yaw_angle += angular_velocity_yaw * dt;  // yaw 각도를 업데이트
    }

    // 계산된 roll, pitch, yaw 회전을 적용하여 livox_frame을 회전시킴
    tf2::Quaternion rotation_quaternion;
    rotation_quaternion.setRPY(corrected_roll_angle, corrected_pitch_angle, -yaw_angle);

    // 회전된 쿼터니언을 transform에 적용
    transformStamped.transform.rotation.x = rotation_quaternion.x();
    transformStamped.transform.rotation.y = rotation_quaternion.y();
    transformStamped.transform.rotation.z = rotation_quaternion.z();
    transformStamped.transform.rotation.w = rotation_quaternion.w();

    // 변환을 브로드캐스트
    br.sendTransform(transformStamped);

    // 현재 시간을 last_time으로 업데이트
    last_time = current_time;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // Transform the point cloud to the base_link frame
    PointCloudT::Ptr cloud(new PointCloudT);
    PointCloudT::Ptr transformed_cloud(new PointCloudT);

    pcl::fromROSMsg(*cloud_msg, *cloud);

    try {
        geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("base_link", cloud_msg->header.frame_id, ros::Time(0));
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transformStamped.transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Publish the transformed point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = "base_link";
    output.header.stamp = cloud_msg->header.stamp;

    fix_cloud_pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_tf_broadcaster_with_cloud");
    ros::NodeHandle nh;

    // IMU 데이터를 구독하여 콜백 함수 호출
    ros::Subscriber imu_sub = nh.subscribe("/livox/imu", 1, imuCallback);
    
    // PointCloud 데이터를 구독하여 콜백 함수 호출
    ros::Subscriber cloud_sub = nh.subscribe("/livox/lidar", 1, pointCloudCallback);

    // 변환된 포인트 클라우드를 퍼블리시하는 퍼블리셔 생성
    fix_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("fix_cloud", 1);

    // Transform listener 초기화
    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    ros::spin();
    return 0;
}
