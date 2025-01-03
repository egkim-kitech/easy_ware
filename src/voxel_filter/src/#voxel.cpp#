#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <vector>

class VoxelGridGenerator {
public:
    VoxelGridGenerator() : nh("~") {
        distance = 10;  // (meter)
        voxel_size = 0.1;  // leaf size
        grid_size_x = 200;  // number of grid x
        grid_size_y = 200;
        grid_size_z = 20;
        voxel_grid = std::vector<std::vector<std::vector<uint8_t>>>(
            grid_size_x * 2 + 1,
            std::vector<std::vector<uint8_t>>(
                grid_size_y * 2 + 1,
                std::vector<uint8_t>(grid_size_z * 2 + 1, 0)));

        sub = nh.subscribe("/velodyne_points", 1, &VoxelGridGenerator::callbackPointCloud, this);
        pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel_points", 1);

        // Precompute grid bounds
        grid_bounds = {
            {-grid_size_x, grid_size_x},
            {-grid_size_y, grid_size_y},
            {-grid_size_z, grid_size_z}};
    }

    void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Make a non-const copy of the PointCloud2 message
    sensor_msgs::PointCloud2 mutable_msg = *msg;

    voxel_grid.clear();
    voxel_grid.resize(grid_size_x * 2 + 1, std::vector<std::vector<uint8_t>>(
        grid_size_y * 2 + 1, std::vector<uint8_t>(grid_size_z * 2 + 1, 0)));

    sensor_msgs::PointCloud2Iterator<float> iter_x(mutable_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(mutable_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(mutable_msg, "z");

    auto[x_min, x_max] = grid_bounds[0];
    auto[y_min, y_max] = grid_bounds[1];
    auto[z_min, z_max] = grid_bounds[2];

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        int voxel_x = std::floor(*iter_x / voxel_size);
        int voxel_y = std::floor(*iter_y / voxel_size);
        int voxel_z = std::floor(*iter_z / voxel_size);

        if (x_min <= voxel_x && voxel_x < x_max &&
            y_min <= voxel_y && voxel_y < y_max &&
            z_min <= voxel_z && voxel_z < z_max) {
            voxel_grid[voxel_x + grid_size_x][voxel_y + grid_size_y][voxel_z + grid_size_z] = 1;
        }
    }

    voxelPublisher(msg->header);
    }

    void voxelPublisher(const std_msgs::Header& header) {
        sensor_msgs::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;
        cloud.width = 1;
        cloud.is_dense = false;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(voxel_grid.size() * voxel_grid[0].size() * voxel_grid[0][0].size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        auto[x_min, x_max] = grid_bounds[0];
        auto[y_min, y_max] = grid_bounds[1];
        auto[z_min, z_max] = grid_bounds[2];

        for (int x = x_min; x < x_max; ++x) {
            for (int y = y_min; y < y_max; ++y) {
                for (int z = z_min; z < z_max; ++z) {
                    if (voxel_grid[x + grid_size_x][y + grid_size_y][z + grid_size_z] > 0) {
                        *iter_x = (x) * voxel_size;
                        *iter_y = (y) * voxel_size;
                        *iter_z = (z) * voxel_size;
                        ++iter_x;
                        ++iter_y;
                        ++iter_z;
                    }
                }
            }
        }

        pub.publish(cloud);
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    double distance;
    double voxel_size;
    int grid_size_x;
    int grid_size_y;
    int grid_size_z;
    std::vector<std::vector<std::vector<uint8_t>>> voxel_grid;
    std::vector<std::pair<int, int>> grid_bounds;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "jaeguk_first_voxel");
    VoxelGridGenerator voxel;
    voxel.run();
    return 0;
}
