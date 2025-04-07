#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <map>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor") {}

    // Function to load the point cloud from a PCD file
    bool loadPointCloud(const std::string& input_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error in loading the file PCD: %s", input_path.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "cloud loaded: %ld points", cloud->points.size());
        return true;
    }

    // Function to apply voxel grid filter to downsample the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "After Voxel filter: %ld points", cloud_filtered->points.size());
        return cloud_filtered;
    }

    // Function to apply pass-through filter based on given x, y, z ranges
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                float x_min, float x_max, 
                                float y_min, float y_max, 
                                float z_min, float z_max) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "After PassThrough Filter: %ld points", cloud_filtered->points.size());
        return cloud_filtered;
    }

    // Function to generate waypoints based on the filtered cloud with a given step size along the x-axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateWaypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, float step_x) {
        std::map<int, pcl::PointXYZ> waypoints_map;

        // Loop over the points and select the lowest z-value for each x position
        for (const auto& point : cloud_filtered->points) {
            int x_bin = static_cast<int>(point.x / step_x); // Bin every 100 mm
            if (waypoints_map.find(x_bin) == waypoints_map.end() || point.z < waypoints_map[x_bin].z) {
                waypoints_map[x_bin] = point;  // Keep the point with the lowest z-value
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& entry : waypoints_map) {
            waypoints->points.push_back(entry.second);
        }

        RCLCPP_INFO(this->get_logger(), "Waypoints generated: %ld", waypoints->points.size());

        // Print first 5 waypoints for debugging
        for (size_t i = 0; i < std::min(5, (int)waypoints->points.size()); i++) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %ld: x=%.3f y=%.3f z=%.3f", 
                        i, waypoints->points[i].x, waypoints->points[i].y, waypoints->points[i].z);
        }
        
        return waypoints;
    }


    bool savePointCloud(const std::string& output_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool binary = false) {
        pcl::PCDWriter cloud_writer;
    
        // Ensure the cloud is unorganized to avoid width*height errors
        cloud->width = cloud->points.size();
        cloud->height = 1; // Set height to 1 to indicate an unordered point cloud
    
        // Save the point cloud (binary or ASCII format)
        if (cloud_writer.write(output_path, *cloud, binary) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error in saving the PCD file: %s", output_path.c_str());
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "Point cloud saved to: %s", output_path.c_str());
        return true;
    }
    
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<PointCloudProcessor>();

    // Define the input and output file paths
    std::string input_pcd = "/home/fra/care_robot_ws/src/cobot_ik/clouds/input/test.pcd";
    std::string output_pcd = "/home/fra/care_robot_ws/src/cobot_ik/clouds/output/post_processing.pcd";
    std::string waypoints_pcd = "/home/fra/care_robot_ws/src/cobot_ik/clouds/output/waypoints.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the point cloud from the PCD file
    if (processor->loadPointCloud(input_pcd, cloud)) {
        // Apply voxel filter to downsample the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered = processor->applyVoxelFilter(cloud, 0.01f);
        
        // Apply pass-through filter to further filter the cloud based on given x, y, z ranges
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = processor->applyPassThroughFilter(cloud_voxel_filtered, 
                                                                                                    -2.0f, 2.0f, 
                                                                                                    -0.5f, 0.3f, 
                                                                                                    0.0f, 1.1f);
        
        // Save the filtered cloud as post_processing.pcd
        processor->savePointCloud(output_pcd, cloud_filtered, false);

        // Generate waypoints from the filtered cloud with a step size of 100mm along the x-axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints = processor->generateWaypoints(cloud_filtered, 0.1f);
        
        // Save the generated waypoints as waypoints.pcd in ASCII format
        processor->savePointCloud(waypoints_pcd, waypoints, false);
    }

    // Shutdown ROS2 node
    rclcpp::shutdown();
    return 0;
}
