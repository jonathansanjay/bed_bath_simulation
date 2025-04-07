#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Subscriber to point cloud of camera
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for point cloud filtered
        filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filtered", 10);

        // Publisher for waypoints (in camera_optical_link frame)
        waypoints_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_waypoints", 10);

        // Publisher for waypoints (in base_link frame as PoseArray)
        base_link_waypoints_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/base_link_waypoints", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr waypoints_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr base_link_waypoints_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert from ROS2 PointCloud2 to PCL
        pcl::fromROSMsg(*msg, *cloud);
        RCLCPP_INFO(this->get_logger(), "Received cloud: %ld points", cloud->points.size());

        // Apply voxel filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered = applyVoxelFilter(cloud, 0.01f);

        // Apply passthrough filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = applyPassThroughFilter(cloud_voxel_filtered, 
                                                                                    -2.0f, 2.0f, 
                                                                                    -0.5f, 0.3f, 
                                                                                    0.0f, 1.1f);

        // Publish the filtered cloud
        publishPointCloud(cloud_filtered, filtered_cloud_publisher_, "camera_optical_link");

        // Generate waypoints
        pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints = generateWaypoints(cloud_filtered, 0.1f);

        // Publish waypoints in camera_optical_link frame
        publishPointCloud(waypoints, waypoints_publisher_, "camera_optical_link");

        // Transform and publish waypoints in base_link frame
        transformAndPublishWaypoints(waypoints);
    }

    void transformAndPublishWaypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& waypoints) {
        geometry_msgs::msg::TransformStamped transform_stamped;

        try {
            transform_stamped = tf_buffer_.lookupTransform("base_link", "camera_optical_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform waypoints to base_link: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "base_link";
        pose_array.header.stamp = this->now();

        for (const auto& point : waypoints->points) {
            geometry_msgs::msg::PointStamped point_camera;
            point_camera.header.frame_id = "camera_optical_link";
            point_camera.header.stamp = this->now();
            point_camera.point.x = point.x;
            point_camera.point.y = point.y;
            point_camera.point.z = point.z;

            geometry_msgs::msg::PointStamped point_base;
            tf2::doTransform(point_camera, point_base, transform_stamped);

            geometry_msgs::msg::Pose pose;
            pose.position = point_base.point;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0; // Default orientation (can be ignored by the planner)

            pose_array.poses.push_back(pose);
        }

        base_link_waypoints_publisher_->publish(pose_array);
        RCLCPP_INFO(this->get_logger(), "Published %ld transformed waypoints to /base_link_waypoints", waypoints->points.size());
    }

    // Function to apply Voxel Grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*cloud_filtered);
        return cloud_filtered;
    }

    // Function to apply PassThrough filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                                               float x_min, float x_max, 
                                                               float y_min, float y_max, 
                                                               float z_min, float z_max) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);

        // Filter for X-axis

        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*cloud_filtered);

        // FFilter for Y-axis

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*cloud_filtered);

        // Filter for Z-axis

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_filtered);

        return cloud_filtered;
    }

    // Function to publish a PointCloud
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher, 
                           const std::string& frame_id) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = frame_id;
        publisher->publish(cloud_msg);
    }

    // function to generate waypoints from a filtered PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateWaypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, float step_x) {
        std::map<int, pcl::PointXYZ> waypoints_map;

        // Create a waypoint for each X-axis bin
        for (const auto& point : cloud_filtered->points) {
            int x_bin = static_cast<int>(point.x / step_x);
            if (waypoints_map.find(x_bin) == waypoints_map.end() || point.z < waypoints_map[x_bin].z) {
                waypoints_map[x_bin] = point;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& entry : waypoints_map) {
            waypoints->points.push_back(entry.second);
        }

        return waypoints;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


