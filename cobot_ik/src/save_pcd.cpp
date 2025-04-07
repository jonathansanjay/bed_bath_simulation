#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PointCloudSaver : public rclcpp::Node {
public:
    PointCloudSaver() : Node("point_cloud_data_saving") {
        // subscribing to the topic that the camera publishes
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            std::bind(&PointCloudSaver::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nodo avviato, in attesa della nuvola di punti...");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        std::string path = "/home/fra/care_robot_ws/src/cobot_ik/clouds/input/";
        std::string filename = path + "test.pcd";

        // Salva la nuvola di punti
        pcl::PCDWriter cloud_writer;
        if (cloud_writer.write<pcl::PointXYZ>(filename, cloud, false) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error in saving PCD!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "cloud saved in: %s", filename.c_str());
        rclcpp::shutdown();  // stop the node 
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

