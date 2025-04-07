#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

//#include "ros2_linkattacher/gazebo_link_attacher.hpp"
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;

class PickAndCare
{
public:
    PickAndCare(rclcpp::Node::SharedPtr node)
        : move_group(node, "arm"),
          gripper(node, "gripper"),
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndCare")),

          // link attacher
          node_(node)
    {
        
        move_group.setPoseReferenceFrame("base_link");

        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        // Subscriber to /base_link_waypoints topic
        waypoint_subscriber = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/base_link_waypoints", 10,
            std::bind(&PickAndCare::waypoints_callback, this, std::placeholders::_1));
    }

    void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        received_waypoints = msg->poses;
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.01);
        gripper.move();
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("finger_right_joint", 0.0);
        gripper.move();
    }

    void pick()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(true);
        move_group.setGoalTolerance(0.03);

        // Get the pose
        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.14, 0, 0);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.08;
        pick_pose.position.y = 0.358;
        pick_pose.position.z = 0.150;

        move_group.setPoseTarget(pick_pose, "link6");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");
            attachObject();

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pick_pose);

            geometry_msgs::msg::Pose post_pick_pose = pick_pose;
            post_pick_pose.position.z += 0.1;
            waypoints.push_back(post_pick_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.02;

            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            if (fraction < 0.99)
            {
                RCLCPP_ERROR(logger, "Cartesian path planning failed. Only %.2f%% of path completed", fraction * 100.0);
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group.execute(cartesian_plan);

            RCLCPP_INFO(logger, "Post-pick motion completed using Cartesian path.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
    }

    void care()
    {
        // if (received_waypoints.empty())
        // {
        //     RCLCPP_ERROR(logger, "No waypoints received!");
        //     return;
        // }

        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);  
        move_group.allowReplanning(true);  
        move_group.setGoalTolerance(0.03);
        // Get the pose 
        geometry_msgs::msg::Pose caring_pose;
        tf2::Quaternion orientation_caring;
        orientation_caring.setRPY(-3.14, 0, 0);
        caring_pose.orientation = tf2::toMsg(orientation_caring);
        caring_pose.position.x = 0.5;
        caring_pose.position.y = 0;
        caring_pose.position.z = 0.7;

        move_group.setPoseTarget(caring_pose, "link6");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");
        if (success)
        {

            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
            RCLCPP_INFO(logger, "caring position...");
            move_group.setMaxVelocityScalingFactor(0.1);
            move_group.setMaxAccelerationScalingFactor(0.1);
            move_group.setPlanningTime(10.0);  
            move_group.allowReplanning(true);  
            move_group.setGoalTolerance(0.03);

            // Find the waypoint closest to y=0
            geometry_msgs::msg::Pose closest_waypoint;
            double min_distance = std::numeric_limits<double>::max();
            for (const auto &pose : received_waypoints)
            {
                double distance = std::abs(pose.position.y); // Calculate distance to y=0
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_waypoint = pose;
                }
            }

            closest_waypoint.orientation = tf2::toMsg(orientation_caring);

            double z_offset = 0.2;
            closest_waypoint.position.z += z_offset;



            RCLCPP_INFO(logger, "Closest waypoint: Position(x: %.2f, y: %.2f, z: %.2f), Orientation(x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
            closest_waypoint.position.x, closest_waypoint.position.y, closest_waypoint.position.z,
            closest_waypoint.orientation.x, closest_waypoint.orientation.y, closest_waypoint.orientation.z, closest_waypoint.orientation.w);



            move_group.setPoseTarget(closest_waypoint, "link6");

            // Planning
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(logger, "Visualizing care plan: %s", success ? "SUCCESS" : "FAILED");

            if (success)
            {
                move_group.move();
                RCLCPP_INFO(logger, "Motion execution completed.");
            }
            else
            {
                RCLCPP_ERROR(logger, "Motion planning for care failed!");
            }
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
        } 
        
    }

    void sponge_back_pose()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        geometry_msgs::msg::Pose pose_back;
        tf2::Quaternion orientation_pose_back;
        orientation_pose_back.setRPY(-3.14, 0, 0);
        pose_back.orientation = tf2::toMsg(orientation_pose_back);
        pose_back.position.x = 0.08;
        pose_back.position.y = 0.358;
        pose_back.position.z = 0.225;

        move_group.setPoseTarget(pose_back, "link6");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Place back execution completed.");
            detachObject();
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
    }

    void back_home()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setNamedTarget("home");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(logger, "Planning succeeded, executing...");
            move_group.execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning failed.");
        }
    }

    void addCollisionObjects()
    {
        // std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.resize(3);

        // Conveyor
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "world";
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[0].primitives[0].dimensions = {0.8, 2.0, 1.0};
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.6;
        collision_objects[0].primitive_poses[0].position.y = 0.0;
        collision_objects[0].primitive_poses[0].position.z = 0.5;
        collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

        // Place table
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = "world";
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        collision_objects[1].primitives[0].dimensions = {2, 0.8, 1.0};
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0.0;
        collision_objects[1].primitive_poses[0].position.y = 0.6;
        collision_objects[1].primitive_poses[0].position.z = 0.5;
        collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

        // Basement robot stands on
        collision_objects[2].id = "basement";
        collision_objects[2].header.frame_id = "world";
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        collision_objects[2].primitives[0].dimensions = {1, 0.2}; 
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0.0;
        collision_objects[2].primitive_poses[0].position.y = 0.0;
        collision_objects[2].primitive_poses[0].position.z = 0.48;
        collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

        // Object to pick
        // collision_objects[3].id = "sponge";
        // collision_objects[3].header.frame_id = "world";
        // collision_objects[3].primitives.resize(1);
        // collision_objects[3].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        // collision_objects[3].primitives[0].dimensions = {0.05, 0.05, 0.05};
        // collision_objects[3].primitive_poses.resize(1);
        // collision_objects[3].primitive_poses[0].position.x = 0.08;
        // collision_objects[3].primitive_poses[0].position.y = 0.37;
        // collision_objects[3].primitive_poses[0].position.z = 1.025;
        // collision_objects[3].operation = moveit_msgs::msg::CollisionObject::ADD;

        // Add the object to the scene
        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(logger, "Collision objects added to the planning scene.");
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "link6";
        request->model2_name = "sponge";
        request->link2_name = "link_1";

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "cobot";
        request->link1_name = "link6";
        request->model2_name = "sponge";
        request->link2_name = "link_1";

        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    rclcpp::Logger logger;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber;
    std::vector<geometry_msgs::msg::Pose> received_waypoints;
};

int main(int argc, char **argv)
{
    // initialization ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pick_and_care_node");

    // Create PickAndCare
    PickAndCare pick_and_care(node);

    // Add collision objects
    pick_and_care.addCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Get the sponge
    pick_and_care.pick();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Care execution
    pick_and_care.care();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Put the sponge back
    pick_and_care.sponge_back_pose();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // back to home
    pick_and_care.back_home();
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Spegni ROS2
    rclcpp::shutdown();
    return 0;
}
