#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller"), pose_received_(false)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle_mukhtar/cmd_vel", 10);

        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle_mukhtar/teleport_absolute");

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle_mukhtar/pose", 10,
            std::bind(&TurtleController::poseCallback, this, std::placeholders::_1)
        );

        // Step 1: Kill default turtle
        if (killTurtle("turtle1")) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed turtle1.");
        } else {
            RCLCPP_WARN(this->get_logger(), "turtle1 might not exist.");
        }

        // Step 2: Spawn a new turtle at the center
        if (spawnTurtle("turtle_mukhtar", 5.5, 5.5, 0.0)) {
            RCLCPP_INFO(this->get_logger(), "Spawned turtle_mukhtar at center.");
        }

        // Wait for initial pose
        waitForPose();

        // Step 3: Teleport turtle to the bottom-left corner (1,1)
        teleportTurtle(1.0, 1.0, 0.0);
        waitForPoseUpdate(1.0, 1.0, 0.0);

        // Step 4: Move in a square path
        moveInSquare();

        // Step 5: Move in a triangular path
        moveInTriangle();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

    turtlesim::msg::Pose current_pose_;
    std::atomic_bool pose_received_;

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void waitForPose() {
        while (!pose_received_ && rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void waitForPoseUpdate(float x, float y, float theta) {
        auto start = this->now();
        while ((this->now() - start).seconds() < 5.0) {
            if (std::abs(current_pose_.x - x) < 0.1 &&
                std::abs(current_pose_.y - y) < 0.1 &&
                std::abs(current_pose_.theta - theta) < 0.1) {
                break;
            }
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool killTurtle(const std::string &name) {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;
        auto future = kill_client_->async_send_request(request);
        return future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    }

    bool spawnTurtle(const std::string &name, float x, float y, float theta) {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;
        auto future = spawn_client_->async_send_request(request);
        return future.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    }

    bool teleportTurtle(float x, float y, float theta) {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto future = teleport_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to teleport turtle");
            return false;
        }
        return true;
    }

    void rotateToAngle(float target_angle) {
        geometry_msgs::msg::Twist msg;
        const float angular_speed = 1.5;
        const float angle_tolerance = 0.01;

        auto start_time = this->now();
        while (rclcpp::ok() && (this->now() - start_time).seconds() < 5.0) {
            float current_angle = current_pose_.theta;
            float angle_diff = target_angle - current_angle;

            angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

            if (std::abs(angle_diff) < angle_tolerance) {
                break;
            }

            msg.angular.z = angular_speed * (angle_diff > 0 ? 1.0 : -1.0);
            cmd_vel_pub_->publish(msg);

            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);
    }

    void moveStraight(float distance, float speed) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = speed;
        msg.angular.z = 0.0;

        double time_to_reach = distance / speed;

        auto start_time = this->now();
        while (rclcpp::ok() && (this->now() - start_time).seconds() < time_to_reach) {
            cmd_vel_pub_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        msg.linear.x = 0.0;
        cmd_vel_pub_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void moveToCorner(float x, float y) {
        RCLCPP_INFO(this->get_logger(), "Moving to (%.2f, %.2f)", x, y);

        float current_x = current_pose_.x;
        float current_y = current_pose_.y;
        float delta_x = x - current_x;
        float delta_y = y - current_y;
        float desired_angle = std::atan2(delta_y, delta_x);

        rotateToAngle(desired_angle);
        float distance = std::hypot(delta_x, delta_y);
        moveStraight(distance, 1.0);
    }

    void moveInSquare() {
        RCLCPP_INFO(this->get_logger(), "Starting square trajectory");
        moveToCorner(10.0, 1.0);
        moveToCorner(10.0, 10.0);
        moveToCorner(1.0, 10.0);
        moveToCorner(1.0, 1.0);
    }

    void moveInTriangle() {
        RCLCPP_INFO(this->get_logger(), "Starting triangular trajectory");
        moveToCorner(10.0, 1.0);
        moveToCorner(10.0, 10.0);
        moveToCorner(1.0, 1.0);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}