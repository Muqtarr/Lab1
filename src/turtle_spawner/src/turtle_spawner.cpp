#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

class TurtleSpawner : public rclcpp::Node
{
public:
    TurtleSpawner() : Node("turtle_spawner")
    {
        // Create a client for the "/spawn" service
        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the spawn service...");
        }

        // Create a request
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 1.0;
        request->y = 5.0;
        request->theta = 0.0;
        request->name = "Turtle_MUKHTARVALERIYAAZHARDARYN";  

        // Call the service
        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture result)
            {
                if (result.valid())
                {
                    RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", result.get()->name.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle.");
                }
            });
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
