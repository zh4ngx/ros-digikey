#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


/**
 * Subscriber example that prints message to console
 */
class MinimalSubscriber : public rclcpp::Node
{
public:

/**
 * Constructor (call Node class with name)
 */
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        // create subscription
        subscription_ = this->create_subscription<example_interfaces::msg::String>(
            "my_topic", 10, std::bind(
                &MinimalSubscriber::listener_callback,
                this,
                std::placeholders::_1));
    }

private:
    void listener_callback(const example_interfaces::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscription_;
};

/**
 * Main function
 */
int main(int argc, char * argv[])
{
    /** Initialize ROS */
    rclcpp::init(argc, argv);

    // Create a shared pointer to the MinimalSubscriber node
    auto node = std::make_shared<MinimalSubscriber>();

    // Spin the node to process incoming messages
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
