#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/accelerometer.hpp"


/**
 * Subscriber example that prints message to console
 */
class AccelSubscriber : public rclcpp::Node
{
public:

/**
 * Constructor (call Node class with name)
 */
    AccelSubscriber()
    : Node("accel_subscriber")
    {
        // create subscription
        subscription_ = this->create_subscription<my_interfaces::msg::Accelerometer>(
            "my_topic", 10, std::bind(
                &AccelSubscriber::listener_callback,
                this,
                std::placeholders::_1));
    }

private:
    void listener_callback(const my_interfaces::msg::Accelerometer & msg) const
    {
        RCLCPP_INFO(this->get_logger(),
            "Accelerometer data: x=%f, y=%f, z=%f", msg.x, msg.y, msg.z);
    }
    rclcpp::Subscription<my_interfaces::msg::Accelerometer>::SharedPtr subscription_;
};

/**
 * Main function
 */
int main(int argc, char * argv[])
{
    /** Initialize ROS */
    rclcpp::init(argc, argv);

    // Create a shared pointer to the MinimalSubscriber node
    auto node = std::make_shared<AccelSubscriber>();

    // Spin the node to process incoming messages
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
