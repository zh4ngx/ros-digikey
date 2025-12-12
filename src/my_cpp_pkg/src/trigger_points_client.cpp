#include <random>

#include "rclcpp/rclcpp.hpp"

#include "my_interfaces/srv/trigger_points.hpp"
#include "geometry_msgs/msg/point.hpp"

/**
 * Client example that periodically calls the add_two_ints service.
 */
 class TriggerPointsClient : public rclcpp::Node
 {
     public:
      /**
       * Consturctor
       */
       TriggerPointsClient() : Node("trigger_points_client")
       {
           // Create a client object
           client_ = this->create_client<my_interfaces::srv::TriggerPoints>("trigger_points");

           // Wait for service
           while (!client_ -> wait_for_service(std::chrono::seconds(2)))
           {
               RCLCPP_INFO(this->get_logger(), "Waiting for service");
           }

           // Seed random number generator with current time
           std::srand(std::time(nullptr));

           // Periodically call service
           timer_ = this->create_wall_timer(std::chrono::seconds(2),
               std::bind(&TriggerPointsClient::call_service, this));
       };

       private:

       /*
        * Send request to server askin it to add two ints
        */
        void call_service()
        {
                        // Create request object
            auto request = std::make_shared<my_interfaces::srv::TriggerPoints::Request>();
            request->num_points = std::rand() % 11;

            // Send request and wait for response
            client_->async_send_request(
                request,
                std::bind(&TriggerPointsClient::call_service_callback, this, std::placeholders::_1));
        }

        /**
         * Log result when recv from server
         */
        void call_service_callback(rclcpp::Client<my_interfaces::srv::TriggerPoints>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Success: %s", response->success ? "true" : "false");
            for (const auto &point : response->points)
            {
                RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f)", point.x, point.y, point.z);
            }
        }

        // Declare member variables
        rclcpp::Client<my_interfaces::srv::TriggerPoints>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
 };


/**
 * Main entrypoint
 */
 int main(int argc, char * argv[])
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<TriggerPointsClient>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
