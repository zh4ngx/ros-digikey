#include <random>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


/**
 * Client example that periodically calls the add_two_ints service.
 */
 class MinimalClient : public rclcpp::Node
 {
     public:
      /**
       * Consturctor
       */
       MinimalClient() : Node("minimal_client")
       {
           // Create a client object
           client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_ints");

           // Wait for service
           while (!client_ -> wait_for_service(std::chrono::seconds(2)))
           {
               RCLCPP_INFO(this->get_logger(), "Waiting for service");
           }

           // Seed random number generator with current time
           std::srand(std::time(nullptr));

           // Periodically call service
           timer_ = this->create_wall_timer(std::chrono::seconds(2),
               std::bind(&MinimalClient::call_service, this));
       };

       private:

       /*
        * Send request to server askin it to add two ints
        */
        void call_service()
        {
            // Generate random numbers
            int a = std::rand() % 11;
            int b = std::rand() % 11;

            // Create request object
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            // Send request and wait for response
            client_->async_send_request(
                request,
                std::bind(&MinimalClient::call_service_callback, this, std::placeholders::_1));
        }

        /**
         * Log result when recv from server
         */
        void call_service_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %d", (int)response->sum);
        }

        // Declare member variables
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
 };


/**
 * Main entrypoint
 */
 int main(int argc, char * argv[])
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<MinimalClient>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
