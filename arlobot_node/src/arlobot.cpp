#include "arlobot_node/arlobot.hpp"

using namespace berrevoets;
using namespace std::chrono_literals;
using namespace arlobot;

Arlobot::Arlobot(const std::string & usb_port) : Node("arlobot_node",rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "Init Arlobot Node Main");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *){});

    run();
}

void Arlobot::run()
{
    RCLCPP_INFO(get_logger(), "Run!");
}
