#include "arlobot_node/arlobot.hpp"

using namespace berrevoets;
using namespace std::chrono_literals;
using namespace arlobot;

Arlobot::Arlobot(const std::string &usb_port) : Node("arlobot_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(get_logger(), "Init Arlobot Node Main");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    init_dynamixel_sdk_wrapper(usb_port);

    check_device_status();

    run();
}

void Arlobot::init_dynamixel_sdk_wrapper(const std::string &usb_port)
{
    DynamixelSDKWrapper::Device opencr = {usb_port, 200, 1000000, 2.0f};

    this->declare_parameter("opencr.id");
    this->declare_parameter("opencr.baud_rate");
    this->declare_parameter("opencr.protocol_version");

    this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
    this->get_parameter_or<uint32_t>("opencr.baud_rate", opencr.baud_rate, 1000000);
    this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

    RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

    dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);

    dxl_sdk_wrapper_->init_read_memory(
        extern_control_table.millis.addr,
        (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) + extern_control_table.profile_acceleration_right.length);
}

void Arlobot::check_device_status()
{
    if (dxl_sdk_wrapper_->is_connected_to_device())
    {
        std::string sdk_msg;
        uint8_t reset = 1;

        dxl_sdk_wrapper_->set_data_to_device(
            extern_control_table.imu_re_calibration.addr,
            extern_control_table.imu_re_calibration.length,
            &reset,
            &sdk_msg);

        RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
        rclcpp::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO(this->get_logger(), "Calibration End");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
        rclcpp::shutdown();
        return;
    }

    const int8_t NOT_CONNECTED_MOTOR = -1;

    int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
        extern_control_table.device_status.addr,
        extern_control_table.device_status.length);

    switch (device_status)
    {
    case NOT_CONNECTED_MOTOR:
        RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
        break;

    default:
        break;
    }
}

void Arlobot::run()
{
    RCLCPP_INFO(get_logger(), "Run!");
}
