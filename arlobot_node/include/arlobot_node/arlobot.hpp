#ifndef ARLOBOT_HPP
#define ARLOBOT_HPP

#include <array>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include "arlobot_node/control_table.hpp"
#include "arlobot_node/dynamixel_sdk_wrapper.hpp"

namespace berrevoets
{
    namespace arlobot
    {
        extern const ControlTable extern_control_table;
        class Arlobot : public rclcpp::Node
        {
        public:
            typedef struct
            {
                float separation;
                float radius;
            } Wheels;

            typedef struct
            {
                float profile_acceleration_constant;
                float profile_acceleration;
            } Motors;

            explicit Arlobot(const std::string &usb_port);
            virtual ~Arlobot(){};

            Wheels *get_wheels();
            Motors *get_motors();

        private:
            void init_dynamixel_sdk_wrapper(const std::string &usb_port);
            void check_device_status();
            void run();

            std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
            rclcpp::Node::SharedPtr node_handle_;
        };
    } // namespace arlobot
} // namespace berrevoets

#endif // ARLOBOT_HPP
