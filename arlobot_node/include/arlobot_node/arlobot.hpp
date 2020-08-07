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

namespace berrevoets
{
    namespace arlobot
    {
        class Arlobot : public rclcpp::Node
        {
        public:
            explicit Arlobot(const std::string &usb_port);
            virtual ~Arlobot(){};

            void run();

        private:
            rclcpp::Node::SharedPtr node_handle_;
        };
    } // namespace arlobot
} // namespace berrevoets

#endif // ARLOBOT_HPP
