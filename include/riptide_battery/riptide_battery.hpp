#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <rtac_asio/Stream.h>
#include <rtac_asio/SerialStream.h>


class RiptideBattery: public rclcpp::Node {
    
    public:

        using Msg = sensor_msgs::msg::BatteryState;
        
        // Constructor
        RiptideBattery();

    private:

        // Pointer to the serial stream
        rtac::asio::Stream::Ptr serial_;

        // Port of the serial communication
        std::string port_;

        // Baud rate of the serial communication
        uint32_t baud_rate_;

        // Buffer
        std::string buffer_;

        // Message
        Msg battery_msg_;

        // Publisher
        std::shared_ptr<rclcpp::Publisher<Msg>> battery_card_publisher_ = nullptr;

        // Data callback
        void serial_callback(
            rtac::asio::Stream::Ptr /*stream*/, std::string* data,
            const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count
        );
};