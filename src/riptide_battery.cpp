#include "riptide_battery/riptide_battery.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nlohmann/json.hpp>


void RiptideBattery::serial_callback(
        rtac::asio::Stream::Ptr /*stream*/, std::string* data,
        const rtac::asio::SerialStream::ErrorCode& /*err*/, std::size_t count
    ) {
        if (count > 0) {
            // Parsing data
            try {
                // Parsing json data
                nlohmann::json json_data = nlohmann::json::parse((*data).substr(0, count));
                RCLCPP_DEBUG(this->get_logger(), "Count: %ld, Received: %s", count, (*data).substr(0, count).c_str());

                // Filling the message
                battery_msg_.voltage = json_data["volt"];
                battery_msg_.current = json_data["current"];

                // Publishing the message
                battery_card_publisher_->publish(battery_msg_);
            }
            catch (...) {
                RCLCPP_WARN(this->get_logger(), "Error while parsing json data. This error could occur sometimes but should not be recurring");
            }
        }

        // Launching another read_until
        buffer_ = std::string(1024, '\0');
        if(!serial_->async_read_until(buffer_.size(), (uint8_t*)buffer_.c_str(), '}',
            std::bind(&RiptideBattery::serial_callback, this, serial_, &buffer_, std::placeholders::_1, std::placeholders::_2))
        ) {
            RCLCPP_FATAL(this->get_logger(),
                "Unable to launch async read until"
            );
        }
    }

RiptideBattery::RiptideBattery() : Node("riptide_battery") {
    // Get port and baudrate from parameters
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", "115200");

    // Creating the message
    battery_msg_.header.frame_id = "base_link";
    battery_msg_.design_capacity = 12;
    battery_msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    battery_msg_.present = true;

    // Creating the publisher
    battery_card_publisher_ = this->create_publisher<Msg>("/riptide_battery/battery_status", 10);

    // Instanciate the driver
    try {
        serial_ = rtac::asio::Stream::CreateSerial(this->get_parameter("port").as_string(), this->get_parameter("baudrate").as_int());
        serial_->start();
        serial_->flush();
    }
    catch(boost::system::system_error& e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Serial error: '%s'", e.what()
        );
    }

    // Launching an asynchronous read on the serial until '}' is on the string
    buffer_ = std::string(1024, '\0');
    if(!serial_->async_read_until(buffer_.size(), (uint8_t*)buffer_.c_str(), '}',
        std::bind(&RiptideBattery::serial_callback, this, serial_, &buffer_, std::placeholders::_1, std::placeholders::_2))
    ) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Unable to launch async read until"
        );
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RiptideBattery>());
    rclcpp::shutdown();
    return 0;
}
