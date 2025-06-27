//  COPYRIGHT (C) 2025 Mitsubishi Electric Corporation

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  Contributor(s):
//     Liu Muyao

#include "melfa_iq_msgs/msg/button_state.hpp"
#include "melfa_iq_msgs/msg/gripper_state.hpp"
#include "melfa_msgs/msg/gpio_command.hpp"
#include "melfa_msgs/msg/gpio_state.hpp"
#include "melfa_msgs/srv/gpio_configure.hpp"
#include "rclcpp/rclcpp.hpp"
#include <bitset>
#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

// I/O addresses
uint16_t button_add = 11008; // io_unit. PLC U3En\HG64
uint16_t gripper_add = 900;  // hand_io

/**
 * @brief Standard ROS2 service call function for initial configuration. Set
 * head bit address and configure I/O controller for read or write operation for
 * input or ouput.
 *
 * @param node_ shared pointer for ROS2 node.
 * @param mode gpio_controller command interface mode. "WRITE_OUT" = Write to
 * output. "READ_OUT" = Read from outout. "READ_IN" = Read from input.
 * @param address robot controller I/O memory address
 * @param mask bit masking value
 * @param data value for gpio_controller command interface to write to robot
 * controller. Only applicable when mode = "WRITE_OUT".
 */
void configure_io_(rclcpp::Node::SharedPtr node_, std::string mode,
                   uint16_t address, uint16_t mask, uint16_t data) {
  rclcpp::Client<melfa_msgs::srv::GpioConfigure>::SharedPtr client =
      node_->create_client<melfa_msgs::srv::GpioConfigure>(
          "/gpio_controller/configure_gpio");
  auto io_write_request =
      std::make_shared<melfa_msgs::srv::GpioConfigure::Request>();
  io_write_request->mode = mode;
  io_write_request->bitid = address;
  io_write_request->bitmask = mask;
  if (!mode.compare("WRITE_OUT")) {
    io_write_request->bitdata = data;
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Writing...");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Reading...");
  }
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("configure_io_"),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("configure_io_"),
                "service not available, waiting again...");
  }

  auto io_write_result = client->async_send_request(io_write_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_, io_write_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto service_response = io_write_result.get();

    RCLCPP_INFO(rclcpp::get_logger("configure_io_"), "Service Success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("configure_io_"), "Failed to call service");
  }
}
/*
IQNode Class
*/
class IQNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new IQNode object
   *
   */
  IQNode() : Node("iq_") {
    auto iq_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options.callback_group = iq_callback_group;

    // Publishers that write to I/O controllers
    gripper_command_publisher_ =
        this->create_publisher<melfa_msgs::msg::GpioCommand>(
            "gpio_controller/gpio_command", 10);
    reset_io_publisher_ = this->create_publisher<melfa_msgs::msg::GpioCommand>(
        "gpio_controller/gpio_command", 10);
    button_publisher_ = this->create_publisher<melfa_msgs::msg::GpioCommand>(
        "gpio_controller/gpio_command", 10);

    // Publishers that write to ROS2 topics
    gripper_state_publisher_ =
        this->create_publisher<melfa_iq_msgs::msg::GripperState>(
            "robot_/gripper_state", 10);
    button_state_publisher_ =
        this->create_publisher<melfa_iq_msgs::msg::ButtonState>(
            "hmi_/button_state", 10);

    // Subscribers that monitor I/O controllers
    gripper_state_subscription_ =
        this->create_subscription<melfa_msgs::msg::GpioState>(
            "/gpio_controller/hand_io_state", rclcpp::SensorDataQoS(),
            std::bind(&IQNode::gripper_state_callback, this, _1), options);
    button_subscription_ =
        this->create_subscription<melfa_msgs::msg::GpioState>(
            "/gpio_controller/plc_link_io_state", rclcpp::SensorDataQoS(),
            std::bind(&IQNode::button_state_callback, this, _1), options);

    // Subscribers that monitor ROS2 topics
    gripper_command_subscription_ =
        this->create_subscription<melfa_iq_msgs::msg::GripperState>(
            "/robot_/gripper_command", rclcpp::SensorDataQoS(),
            std::bind(&IQNode::gripper_command_callback, this, _1), options);
    button_command_subscription_ =
        this->create_subscription<melfa_iq_msgs::msg::ButtonState>(
            "/hmi_/button_command", rclcpp::SensorDataQoS(),
            std::bind(&IQNode::button_command_callback, this, _1), options);
  }

  /**
   * @brief Monitors the state of gripper using inbuilt gripper feedback
   * INPUT:900. Do note that not all gripper OEMs utilize this function. This
   * feature is commonly used by EEF tool designers for close loop control of
   * custom grippers. In this sample program, INPUT:900-931 is mirrored from
   * OUTPUT:900-931 in RT Toolbox3.
   *
   * @param msg
   */
  void gripper_state_callback(const melfa_msgs::msg::GpioState &msg) {
    uint16_t gripper_input = msg.input_data;
    auto gripper_state_ = melfa_iq_msgs::msg::GripperState();
    gripper_state_.double_solenoid = double_solenoid;

    if (reset_io_setting(gripper_add, msg.bitid, msg.bit_send_type) != 0) {
      // if reset_io_setting detects the wrong address, ignore current data.
      return void();
    }

    if (double_solenoid) {
      if ((gripper_input & 0b10) == 0b10) {
        gripper_state_.hand_1 = true;
      }
      if ((gripper_input & 0b1000) == 0b1000) {
        gripper_state_.hand_2 = true;
      }
      if ((gripper_input & 0b100000) == 0b100000) {
        gripper_state_.hand_3 = true;
      }
      if ((gripper_input & 0b10000000) == 0b10000000) {
        gripper_state_.hand_4 = true;
      }
      gripper_state_.hand_5 = false;
      gripper_state_.hand_6 = false;
      gripper_state_.hand_7 = false;
      gripper_state_.hand_8 = false;
    } else {
      if (gripper_input & 0b1) {
        gripper_state_.hand_1 = true;
      }
      if (gripper_input & 0b10) {
        gripper_state_.hand_2 = true;
      }
      if (gripper_input & 0b100) {
        gripper_state_.hand_3 = true;
      }
      if (gripper_input & 0b1000) {
        gripper_state_.hand_4 = true;
      }
      if (gripper_input & 0b10000) {
        gripper_state_.hand_5 = true;
      }
      if (gripper_input & 0b100000) {
        gripper_state_.hand_6 = true;
      }
      if (gripper_input & 0b1000000) {
        gripper_state_.hand_7 = true;
      }
      if (gripper_input & 0b10000000) {
        gripper_state_.hand_8 = true;
      }
    }
    gripper_state_publisher_->publish(gripper_state_);
  }

  /**
   * @brief Monitors gripper commands from other nodes and convert generic
   * gripper commands to ROS2 message for I/O controllers.
   *
   * @param msg
   */
  void gripper_command_callback(const melfa_iq_msgs::msg::GripperState &msg) {
    auto command_ = msg;
    std::string gripper_command_str_;
    uint16_t gripper_command_int_ = 0b0;
    double_solenoid = command_.double_solenoid;
    if (!double_solenoid) {
      gripper_command_str_ =
          std::to_string(command_.hand_8) + std::to_string(command_.hand_7) +
          std::to_string(command_.hand_6) + std::to_string(command_.hand_5) +
          std::to_string(command_.hand_4) + std::to_string(command_.hand_3) +
          std::to_string(command_.hand_2) + std::to_string(command_.hand_1);

      gripper_command_int_ = std::bitset<8>(gripper_command_str_).to_ulong();
    } else if (double_solenoid) {
      if (command_.hand_1) {
        gripper_command_int_ = gripper_command_int_ | 0b10;
      } else {
        gripper_command_int_ = gripper_command_int_ | 0b01;
      }
      if (command_.hand_2) {
        gripper_command_int_ = gripper_command_int_ | 0b1000;
      } else {
        gripper_command_int_ = gripper_command_int_ | 0b0100;
      }
      if (command_.hand_3) {
        gripper_command_int_ = gripper_command_int_ | 0b100000;
      } else {
        gripper_command_int_ = gripper_command_int_ | 0b010000;
      }
      if (command_.hand_4) {
        gripper_command_int_ = gripper_command_int_ | 0b10000000;
      } else {
        gripper_command_int_ = gripper_command_int_ | 0b01000000;
      }
    }
    auto message = melfa_msgs::msg::GpioCommand();
    message.bitid = 900;
    message.bitmask = 0xFFFF;
    message.bit_recv_type = "MXT_IO_IN";
    message.bit_send_type = "MXT_IO_OUT";
    message.bitdata = gripper_command_int_;

    gripper_command_publisher_->publish(message);
  }

  /**
   * @brief Monitors first set of PLC link I/Os. These I/O address are mapped by
   * default and utilizes the MELSEC IQ platform for high speed memory transfer
   * at up to 0.222ms. Converts the input data into generic program interlock
   * message for other ROS2 nodes.
   *
   * @param msg
   */
  void button_state_callback(const melfa_msgs::msg::GpioState &msg) {
    uint16_t button_input = msg.input_data;
    auto message = melfa_iq_msgs::msg::ButtonState();

    if (reset_io_setting(button_add, msg.bitid, msg.bit_send_type) != 0) {
      // if reset_io_setting detects the wrong address, ignore current data.
      return void();
    }

    if (button_input & 0x01) {
      message.button_0 = true;
    }
    if (button_input & 0x02) {
      message.button_1 = true;
    }
    if (button_input & 0x04) {
      message.button_2 = true;
    }
    if (button_input & 0x08) {
      message.button_3 = true;
    }
    if (button_input & 0x10) {
      message.button_4 = true;
    }
    if (button_input & 0x20) {
      message.button_5 = true;
    }
    if (button_input & 0x40) {
      message.button_6 = true;
    }
    if (button_input & 0x80) {
      message.button_7 = true;
    }
    if (button_input & 0x100) {
      message.button_8 = true;
    }
    if (button_input & 0x200) {
      message.button_9 = true;
    }
    if (button_input & 0x400) {
      message.button_10 = true;
    }
    if (button_input & 0x800) {
      message.button_11 = true;
    }
    if (button_input & 0x1000) {
      message.button_12 = true;
    }
    if (button_input & 0x2000) {
      message.button_13 = true;
    }
    if (button_input & 0x4000) {
      message.button_14 = true;
    }
    if (button_input & 0x8000) {
      message.button_15 = true;
    }
    button_state_publisher_->publish(message);
  }

  /**
   * @brief Publishers program interlock parameters from other ROS2 nodes to I/O
   * controller
   *
   * @param msg
   */
  void button_command_callback(const melfa_iq_msgs::msg::ButtonState &msg) {
    auto output_button = melfa_msgs::msg::GpioCommand();

    std::string output_button_str =
        std::to_string(msg.button_15) + std::to_string(msg.button_14) +
        std::to_string(msg.button_13) + std::to_string(msg.button_12) +
        std::to_string(msg.button_11) + std::to_string(msg.button_10) +
        std::to_string(msg.button_9) + std::to_string(msg.button_8) +
        std::to_string(msg.button_7) + std::to_string(msg.button_6) +
        std::to_string(msg.button_5) + std::to_string(msg.button_4) +
        std::to_string(msg.button_3) + std::to_string(msg.button_2) +
        std::to_string(msg.button_1) + std::to_string(msg.button_0);
    output_button.bitdata =
        (uint16_t)std::bitset<16>(output_button_str).to_ulong();

    output_button.bitid = button_add;
    output_button.bitmask = 0xFFFF;
    output_button.bit_recv_type = "MXT_IO_IN";
    output_button.bit_send_type = "MXT_IO_OUT";

    button_publisher_->publish(output_button);
  }

  /**
   * @brief Checks if monitored I/O address is the correct address. I/O address
   * monitor by I/O controller can be modified by other programs using a service
   * call or publisher. This function ensures that the monitored address is the
   * intended address.
   *
   * @param intended_add
   * @param actual_add
   * @param feedback_send_type
   * @return int
   */
  int reset_io_setting(uint16_t intended_add, uint16_t actual_add,
                       std::string feedback_send_type) {
    if (actual_add != intended_add || feedback_send_type.compare("MXT_IO_IN")) {
      auto reset_message = melfa_msgs::msg::GpioCommand();
      reset_message.bitid = intended_add;
      reset_message.bitmask = 0xFFFF;
      reset_message.bit_recv_type = "MXT_IO_IN";
      reset_message.bit_send_type = "MXT_IO_NULL";
      reset_message.bitdata = 0;
      reset_io_publisher_->publish(reset_message);
      return -1;
    } else
      return 0;
  }

private:
  // Subscribers that monitor I/O controllers

  /**
   * @brief Subscribes to MELFA memory address: gripper_add, which reflects the
   * state of robot gripper.
   *
   */
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr
      gripper_state_subscription_;

  /**
   * @brief Subscribes to MELFA memory address: button_add, which reflects the
   * state of the buttons on GOT-HMI.
   *
   */
  rclcpp::Subscription<melfa_msgs::msg::GpioState>::SharedPtr
      button_subscription_;

  // Subscribers that monitor ROS2 topics

  /**
   * @brief Subscribes to ROS2 topic: "/robot_/gripper_command", which is
   * published by other programs, including CLI.
   *
   */
  rclcpp::Subscription<melfa_iq_msgs::msg::GripperState>::SharedPtr
      gripper_command_subscription_;

  /**
   * @brief Subscribes to ROS2 topic: "/hmi_/button_command", which is
   * published by other programs, including CLI.
   *
   */
  rclcpp::Subscription<melfa_iq_msgs::msg::ButtonState>::SharedPtr
      button_command_subscription_;

  // Publishers that write to I/O controllers

  /**
   * @brief Publish the gripper command to gpio_controller.
   *
   */
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr
      gripper_command_publisher_;

  /**
   * @brief Publish to gpio_controller to re-configure I/O interface.
   *
   */
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr
      reset_io_publisher_;

  /**
   * @brief Publish lamp commands to gpio_controller to operate GOT-HMI lamps.
   *
   */
  rclcpp::Publisher<melfa_msgs::msg::GpioCommand>::SharedPtr button_publisher_;

  // Publishers that write to ROS2 topics

  /**
   * @brief Publish gripper state to ROS2.
   *
   */
  rclcpp::Publisher<melfa_iq_msgs::msg::GripperState>::SharedPtr
      gripper_state_publisher_;

  /**
   * @brief Publish GOT-HMI button state to ROS2.
   *
   */
  rclcpp::Publisher<melfa_iq_msgs::msg::ButtonState>::SharedPtr
      button_state_publisher_;

  /**
   * @brief subscription options for callback group.
   *
   */
  rclcpp::SubscriptionOptions options;

  /**
   * @brief Default solenoid configuration for MELFA is double solenoid.
   *
   */
  bool double_solenoid = true;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<IQNode>();

  // configure plc_link_io controller to monitor IQ platform shared memory.
  // This set of memory is used for interacting with the GOT HMI.
  RCLCPP_INFO(rclcpp::get_logger("iq_"), "Configuring plc_link_io to %d",
              button_add);
  configure_io_(node_, "READ_IN", button_add, 0xffff, 0);

  // configure gripper_io controller to monitor gripper state.
  // Change "READ_IN" to "READ_OUT" if you are using this with real hardware and
  // your EEF tool does not use inbuilt gripper feedback INPUT:900~907.
  RCLCPP_INFO(rclcpp::get_logger("iq_"), "Configuring hand_io to %d",
              gripper_add);
  configure_io_(node_, "READ_IN", gripper_add, 0xffff, 0);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
  rclcpp::shutdown();
}