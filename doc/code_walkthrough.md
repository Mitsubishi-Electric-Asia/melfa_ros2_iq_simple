<img src="./figures/MELFA_t.png" width="400" height="98">

# MELFA ROS2 iQ Platform
_**Technology Convergence of Open Source and Factory Automation**_


           ┌────────────┐        ROS2 CLI         ┌────────────┐
           │  IQNode    │ <---------------------> │  User CLI  │
           └────────────┘                         └────────────┘
                 │                                        ▲
                 ▼                                        │
      [PLC Memory / Hand I/O] <--> [GpioController] <--> ROS2 Topics


## Code Explanation

The section explains the code in [IQNode](../melfa_iq_node/src/iq_.cpp).

### 1.1 configure_io_() function

The configure_io_() function is used to initialize the gpio_controllers. Observe the section of the code in [IQNode](../melfa_iq_node/src/iq_.cpp).

```cpp
void configure_io_(rclcpp::Node::SharedPtr node_, std::string mode, uint16_t address, uint16_t mask, uint16_t data)
{
.
.
.
}
```

The configure_io_() function configures the gpio_controllers command interfaces. The function requires the various parameters:
1. `rclcpp::Node::SharedPtr node_`
    -  shared pointer for ROS2 node. ROS2 service client call require a ROS2 node to work. This parameter allows this function to use an existing ROS2 node.
2. `std::string mode`
    - gpio_controller command interface mode. "WRITE_OUT" = Write to output. "READ_OUT" = Read from output. "READ_IN" = Read from input.
3. `uint16_t address`
    - robot controller I/O memory address
4. `uint16_t mask`
    - bit masking value
5. `uint16_t data`
    - value for gpio_controller command interface to write to robot controller. Only applicable when mode = "WRITE_OUT".

### 1.2 Class Constructor

The code below configures the various publishers and subscribers used in this exercise. Observe the section of the code in [IQNode](../melfa_iq_node/src/iq_.cpp).
```cpp
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
.
.
.
  }
}
```

In this exercise, we will be writing and reading various gpio_controller interfaces at the same time. Thus, we require a "Reentrant" callback group with a multithreaded executor. For more information regarding ROS2 executors and callback groups, refer to [Executors — ROS 2 Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html#).

```cpp
auto iq_callback_group =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);
options.callback_group = iq_callback_group;
```

We will use the code below as an example to show how to configure publishers.

```cpp
// Publishers that write to I/O controllers
gripper_command_publisher_ =
    this->create_publisher<melfa_msgs::msg::GpioCommand>(
        "gpio_controller/gpio_command", 10);
reset_io_publisher_ = this->create_publisher<melfa_msgs::msg::GpioCommand>(
    "gpio_controller/gpio_command", 10);
button_publisher_ = this->create_publisher<melfa_msgs::msg::GpioCommand>(
    "gpio_controller/gpio_command", 10);
```

These publishers are initialized with the "melfa_msgs::msg::GpioCommand" message type, the topic name "gpio_controller/gpio_command", and the required queue size to limit messages in the event of a backup. These topics are used to configure the gpio_controller command interfaces.

1. The gripper_command_publisher_ configures the hand_io interface to toggle gripper solenoids and read gripper state feedback.

2. The reset_io_publisher_ re-configures the gpio_controller command interface when it detects a discrepancy between the expected configuration and the actual configuration. This can be caused by relaunching "robot"_control.launch.py which configures the command interfaces to its default setting.

3. The button_publisher_ configures the plc_link_io interface to read & write memory operated by GOT-HMI. The data is transferred between the robot and GOT-HMI via MELSEC using the iQ Platform.


#### List of publisher and subscriber in the `IQ_Node`

|Role|Topic|Msg Type|Direction|Purpose|
|---|---|---|---|---|
|`gripper_command_pub`|`gpio_controller/gpio_command`|`GpioCommand`|Publish|Control gripper|
|`reset_io_pub`|`gpio_controller/gpio_command`|`GpioCommand`|Publish|Reset GPIOs if reconfigured|
|`button_pub`|`gpio_controller/gpio_command`|`GpioCommand`|Publish|Simulate HMI buttons|
|`gripper_state_pub`|`robot_/gripper_state`|`GripperState`|Publish|Share gripper status|
|`button_state_pub`|`hmi_/button_state`|`ButtonState`|Publish|Share button/lamp status|
|`gripper_state_sub`|`/gpio_controller/hand_io_state`|`GpioState`|Subscribe|Read gripper sensor state|
|`button_state_sub`|`/gpio_controller/plc_link_io_state`|`GpioState`|Subscribe|Read PLC HMI memory state|
|`gripper_command_sub`|`/robot_/gripper_command`|`GripperState`|Subscribe|Accept external gripper commands|
|`button_command_sub`|`/hmi_/button_command`|`ButtonState`|Subscribe|Accept external HMI commands|

### 1.3 Callback Functions

These are the callback functions. Observe the section of the code in [IQNode](../melfa_iq_node/src/iq_.cpp).

```cpp
void gripper_state_callback(const melfa_msgs::msg::GpioState &msg) {
.
.
}
void gripper_command_callback(const melfa_iq_msgs::msg::GripperState &msg) {
.
.
}
void button_state_callback(const melfa_msgs::msg::GpioState &msg) {
.
.
}
void button_command_callback(const melfa_iq_msgs::msg::ButtonState &msg) {
.
.
}
int reset_io_setting(uint16_t intended_add, uint16_t actual_add, std::string feedback_send_type) {
.
.
}
```

The callback functions all follows a similar pattern:
1. Read the state.
2. Translate it in a message that ROS2 can understand.
3. Publish it to the designated ROS2 topic.

For convenience, we will only look at one callback function, `gripper_state_callback`.
The code snippet below is the callback function that read `hand_io` state interface, translates it into gripper states and publishes it to ROS2.
```cpp
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
```
### 1.4 Main Program

The code below creates the ROS2 node, configures the initial configuration for the gpio controllers and spins it. Observe the section of the code in [IQNode](../melfa_iq_node/src/iq_.cpp).

```cpp
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
```
