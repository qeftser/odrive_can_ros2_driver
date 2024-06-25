#include "odrive_axis.hpp"
#include <functional>
#include <cstring>
#include <thread>
#include <chrono>

namespace odrive {


    ODriveAxis::ODriveAxis(rclcpp::Node * node, std::string axis_name, int axis_can_id, std::string direction) 
    {
        parent_node = node;
        int can_pub_sub_retries = CAN_PUB_SUB_RETRIES;
        axis_name_ = axis_name;
        axis_can_id_ = axis_can_id;
        axis_status_ = AxisStatus::STARTUP;
        if (direction == "forward") {
            direction_ = 1;
        } else {
            direction_ = -1;
        }
        can_rx_topic_ = parent_node->declare_parameter<std::string>("can_rx_topic", "/can/received_messages");
        can_tx_topic_ = parent_node->declare_parameter<std::string>("can_tx_topic", "/can/sent_messages");
        update_rate_ = parent_node->declare_parameter<double>("update_rate", DEFAULT_UPDATE_RATE);
        engage_on_startup_ = parent_node->declare_parameter<bool>("engage_on_startup", false);
        axis_min_velocity_ = parent_node->declare_parameter<double>("axis_min_velocity", 0);
        received_messages_sub_ = parent_node->create_subscription<can_msgs::msg::Frame>(can_rx_topic_, 10,
            std::bind(&ODriveAxis::canReceivedMessagesCallback, this, std::placeholders::_1));
        sent_messages_pub_ = parent_node->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);
        target_velocity_sub_ = parent_node->create_subscription<std_msgs::msg::Float64>("/" + axis_name + "/target_velocity",
            1, std::bind(&ODriveAxis::velocityReceivedMessagesCallback, this, std::placeholders::_1));
        output_velocity_pub_ = parent_node->create_publisher<std_msgs::msg::Float64>("/" + axis_name + "/output_velocity", 1);
        axis_angle_pub_ = parent_node->create_publisher<std_msgs::msg::Float64>("/" + axis_name + "/angle", 1);
        axis_velocity_pub_ = parent_node->create_publisher<std_msgs::msg::Float64>("/" + axis_name + "/current_velocity", 1);
        axis_voltage_pub_ = parent_node->create_publisher<std_msgs::msg::Float64>("/" + axis_name + "/voltage", 1);
        axis_current_pub_ = parent_node->create_publisher<std_msgs::msg::Float64>("/" + axis_name + "/current", 1);
        /*
            Creating publisher or subscriber in ROS does not guarantee they are immediately connected
            to their peers. When connecting with socketcan_bridge this means a possibility of lost CAN
            messages during startup. To fix this we wait unless messages pub and sub have at least 1
            subscriber and publisher. If they doesn't settle in defined time odrive node will terminate.
            This also guarantees that if socketcan_bridge node fails on startup, odrive node will also fail.
        */
        while (((!parent_node->count_subscribers(can_tx_topic_)) > 0) || ((!parent_node->count_publishers(can_rx_topic_)) > 0)) {
            //rclcpp::Duration(0.1).sleep();
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
            can_pub_sub_retries--;
            if (can_pub_sub_retries == 0) {
                RCLCPP_ERROR(parent_node->get_logger(),"Axis %02x: timeout waiting for CAN pub/sub", axis_can_id_);
                axis_status_ = AxisStatus::ERROR;
                return;
            }
        }
        axis_angle_ = 0.0;
        axis_velocity_ = 0.0;
        axis_current_ = 0.0;
        axis_voltage_ = 0.0;
        axis_update_timer_ = parent_node->create_wall_timer(std::chrono::duration<double,std::milli>(update_rate_),
              std::bind(&ODriveAxis::updateTimerCallback, this));
        // TODO: design the logic behind engage on startup behaviour
        if (engage_on_startup_) {
            RCLCPP_INFO(parent_node->get_logger(), "Axis %02x: engaging", axis_can_id_);
            engage();
        }
        axis_status_ = AxisStatus::OK;
    }

    void ODriveAxis::canReceivedMessagesCallback(const can_msgs::msg::Frame& msg) {
        // TODO: implement CAN message parsing and processing
        uint32_t ID = msg.id;
        uint32_t NODE_ID = (ID >> 5);
        std_msgs::msg::Float64 angle_msg;
        std_msgs::msg::Float64 velocity_msg;
        std_msgs::msg::Float64 current_msg;
        std_msgs::msg::Float64 voltage_msg;
        float velocityEstimate;
        float positionEstimate;
        float axisVoltage;
        float axisCurrent;
        uint8_t *ptrVelocityEstimate = (uint8_t *)&velocityEstimate;
        uint8_t *ptrPositionEstimate = (uint8_t *)&positionEstimate;
        uint8_t *ptrAxisVoltage = (uint8_t *)&axisVoltage;
        uint8_t *ptrAxisCurrent = (uint8_t *)&axisCurrent;
        if (NODE_ID == (uint32_t)axis_can_id_) {
            uint32_t CMD_ID = (ID & 0x01F);
            switch (CMD_ID) {
            case ODriveCommandId::HEARTBEAT_MESSAGE:
                RCLCPP_DEBUG(parent_node->get_logger(), "Axis %02x: HEARTBEAT_MESSAGE", axis_can_id_);
                break;
            case ODriveCommandId::GET_ENCODER_ESTIMATE:
                RCLCPP_DEBUG(parent_node->get_logger(), "Axis %02x: GET_ENCODER_ESTIMATE", axis_can_id_);
                ptrPositionEstimate[0] = msg.data[0];
                ptrPositionEstimate[1] = msg.data[1];
                ptrPositionEstimate[2] = msg.data[2];
                ptrPositionEstimate[3] = msg.data[3];
                ptrVelocityEstimate[0] = msg.data[4];
                ptrVelocityEstimate[1] = msg.data[5];
                ptrVelocityEstimate[2] = msg.data[6];
                ptrVelocityEstimate[3] = msg.data[7];
                axis_angle_ = (double)positionEstimate * 2 * M_PI * direction_;
                axis_velocity_ = (double)velocityEstimate * 2 * M_PI * direction_;
                angle_msg.data = axis_angle_;
                velocity_msg.data = axis_velocity_;
                axis_angle_pub_->publish(angle_msg);
                axis_velocity_pub_->publish(velocity_msg);
                break;
            case ODriveCommandId::GET_BUS_VOLTAGE_AND_CURRENT:
                RCLCPP_DEBUG(parent_node->get_logger(), "Axis %02x: GET_BUS_VOLTAGE_AND_CURRENT", axis_can_id_);
                ptrAxisVoltage[0] = msg.data[0];
                ptrAxisVoltage[1] = msg.data[1];
                ptrAxisVoltage[2] = msg.data[2];
                ptrAxisVoltage[3] = msg.data[3];
                ptrAxisCurrent[0] = msg.data[4];
                ptrAxisCurrent[1] = msg.data[5];
                ptrAxisCurrent[2] = msg.data[6];
                ptrAxisCurrent[3] = msg.data[7];
                axis_current_ = (double)axisCurrent;
                axis_voltage_ = (double)axisVoltage;
                current_msg.data = axis_current_;
                voltage_msg.data = axis_voltage_;
                axis_current_pub_->publish(current_msg);
                axis_voltage_pub_->publish(voltage_msg);
                break;
            default:
                RCLCPP_DEBUG(parent_node->get_logger(), "Axis %02x: unsupported command %02x", axis_can_id_, CMD_ID);
                break;
            }
        }
    }

    void ODriveAxis::velocityReceivedMessagesCallback(const std_msgs::msg::Float64& msg) {
        std_msgs::msg::Float64 velocity_msg;
        double targetVelocity = msg.data;
        // Check that target velocity is not less then axis minimum velocity in both directions
        if (targetVelocity != 0 && axis_min_velocity_ != 0) {
            if (targetVelocity > 0) {
                if (targetVelocity < axis_min_velocity_) {
                    targetVelocity = axis_min_velocity_;
                }
            } else {
                if (targetVelocity > (axis_min_velocity_ * -1)) {
                    targetVelocity = axis_min_velocity_ * -1;
                }
            }
        }
        // Publish velocity to output_velocity
        velocity_msg.data = (double)targetVelocity;
        output_velocity_pub_->publish(velocity_msg);
        double odriveTargetVelocity = targetVelocity / (2.0 * M_PI) * direction_;
        setInputVelocity(odriveTargetVelocity);
    }

    void ODriveAxis::updateTimerCallback(void) {
        requestEncoderEstimate();
        requestBusVoltageAndCurrent();
    }

    void ODriveAxis::requestEncoderEstimate() {
        can_msgs::msg::Frame request_msg;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::GET_ENCODER_ESTIMATE);
        request_msg.is_extended = false;
        request_msg.is_rtr = true;
        request_msg.dlc = 8;
        sent_messages_pub_->publish(request_msg);
    }

    void ODriveAxis::requestBusVoltageAndCurrent() {
        can_msgs::msg::Frame request_msg;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::GET_BUS_VOLTAGE_AND_CURRENT);
        request_msg.is_extended = false;
        request_msg.is_rtr = true;
        request_msg.dlc = 8;
        sent_messages_pub_->publish(request_msg);
    }

    void ODriveAxis::setAxisRequestedState(ODriveAxisState state) {
        can_msgs::msg::Frame request_msg;
        uint32_t requestedState = (uint32_t)state;
        uint8_t *ptrRequestedState;
        ptrRequestedState = (uint8_t *)&requestedState;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_AXIS_REQUESTED_STATE);
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrRequestedState[0];
        request_msg.data[1] = ptrRequestedState[1];
        request_msg.data[2] = ptrRequestedState[2];
        request_msg.data[3] = ptrRequestedState[3];
        sent_messages_pub_->publish(request_msg);
    }

    void ODriveAxis::setAxisControlMode(ODriveControlMode control_mode, ODriveInputMode input_mode) {
        can_msgs::msg::Frame request_msg;
        int32_t requestedMode[2];
        uint8_t *ptrRequestedMode;
        requestedMode[0] = (int32_t)control_mode;
        requestedMode[1] = (int32_t)input_mode;
        ptrRequestedMode = (uint8_t *)&requestedMode[0];
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_CONTROLLER_MODES);
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrRequestedMode[0];
        request_msg.data[1] = ptrRequestedMode[1];
        request_msg.data[2] = ptrRequestedMode[2];
        request_msg.data[3] = ptrRequestedMode[3];
        request_msg.data[4] = ptrRequestedMode[4];
        request_msg.data[5] = ptrRequestedMode[5];
        request_msg.data[6] = ptrRequestedMode[6];
        request_msg.data[7] = ptrRequestedMode[7];
        sent_messages_pub_->publish(request_msg);
    }

    void ODriveAxis::setInputVelocity(double velocity) {
        can_msgs::msg::Frame request_msg;
        float vel = (float)velocity;
        float torq = 0.0;
        uint8_t *ptrVel;
        uint8_t *ptrTor;
        ptrVel = (uint8_t *)&vel;
        ptrTor = (uint8_t *)&torq;
        request_msg.id = createCanId(axis_can_id_, ODriveCommandId::SET_INPUT_VELOCITY);
        request_msg.is_extended = false;
        request_msg.dlc = 8;
        request_msg.data[0] = ptrVel[0];
        request_msg.data[1] = ptrVel[1];
        request_msg.data[2] = ptrVel[2];
        request_msg.data[3] = ptrVel[3];
        request_msg.data[4] = ptrTor[0];
        request_msg.data[5] = ptrTor[1];
        request_msg.data[6] = ptrTor[2];
        request_msg.data[7] = ptrTor[3];
        sent_messages_pub_->publish(request_msg);
    }

    void ODriveAxis::engage() {
        setAxisRequestedState(ODriveAxisState::CLOSED_LOOP_CONTROL);
        setAxisControlMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    }

    void ODriveAxis::disengage() {
        // setAxisRequestedState(ODriveAxisState::IDLE);
    }

    AxisStatus ODriveAxis::getAxisStatus() {
        return axis_status_;
    }

    uint32_t ODriveAxis::createCanId(int axis_can_id, int command) {
        uint32_t can_id;
        can_id = (axis_can_id << 5) | command;
        return can_id;
    }

}
