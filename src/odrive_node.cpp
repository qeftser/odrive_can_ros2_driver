#include "odrive_node.hpp"
#include "odrive_axis.hpp"
#include <signal.h>
#include <fstream>
#include <iostream>



/*
bool engage_on_startup;
bool disengage_on_shutdown;
std::vector<std::string> axis_names_list;
std::vector<int> axis_can_ids_list;
std::vector<std::string> axis_directions_list;
std::vector<odrive::ODriveAxis *> odrive_axises;
// rclcpp::Subscription can_bridge_received_messages_sub;
// */

bool ODriveNode::intsAreDistinct(std::vector<long> arr) {
    std::unordered_set<long> s;
    for (int i = 0; i < (long)arr.size(); i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

bool ODriveNode::stringsAreDistinct(std::vector<std::string> arr) {
    int n = arr.size();
    std::unordered_set<std::string> s;
    for (int i = 0; i < n; i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

void ODriveNode::canReceivedMessagesCallback(const can_msgs::msg::Frame& msg) {
    RCLCPP_INFO(this->get_logger(), "I heard from: [%02x]", msg.id);
}

void ODriveNode::onShutdown(int sig) {
    (void)sig;
    RCLCPP_ERROR(this->get_logger(), "ODrive shutting down");
	if (disengage_on_shutdown) {
      RCLCPP_ERROR(this->get_logger(), "ODrive disengaging motors");
        for (int i = 0; i < (int)odrive_axises.size(); i++) {
            odrive_axises[i]->disengage();
        }
	}
	rclcpp::shutdown();
}

ODriveNode::ODriveNode(void) : Node("odrive_node") {
   if (-1 == onStartup())
      RCLCPP_INFO(this->get_logger(), "Failure!");
}

int ODriveNode::onStartup(void) {
    RCLCPP_INFO(this->get_logger(),"ODrive starting up");

    engage_on_startup = this->declare_parameter<bool>("engage_on_startup", DEFAULT_ENGAGE_ON_STARTUP);
    disengage_on_shutdown = this->declare_parameter<bool>("disengage_on_shutdown", DEFAULT_DISENGAGE_ON_SHUTDOWN);

    if (engage_on_startup) {
        RCLCPP_INFO(this->get_logger(),"Will engage axises on startup");
    } else {
        RCLCPP_INFO(this->get_logger(),"Will not engage axises on startup");
    }
    if (!this->has_parameter("axis_names")) {
        RCLCPP_ERROR(this->get_logger(),"Can't run without axis_names parameter");
        return -1;
    }
    if (!this->has_parameter("axis_can_ids")) {
        RCLCPP_ERROR(this->get_logger(),"Can't run without axis_can_ids parameter");
        return -1;
    }
    if (!this->has_parameter("axis_directions")) {
        RCLCPP_ERROR(this->get_logger(),"Can't run without axis_directions parameter");
        return -1;
    }
    this->get_parameter("axis_names", axis_names_list);
    this->get_parameter("axis_can_ids", axis_can_ids_list);
    this->get_parameter("axis_directions", axis_directions_list);

    if (!(axis_names_list.size() == axis_can_ids_list.size() && axis_can_ids_list.size() ==
        axis_directions_list.size())) {
        RCLCPP_ERROR(this->get_logger(),"axis_names, axis_can_ids and axis_can_directions must be of an equal size");
        return -1;
    }
    if (!stringsAreDistinct(axis_names_list)) {
        RCLCPP_ERROR(this->get_logger(),"axis names must be distinct");
        return -1;
    }
    if (!intsAreDistinct(axis_can_ids_list)) {
        RCLCPP_ERROR(this->get_logger(),"axis CAN ids must be distinct");
        return -1;
    }
    for (int i = 0; i < (int)axis_names_list.size(); i++) {
        RCLCPP_INFO(this->get_logger(),"Adding axis %s with CAN id %d and direction %s", axis_names_list[i].c_str(), 
            axis_can_ids_list[i], axis_directions_list[i].c_str());
        if (axis_names_list[i].length() == 0) {
            RCLCPP_ERROR(this->get_logger(),"axis name can't be empty");
            return -1;
        }
        if (axis_can_ids_list[i] <= 0) {
            RCLCPP_ERROR(this->get_logger(),"axis CAN id must be >0");
            return -1;
        }
        odrive::ODriveAxis *new_axis = new odrive::ODriveAxis(this, axis_names_list[i], axis_can_ids_list[i], 
            axis_directions_list[i]);
        if (new_axis->getAxisStatus() != odrive::AxisStatus::OK) {
            RCLCPP_ERROR(this->get_logger(),"Error starting axis, terminating");
            rclcpp::shutdown();
            return -1;
        }
        odrive_axises.push_back(new_axis);
    }
    return 0;
}

int main(int argc, char** argv) {
   rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ODriveNode>());
   rclcpp::shutdown();
	return 0;
}
