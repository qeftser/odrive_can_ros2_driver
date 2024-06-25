#ifndef ODRIVE_NODE_HPP_
#define ODRIVE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "can_msgs/msg/frame.hpp"
#include "odrive_axis.hpp"
#include <unordered_set>
#include <string>
#include <vector>
#include <functional>

#define DEFAULT_CAN_INTERFACE       "can10"
#define DEFAULT_CAN_BITRATE         250000
#define DEFAULT_ENGAGE_ON_STARTUP   false
#define DEFAULT_DISENGAGE_ON_SHUTDOWN   true

class ODriveNode : public rclcpp::Node {
public:
   ODriveNode();
private:
   bool engage_on_startup;
   bool disengage_on_shutdown;
   std::vector<std::string> axis_names_list;
   std::vector<long> axis_can_ids_list;
   std::vector<std::string> axis_directions_list;
   std::vector<odrive::ODriveAxis *> odrive_axises;

   bool intsAreDistinct(std::vector<long> arr);
   bool stringsAreDistinct(std::vector<std::string> arr); 
   void canReceivedMessagesCallback(const can_msgs::msg::Frame& msg);
   void onShutdown(int sig);
   
   int onStartup(void);

};



#endif // ODRIVE_NODE_HPP_
