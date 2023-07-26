//Copyright 2023 David Dovrat

//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at

//			http://www.apache.org/licenses/LICENSE-2.0

//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.


#ifndef _ROS_ALATE_ADAPTER_H_
#define _ROS_ALATE_ADAPTER_H_

#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_alate_interfaces/msg/op_com.hpp"

namespace ros_alate {

class Adapter : public rclcpp::Node
{
public:
	/* Constructor:
	 * strNodeName: ROS node name
	 */
	
	Adapter(std::string strNodeName = "ros_alate_adapter");

private:

	/********* Callbacks *********/
	
	void callback_Velocity(const geometry_msgs::msg::Twist &msg) const;
	void callback_OpCom(const ros_alate_interfaces::msg::OpCom &msg) const;
	
	/********* Attributes *********/
	
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_pSubscriptionVelocity;
	rclcpp::Subscription<ros_alate_interfaces::msg::OpCom>::SharedPtr m_pSubscriptionOpCom;

};

}  // namespace ros_alate

#endif // _ROS_ALATE_ADAPTER_H_
