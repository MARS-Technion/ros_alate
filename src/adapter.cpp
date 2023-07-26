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

#include "ros_alate/adapter.h"

namespace ros_alate {

Adapter::Adapter(std::string strNodeName) : Node(strNodeName)
{
	m_pSubscriptionVelocity = this->create_subscription<geometry_msgs::msg::Twist>(
		"alate_input_velocity", 10, std::bind(&Adapter::callback_Velocity, this, std::placeholders::_1)
	);
	m_pSubscriptionOpCom = this->create_subscription<ros_alate_interfaces::msg::OpCom>(
		"alate_input_operator_command", 10, std::bind(&Adapter::callback_OpCom, this, std::placeholders::_1)
	);
}
	

void Adapter::callback_Velocity(const geometry_msgs::msg::Twist &msg) const
{
	RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.angular.z);
}

void Adapter::callback_OpCom(const ros_alate_interfaces::msg::OpCom &msg) const
{
	RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.op_com_enum);
}

} // namespace ros_alate
