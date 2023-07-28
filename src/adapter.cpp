/*
Copyright 2023 David Dovrat

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "ros_alate/adapter.h"


namespace ros_alate {

Adapter::Adapter(std::string strNodeName) : Node(strNodeName), 	m_bInterrupted(false), m_pThreadDispatcher(NULL)
{
	/***	ROS Params	***/

	auto param_descriptor = rcl_interfaces::msg::ParameterDescriptor{};

	param_descriptor.description = "The NeMALA node ID, how the dispatcher calls itself.";
	this->declare_parameter("nemala_node_id", 402, param_descriptor);
	param_descriptor.description = "The NeMALA topic ID for the operator command, how the dispatcher knows which handler to call.";
	this->declare_parameter("alate_operator_command_topic", 101, param_descriptor);
	param_descriptor.description = "The NeMALA topic ID for the velocity command, how the dispatcher knows which handler to call.";
	this->declare_parameter("alate_velocity_topic", 104, param_descriptor);
	param_descriptor.description = "The NeMALA proxy endpoint for subscriptions. This is the port the dispatcher listens to.";
	this->declare_parameter("proxy_endpoint_for_subscribing", "ipc:///tmp/alate_subscribers", param_descriptor);
	param_descriptor.description = "The NeMALA proxy endpoint for publishing. This is the port the publishers write to.";
	this->declare_parameter("proxy_endpoint_for_publishing", "ipc:///tmp/alate_publishers", param_descriptor);
	
	rclcpp::Parameter param_nemala_node_id = this->get_parameter("nemala_node_id");
	rclcpp::Parameter param_alate_operator_command_topic = this->get_parameter("alate_operator_command_topic");
	rclcpp::Parameter param_alate_velocity_topic = this->get_parameter("alate_velocity_topic");
	rclcpp::Parameter param_proxy_endpoint_for_subscribing = this->get_parameter("proxy_endpoint_for_subscribing");
	rclcpp::Parameter param_proxy_endpoint_for_publishing = this->get_parameter("proxy_endpoint_for_publishing");
	
	/***	NeMALA Handlers	***/
	
	
	
	/***	NeMALA Publishers	***/
	
	m_pPublisherOpCom = new NeMALA::Publisher(
												param_proxy_endpoint_for_publishing.as_string().c_str(),
												(unsigned int) param_alate_operator_command_topic.as_int()
											 );
	m_pPublisherVelocity = new NeMALA::Publisher(
													param_proxy_endpoint_for_publishing.as_string().c_str(),
													(unsigned int) param_alate_velocity_topic.as_int()
												);
	
	/***	NeMALA Dispatcher	***/
	
	m_pDispatcher = std::make_shared<NeMALA::Dispatcher>(
															1,
															0,
															param_nemala_node_id.as_int(),
															param_proxy_endpoint_for_subscribing.as_string()
														 );
	
	m_pDispatcher->AddPublisher(m_pPublisherOpCom);
	m_pDispatcher->AddPublisher(m_pPublisherVelocity);
	
	/***	ROS Subscriptions	***/

	m_pSubscriptionVelocity = this->create_subscription<geometry_msgs::msg::Twist>(
		"alate_input_velocity", 10, std::bind(&Adapter::callback_Velocity, this, std::placeholders::_1)
	);
	m_pSubscriptionOpCom = this->create_subscription<ros_alate_interfaces::msg::OpCom>(
		"alate_input_operator_command", 10, std::bind(&Adapter::callback_OpCom, this, std::placeholders::_1)
	);

	/***	ROS Publishers	***/

	// TODO: find an elegant way to share this adapter's dispatcher and promise between threads - this is not the way to do it.
	// This works but if the node is interrupted and the dispatcher thread lives beyond the point where the adapter gets destructed, bad things happen.
	m_pThreadDispatcher = new boost::thread(boost::ref(*this));
}

Adapter::~Adapter()
{
	RCLCPP_INFO(this->get_logger(), "Shutting down");
	if (!m_bInterrupted)
	{
		m_pThreadDispatcher->join(); // wait for thread to gently close down
		RCLCPP_INFO(this->get_logger(), "Dispatcher thread joined");
	}
	else
	{
		m_pThreadDispatcher->detach(); // We'll see you in hell!
		RCLCPP_INFO(this->get_logger(), "Dispatcher thread detached");
	}
	delete m_pPublisherVelocity;
	RCLCPP_DEBUG(this->get_logger(), "There goes the m_pPublisherVelocity");
	delete m_pPublisherOpCom;
	RCLCPP_DEBUG(this->get_logger(), "There goes the m_pPublisherOpCom");
	delete m_pThreadDispatcher;
	RCLCPP_DEBUG(this->get_logger(), "There goes the dispatcher thread");

	RCLCPP_DEBUG(this->get_logger(), "Finished");
}

int Adapter::operator()()
{
	int nResult(0);
	RCLCPP_INFO(this->get_logger(), "Running the dispatcher");
	try
	{
		m_pDispatcher->Dispatch();
	}
	catch (std::exception& e)
	{
		RCLCPP_ERROR(this->get_logger(), "Dispatcher error: '%s'", e.what());
		nResult = 1;
	}
	RCLCPP_DEBUG(this->get_logger(), "Dispatcher stopped");
	m_promiseDoneDispatching.set_value();
	RCLCPP_INFO(this->get_logger(), "Dispatcher Done");
	return nResult;
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
