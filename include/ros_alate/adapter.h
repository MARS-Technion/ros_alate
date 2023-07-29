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

#ifndef _ROS_ALATE_ADAPTER_H_
#define _ROS_ALATE_ADAPTER_H_

#include <memory>
#include <functional>
#include <future>

#include <boost/thread/thread.hpp>

#include <NeMALA/Dispatcher.h>
#include <NeMALA/Publisher.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_alate_interfaces/msg/op_com.hpp"
#include "ros_alate_interfaces/OpComMessage.hpp"
#include "ros_alate_interfaces/Vector6Message.hpp"

namespace ros_alate {

class Adapter : public rclcpp::Node
{
public:
	/* Constructor:
	 * strNodeName: ROS node name
	 */
	Adapter(std::string strNodeName = "ros_alate_adapter");
	~Adapter();
	// The Adapter is callable so it can spin and dispatch on different threads.
	int operator()();
	// The Adapter spins as long as the dispatcher dispatches. A shared future synchronizes the threads with the promise that the dispatcher will eventually stop.
	std::shared_future<void> GetFutureDoneDispatching(){return m_promiseDoneDispatching.get_future().share();}
	// ...but if the promise is broken, the dispatcher is interrupted, just in time to terminate the node.
	void InterruptDispatcherThread(){m_bInterrupted = true;}

private:

	/********* Callbacks *********/
	
	void callback_Velocity(const geometry_msgs::msg::Twist &msg) const;
	void callback_OpCom(const ros_alate_interfaces::msg::OpCom &msg) const;
	
	/********* Attributes *********/

	bool m_bInterrupted;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_pSubscriptionVelocity;
	rclcpp::Subscription<ros_alate_interfaces::msg::OpCom>::SharedPtr m_pSubscriptionOpCom;
	NeMALA::Publisher* m_pPublisherOpCom;
	NeMALA::Publisher* m_pPublisherVelocity;
	std::shared_ptr<NeMALA::Dispatcher> m_pDispatcher;
	boost::thread* m_pThreadDispatcher;
	std::promise<void> m_promiseDoneDispatching;
};

}  // namespace ros_alate

#endif // _ROS_ALATE_ADAPTER_H_
