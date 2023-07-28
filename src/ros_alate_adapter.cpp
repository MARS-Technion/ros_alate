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

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<ros_alate::Adapter> pAdapter = std::make_shared<ros_alate::Adapter>();

	RCLCPP_INFO(pAdapter->get_logger(), "Started Spinning");
	std::shared_future<void> futureDoneDispatching(pAdapter->GetFutureDoneDispatching());
	rclcpp::FutureReturnCode eReturnCode(rclcpp::FutureReturnCode::TIMEOUT);
	while(rclcpp::FutureReturnCode::TIMEOUT == eReturnCode)
	{
		eReturnCode = rclcpp::spin_until_future_complete(pAdapter, futureDoneDispatching,  std::chrono::milliseconds(100));
	}
	if (rclcpp::FutureReturnCode::INTERRUPTED == eReturnCode)
	{
		RCLCPP_INFO(pAdapter->get_logger(), "Spinning INTERRUPTED!");
		pAdapter->InterruptDispatcherThread();
	}
	else
	{
		RCLCPP_INFO(pAdapter->get_logger(), "Stopped Spinning");
	}
	rclcpp::shutdown();
	return 0;
}
