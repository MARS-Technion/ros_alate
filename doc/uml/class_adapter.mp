input metauml;

input class_nemala;
input class_ros2;

Class.adapter("Adapter")()("-CallbackOperatorCommand(msg:ros_alate_interfaces::msg::mission_state&)", "-CallbackVelocity(msg:ros_alate_interfaces::msg::velocity)");
Class.handler_mission_state("HandlerMissionState")()();

leftToRight(400)(ros_subscription, ros_publisher);
topToBottom(50)(ros_node, adapter);
topToBottom(100)(adapter, dispatcher);
leftToRight(50)(adapter, handler_mission_state);

adapter.s = 0.5*(ros_subscription.e + ros_publisher.w) + (0, 50);

beginfig(1);

	drawObjects(
		dispatcher,
		handler,
		publisher,
		msg,
		msg_pt,
		ros_node,
		adapter,
		ros_publisher,
		ros_subscription,
		handler_mission_state
	);

	link(inheritance)(msg_pt.n -- msg.s);
	link(aggregation)(handler.n -- dispatcher.se + (-10, 0));
	item(iAssoc)("1")(obj.nw = dispatcher.se + (-10,-10));
	item(iAssoc)("*")(obj.s = handler.n);
	link(aggregation)(publisher.n -- dispatcher.sw + (10, 0));
	item(iAssoc)("1")(obj.nw = dispatcher.sw + (10, -10));
	item(iAssoc)("*")(obj.s = publisher.n);

	link(composition)(dispatcher.n -- adapter.s);
	link(inheritance)(adapter.n -- ros_node.s);

	link(composition)(ros_publisher.n -- adapter.se);
		item(iAssoc)("1")(obj.sw = adapter.se);
		item(iAssoc)("*")(obj.s = ros_publisher.n);
	link(composition)(ros_subscription.n -- adapter.sw);
		item(iAssoc)("1")(obj.se = adapter.sw);
		item(iAssoc)("*")(obj.s = ros_subscription.n);
	
	link(composition)(0.5* (publisher.n + publisher.nw) -- 0.5* (adapter.sw + adapter.s) );
		item(iAssoc)("1")(obj.nw = 0.5* (adapter.sw + adapter.s));
		item(iAssoc)("*")(obj.se = 0.5* (publisher.n + publisher.nw));

	
	link(dependency)(0.5* (handler.n + handler.ne) -- msg_pt.s);
	link(dependency)(0.5* (publisher.n + publisher.ne) -- msg.sw);

	link(composition)(handler_mission_state.w -- adapter.e);
	link(inheritance)(handler_mission_state.se + (-10, 0) -- handler_mission_state.se + (-10, -255) -- handler.e );
	link(aggregation)(0.5* (ros_publisher.n + ros_publisher.ne) -- handler_mission_state.s);
	
endfig;

end
