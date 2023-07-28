input metauml;

input class_nemala;
input class_ros2;

Class.adapter("Adapter")()("-CallbackOperatorCommand(msg:ros_alate_interfaces::msg::mission_state&)", "-CallbackVelocity(msg:ros_alate_interfaces::msg::velocity)");
Class.handler_mission_state("HandlerMissionState")()();

leftToRight(200)(ros_subscription, ros_publisher, handler_mission_state);
topToBottom(50)(ros_node, adapter);
leftToRight(100)(adapter, dispatcher);
topToBottom(50)(handler, handler_mission_state);

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

	link(composition)(dispatcher.w -- adapter.e);
	link(inheritance)(adapter.n -- ros_node.s);

	link(composition)(ros_publisher.nw -- adapter.s + (10, 0));
		item(iAssoc)("1")(obj.nw = adapter.s);
		item(iAssoc)("*")(obj.s = ros_publisher.nw);
	link(composition)(ros_subscription.n -- 0.5 * (adapter.s + adapter.sw));
		item(iAssoc)("1")(obj.nw = 0.5 * (adapter.s + adapter.sw) + (5, 0));
		item(iAssoc)("*")(obj.se = ros_subscription.n);
	
	link(composition)(0.5* (publisher.n + publisher.nw) -- 0.5* (adapter.se + adapter.s) );
		item(iAssoc)("1")(obj.ne = 0.5* (adapter.se + adapter.s));
		item(iAssoc)("*")(obj.sw = 0.5* (publisher.n + publisher.nw));

	
	link(dependency)(0.5* (handler.n + handler.ne) -- msg_pt.s);
	link(dependency)(0.5* (publisher.n + publisher.ne) -- msg.sw);

	link(composition)(handler_mission_state.nw -- adapter.se);
	link(inheritance)(handler_mission_state.n -- handler.s );
	link(aggregation)(ros_publisher.e -- handler_mission_state.w);
	
endfig;

end
