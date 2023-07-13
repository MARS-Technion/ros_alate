input metauml;

input class_nemala;
input class_ros2;

Class.adapter("Adapter")()("-CallbackOperatorCommand", "-CallbackVelocity");
%Class.nemala_ros_publisher(abstract "NeMALA::RosPublisher")()();
%Class.nemala_ros_handler("NeMALA::RosHandler")()();

%topToBottom(50)(publisher, nemala_ros_handler);
%topToBottom(50)(handler, nemala_ros_publisher);

leftToRight(200)(ros_subscription, ros_publisher);
topToBottom(50)(ros_node, adapter);
topToBottom(100)(adapter, dispatcher);

%adapter.n = 0.75*dispatcher.s + 0.25*ros_node.s + (0, -200);
adapter.s = 0.5*(ros_subscription.e + ros_publisher.w) + (0, 50);
%ros_node.n = handler.sw + (0,-20);


beginfig(1);

    drawObjects(dispatcher,
    				handler,
    				publisher,
    				msg,
    				msg_pt,
    				ros_node,
    				adapter,
    				ros_publisher,
    				ros_subscription
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
    %link(inheritance)(nemala_ros_publisher.n -- handler.s);
    %link(inheritance)(nemala_ros_handler.n -- publisher.s);
    	
    link(composition)(ros_publisher.n -- adapter.se);
    	item(iAssoc)("1")(obj.sw = adapter.se);
	item(iAssoc)("*")(obj.sw = ros_publisher.n);
	link(composition)(ros_subscription.n -- adapter.sw);
    	item(iAssoc)("1")(obj.se = adapter.sw);
	item(iAssoc)("*")(obj.se = ros_subscription.n);
	
	link(composition)(0.5* (publisher.n + publisher.nw) -- 0.5* (adapter.sw + adapter.s) );
    	item(iAssoc)("1")(obj.nw = 0.5* (adapter.sw + adapter.s));
    	item(iAssoc)("*")(obj.se = 0.5* (publisher.n + publisher.nw));
	
	
	link(dependency)(0.5* (handler.n + handler.ne) -- msg_pt.s);
	link(dependency)(0.5* (publisher.n + publisher.ne) -- msg.sw);

   	%link(aggregation)(ros_publisher.n -- nemala_ros_publisher.s);
    	%item(iAssoc)("1")(obj.nw = nemala_ros_publisher.s);
	%item(iAssoc)("1")(obj.se = ros_publisher.n);
	
endfig;

end
