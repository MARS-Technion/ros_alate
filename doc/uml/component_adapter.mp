input metauml;


Component.adapterIn("Adapter")();
Component.adapterOut("Adapter")();
Component.proxy("NeMALA::Proxy")();

leftToRight(50)(adapterIn, proxy, adapterOut);

beginfig(2);
	drawObjects(
		adapterIn,
		proxy,
		adapterOut
	);
	
	link(providedInterface)(proxy.se -- 0.5[adapterOut.sw, proxy.se]);
	link(requiredInterface)(proxy.ne -- 0.5[adapterIn.ne, proxy.nw]);
	
	link(requiredInterface)(adapterOut.sw  -- 0.5[adapterOut.sw, proxy.se]);
	link(providedInterface)(adapterOut.se + (-10,0) -- adapterOut.se + (-10, -30));
	item(iAssoc)("ROS2")(obj.s = adapterOut.s + (0, -30));
	
	link(providedInterface)(adapterIn.ne  -- 0.5[adapterIn.ne, proxy.nw]);
   	link(requiredInterface)(adapterIn.sw + (10,0) -- adapterIn.sw + (10, -20));
	item(iAssoc)("ROS2")(obj.s = adapterIn.s + (0, -30));
	
endfig;

beginfig(1);
	drawObjects(
		adapterIn,
		proxy
	);
	
	link(providedInterface)(proxy.sw -- 0.5[adapterIn.se, proxy.sw]);
	link(requiredInterface)(proxy.nw -- 0.5[adapterIn.ne, proxy.nw]);
	
	link(requiredInterface)(adapterIn.se  -- 0.5[adapterIn.se, proxy.sw]);
	link(providedInterface)(adapterIn.se + (-10,0) -- adapterIn.se + (-10, -30));
	item(iAssoc)("ROS2")(obj.s = adapterIn.s + (0, -30));
	
	link(providedInterface)(adapterIn.ne  -- 0.5[adapterIn.ne, proxy.nw]);
   	link(requiredInterface)(adapterIn.sw + (10,0) -- adapterIn.sw + (10, -20));
	
endfig;

end
