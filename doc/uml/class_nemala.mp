Class.dispatcher("NeMALA::Dispatcher")()();
AbstractClass.handler("NeMALA::Handler")()(abstract "+Handle(ptMsgBody:MessagePropertyTree)");
Class.publisher("NeMALA::Publisher")("-m_bMuted:bool")("+Publish(BaseMessage& msg)", "+Mute(bool bMuted)");
AbstractClass.msg_pt("NeMALA::MessagePropertyTree")()();
AbstractClass.msg("NeMALA::BaseMessage")("-m_unTopic:uint","-m_tTime:string")();

topToBottom(50)(msg, msg_pt);
leftToRight(100)(dispatcher, msg);
leftToRight(100)(publisher, handler);
publisher.ne = dispatcher.sw + (-10, -100);
