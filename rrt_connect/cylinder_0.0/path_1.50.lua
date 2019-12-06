wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("UR-6-85-5-A")
gripper = wc:findFrame("Tool")
bottle = wc:findFrame("Cylinder")
table = wc:findFrame("Table")

function setQ(q)
qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])
device:setQ(qq,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

function attach(obj, tool)
rw.gripFrame(obj, tool, state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.5305 , -1.30245 , -1.05826 , -0.171029 , 2.10198 , 0.435934})
setQ({1.63131 , -1.42339 , -1.28638 , -0.357229 , 1.9724 , 0.329546})
setQ({0.238657 , -1.61069 , -1.63969 , -0.645614 , 1.7717 , 0.164773})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
