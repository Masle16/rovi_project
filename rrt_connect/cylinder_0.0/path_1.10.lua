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
setQ({1.65247 , -1.59556 , -1.62552 , -1.46018 , 0.949668 , 0.269621})
setQ({0.928765 , -1.64776 , -1.72028 , -1.32449 , 1.10989 , 0.200094})
setQ({-0.112617 , -1.72288 , -1.85664 , -1.12925 , 1.34045 , 0.100047})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
