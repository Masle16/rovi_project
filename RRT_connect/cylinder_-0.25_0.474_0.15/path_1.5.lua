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

setQ({2.5 , -2.099 , -1.593 , -0.991 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.39511 , -1.41185 , -0.729099 , -0.39767 , 0.937689 , 0.517087})
setQ({1.56815 , -1.50183 , -1.0236 , -0.522638 , 1.08525 , 0.396603})
setQ({0.207073 , -1.64991 , -1.5083 , -0.728319 , 1.32813 , 0.198301})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
