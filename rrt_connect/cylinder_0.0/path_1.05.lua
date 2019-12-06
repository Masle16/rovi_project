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
setQ({1.63608 , -1.39598 , -1.82153 , -1.32978 , 1.81309 , -0.619625})
setQ({0.872957 , -1.22099 , -1.69859 , -1.18587 , 1.29412 , -0.484471})
setQ({0.0449148 , -1.03111 , -1.56519 , -1.02972 , 0.731 , -0.337818})
setQ({-0.403412 , -1.31171 , -1.91824 , -0.470204 , 1.31507 , -0.128243})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
