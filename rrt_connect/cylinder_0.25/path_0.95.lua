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

setQ({1.693 , -1.728 , -2.068 , -0.932 , 1.571 , 0})
attach(bottle, gripper)
setQ({1.61627 , -1.64191 , -1.5 , -1.39926 , 1.00889 , 0.179648})
setQ({1.05978 , -1.67883 , -1.44352 , -1.36304 , 1.1417 , 0.0641799})
setQ({0.161693 , -1.73842 , -1.35237 , -1.30459 , 1.35602 , -0.122167})
setQ({-0.736397 , -1.798 , -1.26123 , -1.24614 , 1.57035 , -0.308515})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
