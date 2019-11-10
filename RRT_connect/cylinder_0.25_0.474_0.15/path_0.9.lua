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
setQ({1.78015 , -1.42885 , -1.39937 , -0.652702 , 1.71723 , 0.407999})
setQ({1.00143 , -1.57043 , -1.41374 , -0.757907 , 1.52143 , 0.215757})
setQ({0.171435 , -1.72133 , -1.42905 , -0.870039 , 1.31274 , 0.0108587})
setQ({-0.658557 , -1.87222 , -1.44436 , -0.98217 , 1.10405 , -0.19404})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
