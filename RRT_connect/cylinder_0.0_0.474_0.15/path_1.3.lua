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
setQ({1.52606 , -1.691 , -2.00236 , -1.34304 , 2.04061 , 0.917058})
setQ({2.06577 , -1.45232 , -1.31895 , -2.12621 , 2.29399 , 0.473051})
setQ({1.1262 , -1.59086 , -1.63453 , -1.65673 , 2.05546 , 0.444074})
setQ({0.0464716 , -1.75007 , -1.99719 , -1.11722 , 1.78135 , 0.410775})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
