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
setQ({2.50757 , -1.46226 , -0.653208 , -2.34989 , 1.48584 , 0.591657})
setQ({2.50374 , -1.46261 , -0.654607 , -2.34842 , 1.48593 , 0.591038})
setQ({0.674872 , -1.63031 , -1.3238 , -1.64121 , 1.52846 , 0.295519})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
