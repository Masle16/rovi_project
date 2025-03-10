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
setQ({0.94274 , -1.59537 , -1.2481 , -1.83432 , 1.33479 , -0.18032})
setQ({0.35776 , -1.65191 , -1.45592 , -1.58314 , 1.40069 , -0.130012})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
