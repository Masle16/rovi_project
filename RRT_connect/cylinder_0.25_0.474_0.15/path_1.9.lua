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
setQ({1.16966 , -1.44762 , -0.600437 , -1.66151 , 1.2037 , 0.660806})
setQ({0.967776 , -1.33946 , -0.0343094 , -1.94293 , 1.06202 , 0.915718})
setQ({0.376683 , -1.72783 , -0.876739 , -0.863517 , 1.67571 , -0.0107465})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
