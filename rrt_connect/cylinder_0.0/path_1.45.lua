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
setQ({2.41195 , -1.38152 , -0.970136 , -1.52261 , 1.21506 , -0.591713})
setQ({1.83437 , -1.3521 , -1.0994 , -1.21814 , 1.30632 , -0.364041})
setQ({0.654727 , -1.29202 , -1.36342 , -0.596299 , 1.49271 , 0.100945})
setQ({-0.524912 , -1.23194 , -1.62743 , 0.0255408 , 1.67911 , 0.565931})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
