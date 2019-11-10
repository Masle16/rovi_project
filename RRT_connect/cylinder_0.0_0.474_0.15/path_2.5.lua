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
setQ({1.97905 , -1.79264 , -0.0876551 , -1.81202 , 1.8335 , 1.31397})
setQ({1.80522 , -1.7866 , 0.0984365 , -1.93164 , 1.67408 , 1.26479})
setQ({0.476858 , -1.74045 , 1.52049 , -2.84576 , 0.45577 , 0.88898})
setQ({-0.652717 , -2.19052 , -0.180319 , -1.6393 , 0.16812 , 0.305471})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
