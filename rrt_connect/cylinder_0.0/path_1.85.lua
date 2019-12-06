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
setQ({2.23787 , -1.02248 , -1.17842 , -1.06637 , 2.03335 , -1.39012})
setQ({2.09359 , -1.05547 , -1.21308 , -1.06074 , 2.01368 , -1.33099})
setQ({0.469793 , -1.42673 , -1.60304 , -0.997369 , 1.79234 , -0.665493})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
