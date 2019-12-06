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
setQ({1.2874 , -0.731946 , -1.40013 , -1.0208 , 0.890233 , -0.573133})
setQ({0.325978 , -1.10536 , -1.75048 , -1.23363 , 0.585828 , -0.35984})
setQ({-0.947203 , -1.59985 , -2.21445 , -1.51547 , 0.182716 , -0.0773826})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
