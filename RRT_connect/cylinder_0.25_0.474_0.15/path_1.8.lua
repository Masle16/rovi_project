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
setQ({0.51381 , -1.308 , -0.960678 , -1.49722 , 1.91389 , -0.0994955})
setQ({0.478609 , -1.71454 , -0.101008 , -1.70219 , 0.708061 , -0.651513})
setQ({-0.108352 , -2.1034 , -1.47842 , -1.18093 , 0.257282 , -0.0415251})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
