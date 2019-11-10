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
setQ({1.61128 , -1.41219 , -1.47802 , -2.3114 , 1.17494 , 0.220112})
setQ({1.09909 , -1.07044 , -1.02363 , -3.55802 , 0.821352 , 0.416615})
setQ({0.0015852 , -1.96097 , -1.07367 , -2.81288 , 0.320907 , 0.706866})
setQ({-0.291362 , -1.43058 , -1.42554 , -2.40824 , 1.66963 , 1.35204})
setQ({-1.77085 , -1.57509 , -1.59694 , -2.29166 , 1.90589 , 0.589523})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
