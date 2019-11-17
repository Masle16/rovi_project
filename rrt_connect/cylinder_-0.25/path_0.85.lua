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

setQ({2.5 , -2.099 , -1.593 , -0.991 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.3805 , -1.93033 , -1.21017 , -1.20853 , 1.38459 , 0.671675})
setQ({1.81629 , -1.93233 , -1.29568 , -1.24213 , 1.38786 , 0.618966})
setQ({0.980906 , -1.93528 , -1.42228 , -1.2919 , 1.39271 , 0.540923})
setQ({0.145518 , -1.93824 , -1.54888 , -1.34166 , 1.39755 , 0.462881})
setQ({-0.68987 , -1.9412 , -1.67548 , -1.39142 , 1.4024 , 0.384838})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
