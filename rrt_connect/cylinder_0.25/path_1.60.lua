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
setQ({1.35817 , -1.2537 , -0.94703 , -1.07948 , 1.72279 , -0.959982})
setQ({1.17259 , -0.990834 , -0.32575 , -1.16122 , 1.80691 , -1.49204})
setQ({-0.0849095 , -1.77488 , 0.0768415 , -0.945793 , 1.75035 , -1.1018})
setQ({-1.37013 , -1.63404 , -0.819953 , -0.67877 , 1.79472 , -0.997583})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
