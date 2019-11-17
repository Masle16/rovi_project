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
setQ({1.59578 , -0.960402 , -1.80053 , -1.48883 , 0.744841 , -0.16492})
setQ({1.18819 , -1.08456 , -1.82906 , -1.40659 , 0.867299 , -0.140475})
setQ({0.0170944 , -1.44128 , -1.91103 , -1.1703 , 1.21915 , -0.0702374})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
