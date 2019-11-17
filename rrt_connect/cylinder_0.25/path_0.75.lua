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
setQ({1.66599 , -1.71492 , -1.65112 , -1.23394 , 1.11787 , -0.302188})
setQ({0.980169 , -1.73203 , -1.49033 , -1.11007 , 1.30709 , -0.290706})
setQ({0.28519 , -1.74937 , -1.3274 , -0.984551 , 1.49883 , -0.279071})
setQ({-0.409789 , -1.76671 , -1.16447 , -0.859029 , 1.69057 , -0.267437})
setQ({-0.869084 , -1.54325 , -1.59296 , -0.582185 , 1.50889 , -0.358836})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
