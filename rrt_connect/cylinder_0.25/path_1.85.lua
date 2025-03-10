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
setQ({0.907772 , -0.996067 , -1.21138 , -1.29906 , 0.646678 , 0.739795})
setQ({0.763402 , -0.861496 , -1.05389 , -1.36654 , 0.476734 , 0.875812})
setQ({-0.450531 , -0.952112 , -0.905134 , -0.187559 , 1.09726 , 0.497041})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
