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
setQ({1.98421 , -1.37408 , -1.12688 , -1.71355 , 1.74127 , 0.0511023})
setQ({1.21454 , -1.3697 , -1.26894 , -1.48515 , 1.27661 , 0.387532})
setQ({0.249485 , -1.3642 , -1.44707 , -1.19878 , 0.693984 , 0.809368})
setQ({-0.168144 , -1.73178 , -1.40887 , -1.26594 , 1.24224 , -0.163476})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
