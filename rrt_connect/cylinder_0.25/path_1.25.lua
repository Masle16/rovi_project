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
setQ({1.65363 , -1.49393 , -1.48605 , -1.33974 , 1.47258 , -0.995769})
setQ({1.1356 , -1.55003 , -1.57959 , -1.26488 , 1.49074 , -0.812041})
setQ({-0.00920014 , -1.67402 , -1.78629 , -1.09944 , 1.53087 , -0.406021})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
