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
setQ({1.69998 , -1.49812 , -1.6704 , -1.7495 , 1.0099 , -0.126014})
setQ({1.4607 , -1.50401 , -1.54294 , -1.65314 , 1.0026 , -0.219704})
setQ({0.591274 , -1.52538 , -1.07981 , -1.30303 , 0.976074 , -0.560121})
setQ({-0.296714 , -1.55603 , -1.551 , -1.04765 , 1.2115 , -0.281041})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
