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
setQ({2.01404 , -1.45891 , -0.843725 , -1.62643 , 0.439446 , -0.84766})
setQ({1.95747 , -1.45836 , -0.819556 , -1.63034 , 0.445097 , -0.872224})
setQ({0.343591 , -1.44281 , -0.13001 , -1.7421 , 0.606336 , -1.57307})
setQ({0.240418 , -1.49413 , -1.75566 , -1.76954 , 1.26354 , -0.851046})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
