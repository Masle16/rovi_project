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
setQ({2.21542 , -2.23508 , -0.119542 , -0.662807 , 2.19469 , 0.350316})
setQ({1.74286 , -2.17378 , -0.382291 , -0.700841 , 2.10722 , 0.301185})
setQ({0.294431 , -1.98589 , -1.18765 , -0.817421 , 1.83911 , 0.150592})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
