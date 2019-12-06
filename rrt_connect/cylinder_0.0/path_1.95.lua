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

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.56685 , -1.16643 , -1.29714 , -2.18053 , 1.00841 , -0.931461})
setQ({1.1579 , -1.34545 , -1.39474 , -2.44619 , 0.737423 , -0.914021})
setQ({-0.706748 , -1.58237 , -1.52389 , -2.79776 , 0.378784 , -0.890941})
setQ({-1.70713 , -1.75506 , -1.71009 , -1.55265 , -0.169511 , 0.0504956})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
