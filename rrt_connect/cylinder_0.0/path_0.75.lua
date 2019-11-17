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
setQ({2.07435 , -1.51058 , -1.55509 , -1.13635 , 1.45997 , 0.470626})
setQ({1.76996 , -1.53768 , -1.59638 , -1.11727 , 1.47044 , 0.426252})
setQ({1.03897 , -1.60276 , -1.69553 , -1.07145 , 1.49558 , 0.319689})
setQ({0.307979 , -1.66784 , -1.79469 , -1.02563 , 1.52072 , 0.213126})
setQ({-0.42301 , -1.73292 , -1.89384 , -0.979817 , 1.54586 , 0.106563})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
