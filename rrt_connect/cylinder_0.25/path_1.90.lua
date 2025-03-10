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
setQ({1.94276 , -1.40692 , -1.53002 , -1.59825 , 0.635264 , -1.35485})
setQ({0.698148 , -1.80021 , -1.20932 , -0.95149 , 0.551089 , -0.518868})
setQ({-0.683164 , -2.2367 , -0.853398 , -0.233691 , 0.457669 , 0.408931})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
