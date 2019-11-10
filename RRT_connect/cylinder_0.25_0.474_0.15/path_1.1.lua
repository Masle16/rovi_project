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
setQ({1.67045 , -1.64138 , -1.26065 , -1.49412 , 1.14129 , -0.22259})
setQ({1.5107 , -1.62649 , -1.28 , -1.42802 , 1.18795 , -0.239609})
setQ({0.542642 , -1.53628 , -1.39724 , -1.02751 , 1.4707 , -0.342733})
setQ({-0.425412 , -1.44606 , -1.51448 , -0.627005 , 1.75344 , -0.445857})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
