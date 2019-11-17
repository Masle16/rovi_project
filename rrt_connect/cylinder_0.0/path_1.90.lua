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
setQ({0.562673 , -1.26283 , -1.49333 , -1.36627 , 1.11135 , 0.190285})
setQ({-1.05965 , -0.730654 , -0.999655 , -1.81754 , 0.651692 , 0.38057})
setQ({-2.31131 , -0.320074 , -0.618779 , -2.1657 , 0.297062 , 0.527378})
setQ({-1.49127 , -0.735966 , -1.40417 , -2.9714 , -0.716394 , 1.21404})
setQ({-1.03714 , -1.86326 , -0.957333 , -2.76814 , -0.15827 , -0.0431428})
setQ({0.109184 , -2.01591 , -1.21613 , -1.69246 , 0.79175 , -0.425281})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
