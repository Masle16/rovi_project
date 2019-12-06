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
setQ({1.73916 , -1.74774 , -1.59275 , -1.39495 , 0.850578 , 0.375213})
setQ({1.38194 , -1.7592 , -1.61409 , -1.42608 , 0.777672 , 0.439688})
setQ({0.374568 , -1.7915 , -1.67429 , -1.51389 , 0.572076 , 0.621511})
setQ({-0.632801 , -1.82381 , -1.73448 , -1.6017 , 0.36648 , 0.803334})
setQ({-1.3905 , -1.95822 , -1.60753 , -1.38095 , 0.809327 , 0.303969})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
