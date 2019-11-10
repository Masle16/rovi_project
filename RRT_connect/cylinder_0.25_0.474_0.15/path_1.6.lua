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
setQ({1.04445 , -1.42142 , -0.986349 , -1.64998 , 1.25679 , 0.511079})
setQ({0.665853 , -1.24245 , -0.354923 , -2.06911 , 1.07336 , 0.809428})
setQ({0.00181177 , -1.61496 , -0.69394 , -1.4683 , 0.289499 , -0.133944})
setQ({-0.0110081 , -1.45005 , -1.92826 , -0.649259 , 0.70335 , -0.542601})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
