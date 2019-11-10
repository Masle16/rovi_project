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
setQ({1.73189 , -1.68264 , -1.70527 , -1.45887 , 1.09441 , 0.0111384})
setQ({1.28096 , -1.6368 , -1.75589 , -1.44216 , 1.19387 , -0.075838})
setQ({0.521686 , -1.55961 , -1.84113 , -1.41404 , 1.36134 , -0.222286})
setQ({-0.237585 , -1.48243 , -1.92637 , -1.38591 , 1.52881 , -0.368735})
setQ({-0.996856 , -1.40524 , -2.0116 , -1.35779 , 1.69628 , -0.515183})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
