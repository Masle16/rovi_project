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
setQ({1.43886 , -1.18274 , -0.985977 , -1.70447 , -0.862615 , -0.598441})
setQ({1.20631 , -1.35495 , -0.80623 , -1.69424 , -0.600839 , -0.352874})
setQ({-0.156204 , -2.36392 , 0.246943 , -1.63429 , 0.932959 , 1.08595})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
