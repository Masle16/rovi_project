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
setQ({1.62523 , -1.6829 , -1.11639 , -1.56951 , 0.832971 , 0.489087})
setQ({1.43469 , -1.69079 , -1.17649 , -1.52594 , 0.883568 , 0.455557})
setQ({0.140346 , -1.74439 , -1.58474 , -1.22997 , 1.22728 , 0.227778})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
