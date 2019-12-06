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

setQ({2.5 , -2.099 , -1.593 , -0.991 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.60053 , -1.78023 , -0.662945 , -0.750086 , 1.15912 , -0.0747902})
setQ({1.93626 , -1.78338 , -0.898264 , -0.782625 , 1.23199 , -0.061558})
setQ({0.906173 , -1.78825 , -1.26318 , -0.833083 , 1.34499 , -0.0410387})
setQ({-0.123913 , -1.79313 , -1.62809 , -0.883542 , 1.458 , -0.0205193})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
