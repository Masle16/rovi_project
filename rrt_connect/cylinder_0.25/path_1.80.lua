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
setQ({2.14561 , -1.3196 , -0.720546 , -1.43556 , 1.48809 , -0.890098})
setQ({0.576939 , -1.42578 , -0.208679 , -1.59042 , 0.983099 , -0.849948})
setQ({-1.04931 , -1.53586 , 0.321979 , -1.75096 , 0.459568 , -0.808325})
setQ({-1.64396 , -1.81215 , -0.806094 , -1.29269 , 0.411346 , 0.342833})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
