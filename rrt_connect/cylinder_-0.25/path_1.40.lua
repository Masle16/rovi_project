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
setQ({2.34866 , -1.32285 , -1.18717 , -1.03121 , 1.58896 , 1.08076})
setQ({1.82884 , -1.34989 , -1.2015 , -0.889263 , 1.52395 , 0.404354})
setQ({0.990215 , -1.39352 , -1.22462 , -0.660256 , 1.41906 , -0.686895})
setQ({0.174134 , -1.7874 , -1.80421 , -0.679092 , 1.86659 , 0.0891905})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
