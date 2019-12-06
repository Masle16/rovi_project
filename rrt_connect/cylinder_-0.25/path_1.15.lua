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
setQ({1.99595 , -1.42099 , -1.54303 , -0.26388 , 1.72146 , 0.234295})
setQ({1.3665 , -1.52767 , -1.45465 , -0.40716 , 1.53665 , 0.450449})
setQ({0.359574 , -1.69833 , -1.31328 , -0.636363 , 1.241 , 0.796225})
setQ({-0.647346 , -1.86899 , -1.17191 , -0.865565 , 0.945356 , 1.142})
setQ({-1.29945 , -1.80237 , -1.30327 , -1.52923 , 1.10188 , 0.501209})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
