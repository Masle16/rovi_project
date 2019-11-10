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
setQ({2.3157 , -1.14192 , -2.02647 , -1.47853 , -0.630523 , 0.60962})
setQ({1.54153 , -1.28686 , -1.31285 , -1.00656 , -0.638061 , 0.276934})
setQ({-0.0585383 , -1.58644 , 0.162093 , -0.0310863 , -0.653642 , -0.410672})
setQ({-0.995453 , -2.26007 , -0.198681 , -1.08881 , 0.404462 , 1.18617})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
