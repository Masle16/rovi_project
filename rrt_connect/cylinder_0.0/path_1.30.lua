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

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.1364 , -1.27449 , -1.32471 , -1.15992 , 1.85523 , -0.915046})
setQ({1.39975 , -1.3078 , -1.36749 , -1.06621 , 1.3385 , -0.564748})
setQ({0.414154 , -1.35236 , -1.42472 , -0.940843 , 0.647152 , -0.0960703})
setQ({-0.131767 , -1.41994 , -1.33102 , -0.987762 , 1.80922 , 0.0652246})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
