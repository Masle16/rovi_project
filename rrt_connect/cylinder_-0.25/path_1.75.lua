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
setQ({2.63938 , -1.26112 , -1.91286 , -0.146655 , 0.580415 , 0.73793})
setQ({1.53404 , -1.68362 , -1.7496 , -0.544151 , 1.42902 , 0.411855})
setQ({0.288381 , -2.15976 , -1.56562 , -0.992108 , 2.38535 , 0.0443843})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
