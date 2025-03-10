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
setQ({2.56134 , -1.28869 , -2.02084 , -2.16401 , 0.912746 , 1.16075})
setQ({1.86439 , -1.20414 , -1.92423 , -1.88324 , 0.838781 , 0.664295})
setQ({0.337138 , -1.01885 , -1.71251 , -1.26798 , 0.676699 , -0.423603})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
