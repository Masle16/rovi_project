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
setQ({1.19165 , -1.06881 , -1.98396 , -0.836397 , 0.321355 , 0.397626})
setQ({0.214964 , -0.354798 , -1.98098 , -0.759111 , -0.907332 , 0.788584})
setQ({-0.211779 , -1.49178 , -1.8482 , -1.29453 , 0.128837 , 0.166326})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
