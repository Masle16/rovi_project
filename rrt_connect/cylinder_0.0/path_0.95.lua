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
setQ({2.29405 , -1.62755 , -1.29497 , -1.27881 , 1.10475 , -0.18417})
setQ({1.59534 , -1.66209 , -1.43642 , -1.20894 , 1.19923 , -0.14685})
setQ({0.678894 , -1.70739 , -1.62194 , -1.11729 , 1.32316 , -0.0978999})
setQ({-0.237553 , -1.7527 , -1.80747 , -1.02565 , 1.44708 , -0.0489499})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
