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
setQ({2.26021 , -1.29619 , -1.09817 , -1.8032 , 0.749719 , -0.463259})
setQ({1.81984 , -1.36091 , -1.21358 , -1.69109 , 0.85565 , -0.403507})
setQ({0.332921 , -1.57946 , -1.60329 , -1.31254 , 1.21332 , -0.201753})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
