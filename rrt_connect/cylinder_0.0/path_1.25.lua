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
setQ({2.34288 , -1.03947 , -2.33135 , -1.34803 , 0.758436 , -0.0200849})
setQ({1.81937 , -1.2043 , -2.15248 , -1.38263 , 0.830259 , 0.0420649})
setQ({0.702737 , -1.55586 , -1.77097 , -1.45642 , 0.983455 , 0.174628})
setQ({-0.413893 , -1.90742 , -1.38945 , -1.53022 , 1.13665 , 0.307191})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
