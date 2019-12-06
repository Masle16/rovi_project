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
setQ({2.28633 , -1.68495 , -1.24662 , -0.947995 , 0.957462 , -0.533524})
setQ({1.70118 , -1.70417 , -1.37357 , -0.945614 , 1.06182 , -0.44278})
setQ({0.749453 , -1.73545 , -1.58005 , -0.941743 , 1.23154 , -0.295186})
setQ({-0.202274 , -1.76672 , -1.78652 , -0.937871 , 1.40127 , -0.147593})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
