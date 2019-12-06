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
setQ({1.62518 , -1.24989 , -1.3934 , -1.60025 , 1.56514 , -0.829237})
setQ({1.27859 , -1.31825 , -1.46818 , -1.51716 , 1.56587 , -0.725822})
setQ({0.0622931 , -1.55812 , -1.73059 , -1.22558 , 1.56844 , -0.362911})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
