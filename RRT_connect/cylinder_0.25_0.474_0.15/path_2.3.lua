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
setQ({2.12789 , -1.21633 , -0.180933 , -1.66189 , 1.78242 , -0.837021})
setQ({1.32421 , -1.35745 , -0.754411 , -1.56801 , 1.1028 , -0.126808})
setQ({0.00717274 , -1.58872 , -1.69421 , -1.41417 , -0.0109384 , 1.03707})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
