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
setQ({1.85695 , -1.46824 , -1.56591 , -1.12074 , 0.20645 , -0.0770129})
setQ({1.53791 , -1.50318 , -1.61116 , -1.10095 , 0.351038 , -0.0688526})
setQ({0.191953 , -1.65059 , -1.80208 , -1.01748 , 0.961019 , -0.0344263})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
