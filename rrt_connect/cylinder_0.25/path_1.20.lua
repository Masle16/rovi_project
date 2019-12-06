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
setQ({1.9953 , -1.60959 , -1.52592 , -1.45948 , 1.20063 , -0.790777})
setQ({1.10343 , -1.66295 , -1.65819 , -1.31067 , 1.30552 , -0.566832})
setQ({-0.0252831 , -1.73047 , -1.8256 , -1.12233 , 1.43826 , -0.283416})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
