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
setQ({0.88969 , -1.43461 , -1.14873 , -1.56385 , 2.12635 , -0.0132253})
setQ({0.383202 , -1.52467 , -1.35796 , -1.40775 , 1.98872 , -0.00994767})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
