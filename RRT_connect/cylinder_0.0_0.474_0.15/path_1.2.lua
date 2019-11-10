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
setQ({2.19143 , -1.3893 , -1.15088 , -0.500195 , 0.973093 , 0.216133})
setQ({1.10251 , -1.52233 , -1.42499 , -0.641396 , 1.16771 , 0.145783})
setQ({-0.0257435 , -1.66016 , -1.70899 , -0.787698 , 1.36935 , 0.0728916})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
