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
setQ({0.607602 , -1.33513 , -1.24827 , -2.1688 , 1.48707 , 0.124714})
setQ({0.569245 , -1.37155 , -1.2594 , -2.11354 , 1.40186 , 0.0783768})
setQ({-0.111146 , -2.01767 , -1.45687 , -1.13336 , -0.109522 , -0.743553})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
