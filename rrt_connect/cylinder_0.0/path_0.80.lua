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
setQ({2.14981 , -1.83435 , -1.73514 , -1.43247 , 1.02385 , 0.0814271})
setQ({1.65858 , -1.76263 , -1.68673 , -1.92585 , 0.639964 , 0.101926})
setQ({1.42622 , -1.82578 , -1.66113 , -1.3388 , 1.06099 , -0.0109133})
setQ({0.644112 , -1.81652 , -1.58114 , -1.23755 , 1.10114 , -0.110722})
setQ({-0.137995 , -1.80727 , -1.50115 , -1.1363 , 1.14128 , -0.21053})
setQ({-0.920101 , -1.79801 , -1.42116 , -1.03505 , 1.18143 , -0.310338})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
