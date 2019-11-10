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
setQ({2.36797 , -1.3755 , -1.92333 , -0.499385 , 1.1615 , -0.293359})
setQ({1.96819 , -1.42346 , -1.93124 , -0.548719 , 1.20798 , -0.260059})
setQ({1.18764 , -1.5171 , -1.94668 , -0.645039 , 1.29873 , -0.195044})
setQ({0.407093 , -1.61073 , -1.96212 , -0.741359 , 1.38949 , -0.13003})
setQ({-0.373453 , -1.70437 , -1.97756 , -0.83768 , 1.48024 , -0.0650148})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
