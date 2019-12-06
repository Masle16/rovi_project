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
setQ({2.21728 , -1.37469 , -1.15914 , -1.10353 , -0.0322018 , -0.21633})
setQ({0.743869 , -1.19234 , -1.1645 , -1.00246 , 0.550417 , 0.547818})
setQ({-0.794932 , -1.00189 , -1.17009 , -0.896897 , 1.15889 , 1.34588})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
