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
setQ({2.21417 , -1.36524 , -1.72926 , -1.03585 , 0.875754 , -0.562094})
setQ({2.041 , -1.42386 , -1.65743 , -1.11097 , 0.940562 , -0.409645})
setQ({1.26475 , -1.68664 , -1.33544 , -1.44769 , 1.23105 , 0.273689})
setQ({0.488506 , -1.94942 , -1.01346 , -1.78441 , 1.52155 , 0.957023})
setQ({-0.191634 , -1.65366 , -1.66317 , -1.38097 , 1.44382 , 0.410215})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
