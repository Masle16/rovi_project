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
setQ({1.63273 , -1.40634 , -1.41243 , -1.4269 , 1.40459 , -0.0227849})
setQ({1.42261 , -1.43587 , -1.45621 , -1.38973 , 1.41713 , -0.0210669})
setQ({0.563739 , -1.55658 , -1.63514 , -1.23782 , 1.46842 , -0.0140446})
setQ({-0.295131 , -1.67729 , -1.81407 , -1.08591 , 1.51971 , -0.0070223})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
