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
setQ({1.40031 , -1.0809 , -2.30617 , 0.0621441 , 1.66818 , 1.19088})
setQ({0.900931 , -1.10622 , -2.17648 , 0.0419295 , 1.28748 , 0.717137})
setQ({-0.288246 , -1.16653 , -1.86765 , -0.00620777 , 0.380922 , -0.410996})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
